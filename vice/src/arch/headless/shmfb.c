/** \file   shmfb.c
 * \brief   Publish the headless UI's finished frames into a shared-memory
 *          framebuffer (VICE_SHM_PATH)
 *
 * \author  osgallery lab
 */

/* This file is part of VICE, the Versatile Commodore Emulator.
 * See README for copyright notice.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 *  02111-1307  USA.
 *
 */

/*
 *  WHAT THIS IS, AND WHY IT IS HERE
 *
 *  The headless UI already runs the whole emulator with no window, no SDL and
 *  no X server -- but its video.c is a set of empty stubs, so the finished
 *  frame never leaves the process.  This file gives it one destination: a
 *  file-backed mapping in the wire format an external consumer already speaks
 *  (see WIRE FORMAT below).  No X server, no window, no readback.
 *
 *  THE SEAM is video_canvas_refresh(), the one arch-side function the emulator
 *  core calls when a rectangle of the emulated screen is finished
 *  (raster/raster-canvas.c: refresh_canvas() for the dirty rectangle,
 *  video_canvas_refresh_all() for a whole frame).  Every UI port -- gtk3, sdl2
 *  -- publishes from exactly there, by calling the machine-independent
 *  video_canvas_render() with a 32bpp destination.  We call the same function
 *  with the mapping as the destination.  Nothing in this file names a machine,
 *  a video chip or a driver; the only types it touches are video_canvas_t and
 *  its draw buffer.
 *
 *  DAMAGE COMES FOR FREE.  Unlike a render-layer producer, VICE hands us the
 *  refreshed rectangle in the call itself, so the published dirty rect is the
 *  emulator's own, not a whole-frame lie the consumer has to diff back down.
 *
 *  CONFIGURATION is by environment, so one binary serves every machine and an
 *  unset variable leaves the headless UI byte-for-byte as upstream:
 *
 *    VICE_SHM_PATH   (required)  file to publish into; unset = do nothing
 *    VICE_SHM_CHIP   (optional)  which video chip's canvas to publish, by the
 *                                chip's own resource prefix and case-
 *                                insensitively: VICII, VDC, VIC, TED, CRTC.
 *                                Unset keeps the historical behaviour (the
 *                                first canvas to refresh claims the mapping).
 *    VICE_SHM_HOLD_RESTORE (optional)  publish NOTHING until the machine has
 *                                finished a frame after a snapshot restore.
 *                                Set it when, and only when, the launcher is
 *                                restoring a checkpoint at startup.
 *    VICE_SHM_TRACE  (optional)  log the mapping and a periodic publish count
 *
 *  WIRE FORMAT -- a 64-byte header followed by width*height 32bpp pixels:
 *
 *    off  type  field
 *      0  u32   magic 'IFB1' (0x31424649 little-endian)
 *      4  u32   version (1)
 *      8  u32   width
 *     12  u32   height
 *     16  u32   stride (bytes per row; always width*4 here)
 *     20  u32   bpp (32)
 *     24  u64   sequence (seqlock)
 *     32  u32   dirty_x0    36 u32 dirty_y0
 *     40  u32   dirty_x1    44 u32 dirty_y1
 *     48        pad to 64
 *     64        width*height pixels
 *
 *  Pixels are host-endian XRGB8888 -- B,G,R,X in memory on a little-endian
 *  host, which is byte-identical to the BGRA a video encoder consumes, so
 *  nothing on this path converts a pixel.  That is what shmfb_set_palette()
 *  buys by programming the render tables with red at bit 16, green at bit 8
 *  and blue at bit 0 -- the same layout the SDL2 port selects for an rmask of
 *  0x00ff0000.
 *
 *  SYNCHRONISATION is a seqlock: one producer (the emulation thread, the only
 *  caller of video_canvas_refresh()), any number of readers, readers never
 *  write.  The sequence goes ODD before pixels are touched and EVEN after,
 *  both with release ordering.  A reader takes it with acquire, copies,
 *  re-reads, and retries on odd-or-changed, so a torn frame can never be
 *  accepted.
 *
 *  ONE PUBLISHER PER MAPPING.  A machine with two video chips (x128: VICII and
 *  VDC) has two canvases, and only one may write the mapping or the seqlock's
 *  single-producer premise breaks.
 *
 *  WHICH one must be a CHOICE, not a race.  Without VICE_SHM_CHIP the first
 *  canvas to reach video_canvas_refresh() claims the mapping, and on x128 that
 *  is decided by refresh order -- an 80-column exhibit that wants the VDC gets
 *  it by luck and loses it to any change in when the two chips first paint.
 *  With VICE_SHM_CHIP set, a canvas may claim the mapping only if its
 *  videoconfig->chip_name matches, and if no canvas ever matches the mapping
 *  stays empty, which is loud (the boot gate reads zero bytes) rather than
 *  quietly wrong.
 */

#include "vice.h"

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <unistd.h>

#include "palette.h"
#include "shmfb.h"
#include "video.h"
#include "videoarch.h"
#include "viewport.h"

/** \brief  Fixed header size in bytes; pixels start here. */
#define SHM_HEADER  64
#define SHM_MAGIC   0x31424649U     /* 'IFB1' */
#define SHM_VERSION 1U

static const char *shm_path = NULL;
static const char *shm_chip = NULL;
static int shm_trace = 0;

/** \brief  Hold every frame until the restored scene is the frame we publish.
 *
 * A station that resets by RELAUNCHING and restoring a checkpoint boots the
 * machine first: the ROM paints its power-on screen a character cell at a
 * time, the monitor then reads the snapshot, and only after that is the
 * exhibit's own scene on the canvas.  Published as it happens, that is a
 * half-drawn line of text and a flash of the wrong screen -- the operator's
 * "black dashes across one text row" -- on every single reset.
 *
 * With this set the mapping is left ALONE until the machine finishes a frame
 * after a snapshot restore, so the consumer keeps showing the last frame of
 * the previous session and then cuts, in one seqlock step, to the scene.
 *
 * The bound matters more than the feature: if the restore never happens (no
 * golden, a golden the machine rejects), the hold expires and the exhibit
 * shows the cold boot instead of nothing at all.
 */
#define SHM_HOLD_MAX_FRAMES 600     /* ~10 s at 60 Hz */
static int shm_hold = 0;
static uint64_t shm_held = 0;

/** \brief  The canvas that claimed the mapping (see ONE PUBLISHER above). */
static struct video_canvas_s *shm_owner = NULL;

/** \brief  Case-insensitive chip-name compare, no <strings.h> assumptions.
 *
 * The names are VICE's own resource prefixes -- "VICII", "VDC", "VIC", "TED",
 * "CRTC" -- and an operator writing VICE_SHM_CHIP into a station fixture
 * should not have to guess VICE's capitalisation.
 */
static int shmfb_chip_matches(const char *chip, const char *want)
{
    size_t i;

    for (i = 0; chip[i] != '\0' && want[i] != '\0'; i++) {
        int a = (unsigned char)chip[i];
        int b = (unsigned char)want[i];

        if (a >= 'a' && a <= 'z') {
            a -= 'a' - 'A';
        }
        if (b >= 'a' && b <= 'z') {
            b -= 'a' - 'A';
        }
        if (a != b) {
            return 0;
        }
    }
    return chip[i] == '\0' && want[i] == '\0';
}

static uint8_t *shm_base = NULL;
static size_t shm_bytes = 0;
static unsigned int shm_w = 0;
static unsigned int shm_h = 0;
static uint64_t shm_frames = 0;


void shmfb_init(void)
{
    const char *path = getenv("VICE_SHM_PATH");

    if (path != NULL && *path != '\0') {
        shm_path = path;
        shm_chip = getenv("VICE_SHM_CHIP");
        if (shm_chip != NULL && *shm_chip == '\0') {
            shm_chip = NULL;
        }
        shm_hold = (getenv("VICE_SHM_HOLD_RESTORE") != NULL);
        shm_trace = (getenv("VICE_SHM_TRACE") != NULL);
        if (shm_trace) {
            fprintf(stderr, "[shmfb] publishing to %s\n", shm_path);
        }
    }
}


int shmfb_enabled(void)
{
    return shm_path != NULL;
}


static void shmfb_unmap(void)
{
    if (shm_base != NULL) {
        munmap(shm_base, shm_bytes);
    }
    shm_base = NULL;
    shm_bytes = 0;
    shm_w = 0;
    shm_h = 0;
}


/** \brief  (Re)map whenever the published geometry changes.
 *
 * The keyed comparison is (w, h) and NOT the byte size: 384x272 and 272x384
 * need the same number of bytes and a size-keyed test would keep publishing a
 * transposed frame under a stale header forever.
 *
 * The order below is the one a consumer's validation expects: grow the file
 * first, then zero the header (which makes the magic invalid, so a reader that
 * looks mid-resize sees "not initialised yet" rather than a lie), then write
 * the new geometry.
 */
static int shmfb_ensure_mapped(unsigned int w, unsigned int h)
{
    size_t want;
    uint32_t *hdr;
    void *p;
    int fd;

    if (shm_base != NULL && shm_w == w && shm_h == h) {
        return 1;
    }
    shmfb_unmap();

    want = SHM_HEADER + (size_t)w * (size_t)h * 4;
    fd = open(shm_path, O_RDWR | O_CREAT, 0644);
    if (fd < 0) {
        fprintf(stderr, "[shmfb] cannot open %s: %s\n", shm_path, strerror(errno));
        return 0;
    }
    p = MAP_FAILED;
    if (ftruncate(fd, (off_t)want) == 0) {
        p = mmap(NULL, want, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    }
    close(fd);
    if (p == MAP_FAILED) {
        fprintf(stderr, "[shmfb] cannot map %lu bytes of %s: %s\n",
                (unsigned long)want, shm_path, strerror(errno));
        return 0;
    }

    shm_base = (uint8_t *)p;
    shm_bytes = want;
    shm_w = w;
    shm_h = h;

    memset(shm_base, 0, SHM_HEADER);
    hdr = (uint32_t *)shm_base;
    hdr[0] = SHM_MAGIC;
    hdr[1] = SHM_VERSION;
    hdr[2] = (uint32_t)w;
    hdr[3] = (uint32_t)h;
    hdr[4] = (uint32_t)w * 4;
    hdr[5] = 32;
    if (shm_trace) {
        fprintf(stderr, "[shmfb] mapped %ux%u (%lu bytes) at %s\n",
                w, h, (unsigned long)want, shm_path);
    }
    return 1;
}


void shmfb_set_palette(struct video_canvas_s *canvas)
{
    video_render_color_tables_t *color_tables;
    struct palette_s *palette;
    unsigned int i;

    if (!shmfb_enabled() || canvas == NULL || canvas->palette == NULL) {
        return;
    }
    palette = canvas->palette;
    color_tables = &canvas->videoconfig->color_tables;

    /* Host-endian XRGB8888: red at bit 16, green at bit 8, blue at bit 0, so
     * the bytes in memory are B,G,R,X on a little-endian host. */
    for (i = 0; i < palette->num_entries; i++) {
        palette_entry_t color = palette->entries[i];
        uint32_t color_code = ((uint32_t)color.red << 16)
                            | ((uint32_t)color.green << 8)
                            | (uint32_t)color.blue
                            | 0xff000000U;
        video_render_setphysicalcolor(canvas->videoconfig, (int)i, color_code, 32);
    }

    /* The same layout again for the CRT/raw renderers, which build their
     * output from these per-component tables rather than from the palette. */
    for (i = 0; i < 256; i++) {
        video_render_setrawrgb(color_tables, i, i << 16, i << 8, i);
    }
    video_render_setrawalpha(color_tables, 0xff000000U);
    video_render_initraw(canvas->videoconfig);
}


void shmfb_refresh(struct video_canvas_s *canvas,
                   unsigned int xs, unsigned int ys,
                   unsigned int xi, unsigned int yi,
                   unsigned int w, unsigned int h)
{
    unsigned int pw, ph;
    uint32_t *hdr;
    uint64_t *seq;
    uint64_t start;

    if (!shmfb_enabled() || canvas == NULL || !canvas->created) {
        return;
    }
    /* ONLY FINISHED FRAMES LEAVE THIS PROCESS.  video_canvas_refresh() has two
       kinds of caller: the end-of-frame path, which hands over a frame the
       emulator has just drawn, and everything else -- the monitor's
       refresh-on-break, a viewport resize -- which repaints the canvas while
       the machine is STOPPED, i.e. hands over whatever state the stop caught.
       That second kind is what a visitor sees as the restore's debris: the
       monitor is open for the `undump`, so the frames it forces are the
       cleared canvas at the geometry the restore is passing THROUGH, and on a
       CRTC machine the mapping is resized to publish them. A consumer that
       trusts our damage rect has no way to tell those apart, so they are not
       published at all; the next end-of-frame is 20 ms away. */
    if (!video_canvas_frame_is_complete()) {
        if (shm_trace) {
            fprintf(stderr, "[shmfb] not a finished frame, not published\n");
        }
        return;
    }
    /* HOLD ACROSS THE RESTORE (VICE_SHM_HOLD_RESTORE); see the comment on
       shm_hold.  This is a finished frame, so the only question left is
       whether it is a frame of the RESTORED machine. */
    if (shm_hold) {
        if (video_canvas_repaint_pending()) {
            shm_hold = 0;
            if (shm_trace) {
                fprintf(stderr, "[shmfb] restore complete after %lu held frames,"
                        " publishing again\n", (unsigned long)shm_held);
            }
        } else if (++shm_held >= SHM_HOLD_MAX_FRAMES) {
            shm_hold = 0;
            fprintf(stderr, "[shmfb] no snapshot restore after %lu frames -- "
                    "publishing the machine as it is\n",
                    (unsigned long)shm_held);
        } else {
            return;
        }
    }
    /* One publisher per mapping. With VICE_SHM_CHIP set the owner is CHOSEN by
       chip name; without it, the first canvas to get here owns it. */
    if (shm_owner == NULL) {
        const char *chip = (canvas->videoconfig != NULL)
                           ? canvas->videoconfig->chip_name : NULL;

        if (shm_chip != NULL) {
            if (chip == NULL || !shmfb_chip_matches(chip, shm_chip)) {
                if (shm_trace) {
                    fprintf(stderr, "[shmfb] canvas %p chip %s is not %s, skipping\n",
                            (void *)canvas, chip != NULL ? chip : "(none)", shm_chip);
                }
                return;
            }
        }
        shm_owner = canvas;
        if (shm_trace) {
            fprintf(stderr, "[shmfb] canvas %p (chip %s) owns the mapping\n",
                    (void *)canvas, chip != NULL ? chip : "(none)");
        }
    } else if (shm_owner != canvas) {
        return;
    }

    pw = canvas->draw_buffer->canvas_physical_width;
    ph = canvas->draw_buffer->canvas_physical_height;
    if (pw == 0 || ph == 0) {
        return;
    }
    if (!shmfb_ensure_mapped(pw, ph)) {
        return;
    }

    /* Destination coordinates are in physical pixels; the core hands them to
     * us in canvas pixels (the same conversion every other port does). */
    xi *= (unsigned int)canvas->videoconfig->scalex;
    w *= (unsigned int)canvas->videoconfig->scalex;
    yi *= (unsigned int)canvas->videoconfig->scaley;
    h *= (unsigned int)canvas->videoconfig->scaley;

    if (xi >= pw || yi >= ph) {
        return;
    }
    if (xi + w > pw) {
        w = pw - xi;
    }
    if (yi + h > ph) {
        h = ph - yi;
    }
    if (w == 0 || h == 0) {
        return;
    }

    hdr = (uint32_t *)shm_base;
    seq = (uint64_t *)(shm_base + 24);

    start = *seq;
    __atomic_store_n(seq, start + 1, __ATOMIC_RELEASE);  /* odd: writing */

    video_canvas_render(canvas, shm_base + SHM_HEADER,
                        (int)w, (int)h, (int)xs, (int)ys, (int)xi, (int)yi,
                        (int)(pw * 4));

    /* The emulator's own refreshed rectangle, published as the dirty rect. */
    hdr[8] = xi;
    hdr[9] = yi;
    hdr[10] = xi + w;
    hdr[11] = yi + h;

    __atomic_store_n(seq, start + 2, __ATOMIC_RELEASE);  /* even: complete */

    if (shm_trace && ((++shm_frames % 300) == 0)) {
        fprintf(stderr, "[shmfb] published %lu updates, surface %ux%u\n",
                (unsigned long)shm_frames, pw, ph);
    }
}


void shmfb_shutdown(void)
{
    shmfb_unmap();
    shm_owner = NULL;
}
