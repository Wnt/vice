/*
 * soundfifo.c - Raw stereo PCM sound device, for a FIFO with a reader on the
 *               other end
 *
 * Written by
 *  osgallery lab
 *
 * This file is part of VICE, the Versatile Commodore Emulator.
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
 *  A consumer reading a NAMED PIPE has no file to seek in and no header to
 *  parse: it opens the pipe, reads bytes, and plays them.  The two existing
 *  sinks each get half of that right.  `wav` writes the right FORMAT --
 *  stereo, whatever rate you ask for -- but opens the stream with a 44-byte
 *  RIFF header, and a consumer that expects raw PCM decodes "RIFF....WAVEfmt "
 *  as eleven full-scale 16-bit stereo frames: an audible CLICK at the start of
 *  every run, which on an exhibit that relaunches to reset is a click on every
 *  reset.  `fs` writes raw PCM but forces *channels = 1, which halves the byte
 *  rate a stereo consumer is clocked at.
 *
 *  This device is the intersection: raw PCM, no header, up to two channels.
 *  It is otherwise a copy of `wav`'s behaviour, deliberately -- including the
 *  ONE property that decides whether the emulator runs at full speed:
 *
 *  NOT A TIMING SOURCE.  A sound device flagged is_timing_source=true makes
 *  the sink's own write pacing the emulator's clock.  Point such a device at a
 *  pipe and the PIPE becomes the clock: VICE's `sdl` device is flagged that
 *  way, and measured 24 % speed through a FIFO.  This one is flagged false,
 *  like `wav` and `fs`, so the emulator keeps its own timing and a blocking
 *  write is nothing but backpressure.
 *
 *  It is registered as a RECORD device, again like `wav` and `fs`: that is
 *  what makes it selectable as -sounddev while writing to a file/pipe path
 *  given by -soundarg.
 */

#include "vice.h"

#include <stdio.h>

#include "sound.h"
#include "types.h"
#include "archdep.h"

static FILE *fifo_fd = NULL;

static int fifo_init(const char *param, int *speed, int *fragsize, int *fragnr,
                     int *channels)
{
    /* Mono and stereo both pass through untouched; the caller's -soundoutput
       has already decided which, and a consumer clocked at 4 bytes per frame
       must not be handed 2. */
    if (*channels > 2) {
        *channels = 2;
    }

    fifo_fd = fopen(param ? param : "vicesnd.pcm", MODE_WRITE);

    /* No header.  That is the whole point of this file. */
    return !fifo_fd;
}

static int fifo_write(int16_t *pbuf, size_t nr)
{
#ifdef WORDS_BIGENDIAN
    unsigned int i;

    /* The wire is little-endian s16, so swap on a big-endian host and swap
       back, exactly as soundwav.c does -- the buffer belongs to the caller. */
    for (i = 0; i < nr; i++) {
        pbuf[i] = (int16_t)((((uint16_t)pbuf[i] & 0xff) << 8) | ((uint16_t)pbuf[i] >> 8));
    }
#endif

    if (nr != fwrite(pbuf, sizeof(int16_t), nr, fifo_fd)) {
#ifdef WORDS_BIGENDIAN
        for (i = 0; i < nr; i++) {
            pbuf[i] = (int16_t)((((uint16_t)pbuf[i] & 0xff) << 8) | ((uint16_t)pbuf[i] >> 8));
        }
#endif
        return 1;
    }

#ifdef WORDS_BIGENDIAN
    for (i = 0; i < nr; i++) {
        pbuf[i] = (int16_t)((((uint16_t)pbuf[i] & 0xff) << 8) | ((uint16_t)pbuf[i] >> 8));
    }
#endif

    return 0;
}

static void fifo_close(void)
{
    /* Nothing to patch up on close: there is no length field to fix, which is
       also why this sink survives being killed mid-stream. */
    fclose(fifo_fd);
    fifo_fd = NULL;
}

static const sound_device_t fifo_device =
{
    "fifo",
    fifo_init,
    fifo_write,
    NULL,
    NULL,
    NULL,
    fifo_close,
    NULL,
    NULL,
    0,
    2,          /* max_channels: stereo, unlike `fs` */
    false       /* is_timing_source: NEVER true for a pipe */
};

int sound_init_fifo_device(void)
{
    return sound_register_device(&fifo_device);
}
