/*
 * vicectl.c - vicectl: a unix-socket control channel for headless VICE.
 *
 * Written by
 *  Kernel Hive lab
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
    vicectl/1 - a SOCK_STREAM control channel on VICE_CTL_SOCK, so a host-side
    daemon can drive a HEADLESS VICE (no X server, no XTEST, no SDL) the way
    the Kernel Hive MAME stations are driven by the ctlsock OSD module.

    Shape deliberately copied from that module (protocol section below), so the
    daemon side is the same state machine:

        request   "<seq> VERB args\n"
        reply     "<seq> OK [detail]\n" | "<seq> ERR <code> <text>\n"
                  "<seq> DATA <text>\n" (zero or more, before the OK)
        banner    "vicectl/1 machine=... geometry=WxH keymap=<n> keys=<n>\n"

    THE ONE RULE (inherited): the socket thread parses lines and enqueues; the
    EMULATION thread applies. The socket thread never touches emulator state.
    The emulation-thread drain is vicectl_frame(), called once per emulated
    frame from vsync_do_vsync(). Frame pace is not a compromise here the way it
    was for MAME: every CBM machine scans its keyboard matrix from a raster IRQ,
    so an edge that lands inside a frame is indistinguishable from one that
    lands at its start, and a sub-frame timer would buy nothing.

    KEYS. Two verbs, two levels:

      KEY <0|1> <keysym> [mod]   host-key level. Feeds VICE's own
                                 keyboard_key_pressed()/_released() with an X11
                                 keysym, so the machine's loaded keymap (.vkm)
                                 resolves row/column, virtual shift, deshift,
                                 shift-lock and the C128 alternative set exactly
                                 as it does for a windowed VICE. This is the
                                 verb that makes a de-bridged station behave
                                 like the kiosk it replaces, because it reuses
                                 the very same keymap file.

      MKEY <0|1> <row> <col>     matrix level. keyboard_set_keyarr_any(), i.e.
                                 one bit of the emulated matrix, no keymap
                                 involved. Escape hatch for keys no keymap
                                 names (RESTORE, 40/80, joyport keypad) and for
                                 machines run without a keymap at all.

    Both ride ONE paced queue, because both end in the same matrix.

    PACING (the 2026-08-12 lesson, ported). A browser delivers a typed line as
    ONE burst; the guest scans its own matrix. So:
      - a field's RELEASE waits VICE_CTL_KEY_HOLD ms after its own press (a
        press+release inside one scan is invisible to the KERNAL);
      - a field's RE-PRESS waits VICE_CTL_KEY_GAP ms after its own release (two
        presses would otherwise merge into one);
      - VICE_CTL_KEY_EXCL=1 additionally serializes non-modifier presses: a
        press waits until no other non-modifier key is down. Modifiers are
        exempt by matrix position (key_is_modifier()), because a held shift is a
        level, not a keystroke.
    Pacing is PER KEY, not global: a global slot is the ~6 keys/second ceiling
    that the MAME campaign had to remove.

    Redundant edges COALESCE against the queued state, not the applied one:
    browser auto-repeat resends keydown with no keyup and the CBM KERNAL does
    its own repeat from the held matrix bit.

    ENV (all optional; the gate is the only one that must be set):
      VICE_CTL_SOCK       listener path; unset/empty => this file does nothing
                          at all and the binary behaves exactly like upstream
      VICE_CTL_KEY_HOLD   press->release dwell, ms                       [60]
      VICE_CTL_KEY_GAP    release->re-press dwell, ms                    [60]
      VICE_CTL_KEY_EXCL   1 => serialize non-modifier presses             [0]
      VICE_CTL_TRACE      1 => one line per applied edge on stderr        [0]
*/

#include "vice.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include "kbd.h"
#include "keyboard.h"
#include "keymap.h"
#include "lib.h"
#include "archdep.h"
#include "log.h"
#include "machine.h"
#include "machine-video.h"
#include "maincpu.h"
#include "screenshot.h"
#include "vicectl.h"

#define VICECTL_PROTO       "vicectl/1"
#define VICECTL_MAX_CONN    4
#define VICECTL_MAX_LINE    8192
#define VICECTL_MAX_OUT     (1 << 20)

/* X11 keysyms of the host modifiers we must track, so that a keymap entry
   flagged MAP_MOD_SHIFT / MAP_MOD_CTRL / MAP_MOD_RIGHT_ALT resolves the way it
   does under a real X keyboard. Numeric, not #included: this file must build
   with no X11 headers present. */
#define XKS_SHIFT_L     0xffe1
#define XKS_SHIFT_R     0xffe2
#define XKS_CONTROL_L   0xffe3
#define XKS_CONTROL_R   0xffe4
#define XKS_ALT_L       0xffe9
#define XKS_ALT_R       0xffea
#define XKS_ISO_LEVEL3  0xfe03

/* ------------------------------------------------------------------ state */

typedef struct ctl_cmd_s {
    struct ctl_cmd_s *next;
    int conn;               /* connection generation-checked index */
    unsigned int gen;
    char *seq;
    char *line;             /* verb + args */
} ctl_cmd_t;

typedef struct ctl_reply_s {
    struct ctl_reply_s *next;
    int conn;
    unsigned int gen;
    char *text;
} ctl_reply_t;

typedef enum { KEY_KIND_SYM = 0, KEY_KIND_MATRIX } key_kind_t;

typedef struct key_ent_s {
    struct key_ent_s *next;
    key_kind_t kind;
    long sym;               /* KEY_KIND_SYM */
    int row, col;           /* KEY_KIND_MATRIX */
    int val;
    int id;                 /* pacing identity */
    int conn;
    unsigned int gen;
    char *seq;
} key_ent_t;

typedef struct {
    int id;
    int in_use;
    int want;               /* last queued value */
    long down_frame;
    long up_frame;
    int down;               /* last applied value */
} key_state_t;

#define KEY_STATE_MAX 512

typedef struct {
    int fd;
    unsigned int gen;
    char in[VICECTL_MAX_LINE];
    size_t in_len;
    char *out;
    size_t out_len, out_cap;
} ctl_conn_t;

static int ctl_checked = 0;
static int ctl_on = 0;
static const char *ctl_path = NULL;

static int ctl_listen_fd = -1;
static pthread_t ctl_thread;
static int ctl_thread_running = 0;
static volatile int ctl_stop = 0;

static pthread_mutex_t ctl_lock = PTHREAD_MUTEX_INITIALIZER;
static ctl_cmd_t *ctl_cmd_head = NULL, *ctl_cmd_tail = NULL;
static ctl_reply_t *ctl_reply_head = NULL, *ctl_reply_tail = NULL;

static ctl_conn_t ctl_conns[VICECTL_MAX_CONN];

/* emulation-thread only below this line */
static long ctl_frame = 0;
static key_ent_t *ctl_key_head = NULL, *ctl_key_tail = NULL;
static key_state_t ctl_key_state[KEY_STATE_MAX];
static int ctl_mod = 0;                 /* KBD_MOD_* accumulated host modifiers */
static int ctl_excl_down = 0;           /* non-modifier keys currently down */
static long ctl_excl_up_frame = -100000;
static long ctl_hold_frames = 0, ctl_gap_frames = 0;
static int ctl_excl = 0;
static int ctl_trace = 0;
static unsigned long ctl_c_key = 0, ctl_c_coalesced = 0, ctl_c_err = 0;

static log_t ctl_log = LOG_DEFAULT;

/* --------------------------------------------------------------- helpers */

static long env_long(const char *name, long dflt)
{
    const char *v = getenv(name);
    char *end = NULL;
    long r;

    if (v == NULL || *v == '\0') {
        return dflt;
    }
    r = strtol(v, &end, 10);
    if (end == NULL || *end != '\0') {
        return dflt;
    }
    return r;
}

int vicectl_enabled(void)
{
    if (!ctl_checked) {
        const char *p = getenv("VICE_CTL_SOCK");
        ctl_checked = 1;
        if (p != NULL && *p != '\0') {
            ctl_path = p;
            ctl_on = 1;
        }
    }
    return ctl_on;
}

/* ------------------------------------------------------- reply plumbing */

static void reply_push(int conn, unsigned int gen, const char *text)
{
    ctl_reply_t *r = lib_malloc(sizeof(ctl_reply_t));

    r->next = NULL;
    r->conn = conn;
    r->gen = gen;
    r->text = lib_strdup(text);

    pthread_mutex_lock(&ctl_lock);
    if (ctl_reply_tail != NULL) {
        ctl_reply_tail->next = r;
    } else {
        ctl_reply_head = r;
    }
    ctl_reply_tail = r;
    pthread_mutex_unlock(&ctl_lock);
}

static void reply_ok(int conn, unsigned int gen, const char *seq, const char *detail)
{
    char buf[512];

    /* Bound both fields explicitly. gcc cannot see that a caller's `detail`
       is short, so an unbounded %s here is a -Wformat-truncation warning on
       every build; the kernel-hive builder asserts a clean build, and a
       warning nobody can fix is a warning everybody learns to ignore. The
       clamps are far above any reply this module actually sends (the long
       payloads go through reply_data(), not here). */
    snprintf(buf, sizeof(buf), "%.64s OK%s%.400s\n", seq,
             (detail != NULL && *detail != '\0') ? " " : "",
             (detail != NULL) ? detail : "");
    reply_push(conn, gen, buf);
}

static void reply_err(int conn, unsigned int gen, const char *seq,
                      const char *code, const char *text)
{
    char buf[512];

    ctl_c_err++;
    snprintf(buf, sizeof(buf), "%s ERR %s %s\n", seq, code, text != NULL ? text : "");
    reply_push(conn, gen, buf);
}

static void reply_data(int conn, unsigned int gen, const char *seq, const char *text)
{
    char buf[1024];

    snprintf(buf, sizeof(buf), "%s DATA %s\n", seq, text);
    reply_push(conn, gen, buf);
}

/* --------------------------------------------------------- pacing state */

static key_state_t *key_state(int id)
{
    int i, free_slot = -1;

    for (i = 0; i < KEY_STATE_MAX; i++) {
        if (ctl_key_state[i].in_use && ctl_key_state[i].id == id) {
            return &ctl_key_state[i];
        }
        if (!ctl_key_state[i].in_use && free_slot < 0) {
            free_slot = i;
        }
    }
    if (free_slot < 0) {
        return NULL;
    }
    memset(&ctl_key_state[free_slot], 0, sizeof(key_state_t));
    ctl_key_state[free_slot].in_use = 1;
    ctl_key_state[free_slot].id = id;
    ctl_key_state[free_slot].down_frame = -100000;
    ctl_key_state[free_slot].up_frame = -100000;
    return &ctl_key_state[free_slot];
}

/* Resolve a keysym to its matrix position through the machine's OWN keymap.
   Returns 1 and fills row/col when the keymap names it. Deliberately not
   keyboard_find_mapped_row_col(): that helper indexes keyconvmap[-1] when the
   symbol is unmapped. */
static int sym_to_rowcol(long sym, int *row, int *col)
{
    int i;

    if (keyconvmap == NULL) {
        return 0;
    }
    for (i = 0; i < keyconvmap_num_keys; i++) {
        if (keyconvmap[i].sym == sym) {
            *row = keyconvmap[i].row;
            *col = keyconvmap[i].column;
            return 1;
        }
    }
    return 0;
}

static int ent_is_modifier(const key_ent_t *e)
{
    int row, col;

    if (e->kind == KEY_KIND_SYM) {
        switch (e->sym) {
            case XKS_SHIFT_L: case XKS_SHIFT_R:
            case XKS_CONTROL_L: case XKS_CONTROL_R:
            case XKS_ALT_L: case XKS_ALT_R: case XKS_ISO_LEVEL3:
                return 1;
            default:
                break;
        }
        if (sym_to_rowcol(e->sym, &row, &col)) {
            return key_is_modifier(row, col);
        }
        return 0;
    }
    return key_is_modifier(e->row, e->col);
}

static void track_host_mod(long sym, int pressed)
{
    int bit = 0;

    switch (sym) {
        case XKS_SHIFT_L:   bit = KBD_MOD_LSHIFT; break;
        case XKS_SHIFT_R:   bit = KBD_MOD_RSHIFT; break;
        case XKS_CONTROL_L: bit = KBD_MOD_LCTRL; break;
        case XKS_CONTROL_R: bit = KBD_MOD_RCTRL; break;
        case XKS_ALT_L:     bit = KBD_MOD_LALT; break;
        case XKS_ALT_R:
        case XKS_ISO_LEVEL3: bit = KBD_MOD_RALT; break;
        default: return;
    }
    if (pressed) {
        ctl_mod |= bit;
    } else {
        ctl_mod &= ~bit;
    }
}

/* ------------------------------------------------------------ key queue */

static void key_enqueue(key_ent_t *e)
{
    key_state_t *st = key_state(e->id);

    if (st != NULL && st->want == e->val) {
        ctl_c_coalesced++;
        reply_ok(e->conn, e->gen, e->seq, "coalesced");
        lib_free(e->seq);
        lib_free(e);
        return;
    }
    if (st != NULL) {
        st->want = e->val;
    }
    e->next = NULL;
    if (ctl_key_tail != NULL) {
        ctl_key_tail->next = e;
    } else {
        ctl_key_head = e;
    }
    ctl_key_tail = e;
}

static void key_apply(key_ent_t *e)
{
    if (e->kind == KEY_KIND_SYM) {
        track_host_mod(e->sym, e->val);
        if (e->val) {
            keyboard_key_pressed(e->sym, ctl_mod);
        } else {
            keyboard_key_released(e->sym, ctl_mod);
        }
    } else {
        keyboard_set_keyarr_any(e->row, e->col, e->val);
    }
    if (ctl_trace) {
        fprintf(stderr, "VICECTL frame=%ld apply %s %ld/%d,%d val=%d mod=0x%x\n",
                ctl_frame, e->kind == KEY_KIND_SYM ? "KEY" : "MKEY",
                e->sym, e->row, e->col, e->val, (unsigned)ctl_mod);
    }
}

static void key_drain(void)
{
    key_ent_t *e, *prev = NULL, *next;
    int blocked[KEY_STATE_MAX];
    int n_blocked = 0, i, is_blocked;

    for (e = ctl_key_head; e != NULL; e = next) {
        key_state_t *st;
        long gate;
        int mod_key;

        next = e->next;

        is_blocked = 0;
        for (i = 0; i < n_blocked; i++) {
            if (blocked[i] == e->id) {
                is_blocked = 1;
                break;
            }
        }
        if (is_blocked) {
            if (ctl_excl) {
                /* strict order: a shift release must never overtake a
                   deferred press */
                break;
            }
            prev = e;
            continue;
        }

        st = key_state(e->id);
        if (st == NULL) {
            reply_err(e->conn, e->gen, e->seq, "nostate", "pacing table full");
            goto drop;
        }

        gate = (e->val == 1) ? (st->up_frame + ctl_gap_frames)
                             : (st->down_frame + ctl_hold_frames);
        if (ctl_frame < gate) {
            if (ctl_excl) {
                break;
            }
            if (n_blocked < KEY_STATE_MAX) {
                blocked[n_blocked++] = e->id;
            }
            prev = e;
            continue;
        }

        mod_key = ent_is_modifier(e);
        if (ctl_excl && e->val == 1 && !mod_key
            && (ctl_excl_down > 0 || ctl_frame < ctl_excl_up_frame + ctl_gap_frames)) {
            break;
        }

        key_apply(e);
        ctl_c_key++;
        if (e->val == 1) {
            st->down_frame = ctl_frame;
            st->down = 1;
            if (!mod_key) {
                ctl_excl_down++;
            }
        } else {
            st->up_frame = ctl_frame;
            if (st->down && !mod_key) {
                ctl_excl_down--;
                ctl_excl_up_frame = ctl_frame;
            }
            st->down = 0;
        }
        reply_ok(e->conn, e->gen, e->seq, NULL);

drop:
        if (prev != NULL) {
            prev->next = next;
        } else {
            ctl_key_head = next;
        }
        if (ctl_key_tail == e) {
            ctl_key_tail = prev;
        }
        lib_free(e->seq);
        lib_free(e);
    }
}

/* ------------------------------------------------------------- KEYDUMP */

/* The KEYDUMP analogue of the MAME module: the map is read OUT of the running
   machine (its parsed .vkm, the anchors it resolved, its matrix geometry), so
   the host-side keymap is GENERATED, never authored. */
static void do_keydump(int conn, unsigned int gen, const char *seq)
{
    char buf[512];
    int i, n = 0;

    snprintf(buf, sizeof(buf), "MACHINE %s rows=%d cols=%d keys=%d",
             machine_get_name(), KBD_ROWS, KBD_COLS,
             keyconvmap != NULL ? keyconvmap_num_keys : 0);
    reply_data(conn, gen, seq, buf);

    snprintf(buf, sizeof(buf),
             "ANCHOR lshift=%d,%d rshift=%d,%d lcbm=%d,%d lctrl=%d,%d "
             "vshift=%d vcbm=%d vctrl=%d shiftl=%d",
             kbd_lshiftrow, kbd_lshiftcol, kbd_rshiftrow, kbd_rshiftcol,
             kbd_lcbmrow, kbd_lcbmcol, kbd_lctrlrow, kbd_lctrlcol,
             key_ctrl_vshift, key_ctrl_vcbm, key_ctrl_vctrl, key_ctrl_shiftl);
    reply_data(conn, gen, seq, buf);

    snprintf(buf, sizeof(buf), "CUSTOM restore1=%d restore2=%d column4080=%d caps=%d",
             key_ctrl_restore1, key_ctrl_restore2,
             key_ctrl_column4080, key_ctrl_caps);
    reply_data(conn, gen, seq, buf);

    if (keyconvmap == NULL) {
        reply_err(conn, gen, seq, "nokeymap",
                  "no keymap loaded (headless without VICE_CTL_SOCK at init?)");
        return;
    }
    for (i = 0; i < keyconvmap_num_keys; i++) {
        snprintf(buf, sizeof(buf), "KEY %ld %s %d %d 0x%04x",
                 keyconvmap[i].sym,
                 kbd_arch_keynum_to_keyname(keyconvmap[i].sym),
                 keyconvmap[i].row, keyconvmap[i].column,
                 (unsigned)keyconvmap[i].shift);
        reply_data(conn, gen, seq, buf);
        n++;
    }
    snprintf(buf, sizeof(buf), "%d", n);
    reply_ok(conn, gen, seq, buf);
}

/* ------------------------------------------------------------- dispatch */

static void dispatch(ctl_cmd_t *c)
{
    char *line = c->line;
    char *sp = strchr(line, ' ');
    const char *rest = "";
    char verb[32];
    size_t vlen;

    vlen = (sp != NULL) ? (size_t)(sp - line) : strlen(line);
    if (vlen >= sizeof(verb)) {
        vlen = sizeof(verb) - 1;
    }
    memcpy(verb, line, vlen);
    verb[vlen] = '\0';
    if (sp != NULL) {
        rest = sp + 1;
    }

    if (strcmp(verb, "PING") == 0) {
        reply_ok(c->conn, c->gen, c->seq, NULL);
    } else if (strcmp(verb, "KEY") == 0) {
        int val;
        long sym;
        char *end = NULL;
        key_ent_t *e;

        if (rest[0] != '0' && rest[0] != '1') {
            reply_err(c->conn, c->gen, c->seq, "badarg", "KEY <0|1> <keysym> [mod]");
            return;
        }
        val = rest[0] - '0';
        sym = strtol(rest + 1, &end, 0);
        if (end == rest + 1) {
            reply_err(c->conn, c->gen, c->seq, "badarg", "KEY <0|1> <keysym> [mod]");
            return;
        }
        e = lib_calloc(1, sizeof(key_ent_t));
        e->kind = KEY_KIND_SYM;
        e->sym = sym;
        e->row = e->col = -1;
        e->val = val;
        e->id = (int)(sym & 0x7fffff);
        e->conn = c->conn;
        e->gen = c->gen;
        e->seq = lib_strdup(c->seq);
        key_enqueue(e);
    } else if (strcmp(verb, "MKEY") == 0) {
        int val, row, col;
        key_ent_t *e;

        if (sscanf(rest, "%d %d %d", &val, &row, &col) != 3
            || (val != 0 && val != 1)) {
            reply_err(c->conn, c->gen, c->seq, "badarg", "MKEY <0|1> <row> <col>");
            return;
        }
        e = lib_calloc(1, sizeof(key_ent_t));
        e->kind = KEY_KIND_MATRIX;
        e->sym = -1;
        e->row = row;
        e->col = col;
        e->val = val;
        e->id = 0x800000 | ((row & 0xff) << 8) | (col & 0xff);
        e->conn = c->conn;
        e->gen = c->gen;
        e->seq = lib_strdup(c->seq);
        key_enqueue(e);
    } else if (strcmp(verb, "KEYCLEAR") == 0) {
        keyboard_key_clear();
        ctl_mod = 0;
        ctl_excl_down = 0;
        memset(ctl_key_state, 0, sizeof(ctl_key_state));
        reply_ok(c->conn, c->gen, c->seq, NULL);
    } else if (strcmp(verb, "KEYDUMP") == 0) {
        do_keydump(c->conn, c->gen, c->seq);
    } else if (strcmp(verb, "SHOT") == 0) {
        struct video_canvas_s *canvas = machine_video_canvas_get(0);

        if (rest[0] == '\0') {
            reply_err(c->conn, c->gen, c->seq, "badarg", "SHOT <path>");
        } else if (canvas == NULL) {
            reply_err(c->conn, c->gen, c->seq, "nocanvas", "no video canvas");
        } else if (screenshot_save("PNG", rest, canvas) < 0) {
            reply_err(c->conn, c->gen, c->seq, "shotfail", rest);
        } else {
            reply_ok(c->conn, c->gen, c->seq, rest);
        }
    } else if (strcmp(verb, "RESET") == 0) {
        machine_trigger_reset(rest[0] == '1' ? MACHINE_RESET_MODE_POWER_CYCLE
                                             : MACHINE_RESET_MODE_RESET_CPU);
        reply_ok(c->conn, c->gen, c->seq, NULL);
    } else if (strcmp(verb, "SAVEST") == 0) {
        if (machine_write_snapshot(rest, 0, 0, 0) < 0) {
            reply_err(c->conn, c->gen, c->seq, "savefail", rest);
        } else {
            reply_ok(c->conn, c->gen, c->seq, rest);
        }
    } else if (strcmp(verb, "LOADST") == 0) {
        if (machine_read_snapshot(rest, 0) < 0) {
            reply_err(c->conn, c->gen, c->seq, "loadfail", rest);
        } else {
            reply_ok(c->conn, c->gen, c->seq, rest);
        }
    } else if (strcmp(verb, "STAT") == 0) {
        char buf[256];

        snprintf(buf, sizeof(buf),
                 "frame=%ld keys=%lu coalesced=%lu err=%lu queued=%d mod=0x%x "
                 "hold=%ld gap=%ld excl=%d clk=%lu",
                 ctl_frame, ctl_c_key, ctl_c_coalesced, ctl_c_err,
                 ctl_key_head != NULL, (unsigned)ctl_mod,
                 ctl_hold_frames, ctl_gap_frames, ctl_excl,
                 (unsigned long)maincpu_clk);
        reply_ok(c->conn, c->gen, c->seq, buf);
    } else if (strcmp(verb, "EXIT") == 0) {
        reply_ok(c->conn, c->gen, c->seq, NULL);
        ctl_stop = 1;
        archdep_vice_exit(0);
    } else if (verb[0] == '\0') {
        reply_err(c->conn, c->gen, c->seq, "badline", "empty verb");
    } else {
        reply_err(c->conn, c->gen, c->seq, "badverb", verb);
    }
}

/* ------------------------------------------------------- emulation side */

void vicectl_frame(void)
{
    ctl_cmd_t *list, *c, *next;

    if (!vicectl_enabled() || !ctl_thread_running) {
        return;
    }

    ctl_frame++;

    pthread_mutex_lock(&ctl_lock);
    list = ctl_cmd_head;
    ctl_cmd_head = ctl_cmd_tail = NULL;
    pthread_mutex_unlock(&ctl_lock);

    for (c = list; c != NULL; c = next) {
        next = c->next;
        dispatch(c);
        lib_free(c->seq);
        lib_free(c->line);
        lib_free(c);
    }

    key_drain();
}

/* ---------------------------------------------------------- socket side */

static void conn_out(ctl_conn_t *cn, const char *text)
{
    size_t len = strlen(text);

    if (cn->out_len + len > VICECTL_MAX_OUT) {
        /* stalled reader: drop the connection rather than grow forever */
        close(cn->fd);
        cn->fd = -1;
        return;
    }
    if (cn->out_len + len + 1 > cn->out_cap) {
        cn->out_cap = (cn->out_len + len + 1) * 2;
        cn->out = lib_realloc(cn->out, cn->out_cap);
    }
    memcpy(cn->out + cn->out_len, text, len);
    cn->out_len += len;
}

static void cmd_push(int conn, unsigned int gen, const char *line)
{
    ctl_cmd_t *c;
    const char *sp = strchr(line, ' ');
    char seq[64];
    size_t slen;

    slen = (sp != NULL) ? (size_t)(sp - line) : strlen(line);
    if (slen >= sizeof(seq)) {
        slen = sizeof(seq) - 1;
    }
    memcpy(seq, line, slen);
    seq[slen] = '\0';

    c = lib_malloc(sizeof(ctl_cmd_t));
    c->next = NULL;
    c->conn = conn;
    c->gen = gen;
    c->seq = lib_strdup(seq);
    c->line = lib_strdup(sp != NULL ? sp + 1 : "");

    pthread_mutex_lock(&ctl_lock);
    if (ctl_cmd_tail != NULL) {
        ctl_cmd_tail->next = c;
    } else {
        ctl_cmd_head = c;
    }
    ctl_cmd_tail = c;
    pthread_mutex_unlock(&ctl_lock);
}

static void conn_feed(int idx)
{
    ctl_conn_t *cn = &ctl_conns[idx];
    char buf[4096];
    ssize_t n;
    size_t i;

    n = read(cn->fd, buf, sizeof(buf));
    if (n <= 0) {
        if (n < 0 && (errno == EAGAIN || errno == EINTR)) {
            return;
        }
        close(cn->fd);
        cn->fd = -1;
        return;
    }
    for (i = 0; i < (size_t)n; i++) {
        if (buf[i] == '\n') {
            cn->in[cn->in_len] = '\0';
            if (cn->in_len > 0) {
                cmd_push(idx, cn->gen, cn->in);
            }
            cn->in_len = 0;
        } else if (cn->in_len < VICECTL_MAX_LINE - 1) {
            cn->in[cn->in_len++] = buf[i];
        }
    }
}

static void *ctl_thread_main(void *arg)
{
    struct pollfd pfd[VICECTL_MAX_CONN + 1];
    int i;

    (void)arg;

    while (!ctl_stop) {
        int nfd = 0;
        int listen_idx;
        ctl_reply_t *rlist, *r, *rnext;

        pfd[nfd].fd = ctl_listen_fd;
        pfd[nfd].events = POLLIN;
        listen_idx = nfd;
        nfd++;

        for (i = 0; i < VICECTL_MAX_CONN; i++) {
            if (ctl_conns[i].fd >= 0) {
                pfd[nfd].fd = ctl_conns[i].fd;
                pfd[nfd].events = POLLIN | (ctl_conns[i].out_len > 0 ? POLLOUT : 0);
                pfd[nfd].revents = 0;
                nfd++;
            }
        }

        if (poll(pfd, nfd, 10) < 0 && errno != EINTR) {
            break;
        }

        if (pfd[listen_idx].revents & POLLIN) {
            int fd = accept(ctl_listen_fd, NULL, NULL);

            if (fd >= 0) {
                int slot = -1;

                for (i = 0; i < VICECTL_MAX_CONN; i++) {
                    if (ctl_conns[i].fd < 0) {
                        slot = i;
                        break;
                    }
                }
                if (slot < 0) {
                    close(fd);
                } else {
                    char banner[256];

                    fcntl(fd, F_SETFL, O_NONBLOCK);
                    ctl_conns[slot].fd = fd;
                    ctl_conns[slot].gen++;
                    ctl_conns[slot].in_len = 0;
                    ctl_conns[slot].out_len = 0;
                    snprintf(banner, sizeof(banner),
                             VICECTL_PROTO " machine=%s rows=%d cols=%d keys=%d\n",
                             machine_get_name(), KBD_ROWS, KBD_COLS,
                             keyconvmap != NULL ? keyconvmap_num_keys : 0);
                    conn_out(&ctl_conns[slot], banner);
                }
            }
        }

        /* the pollfd order matches the live-connection order built above */
        {
            int p = 1;

            for (i = 0; i < VICECTL_MAX_CONN; i++) {
                if (ctl_conns[i].fd < 0) {
                    continue;
                }
                if (p < nfd) {
                    if (pfd[p].revents & (POLLIN | POLLHUP | POLLERR)) {
                        conn_feed(i);
                    }
                    p++;
                }
            }
        }

        pthread_mutex_lock(&ctl_lock);
        rlist = ctl_reply_head;
        ctl_reply_head = ctl_reply_tail = NULL;
        pthread_mutex_unlock(&ctl_lock);

        for (r = rlist; r != NULL; r = rnext) {
            rnext = r->next;
            if (r->conn >= 0 && r->conn < VICECTL_MAX_CONN
                && ctl_conns[r->conn].fd >= 0
                && ctl_conns[r->conn].gen == r->gen) {
                conn_out(&ctl_conns[r->conn], r->text);
            }
            lib_free(r->text);
            lib_free(r);
        }

        for (i = 0; i < VICECTL_MAX_CONN; i++) {
            ctl_conn_t *cn = &ctl_conns[i];
            ssize_t w;

            if (cn->fd < 0 || cn->out_len == 0) {
                continue;
            }
            w = write(cn->fd, cn->out, cn->out_len);
            if (w > 0) {
                memmove(cn->out, cn->out + w, cn->out_len - w);
                cn->out_len -= w;
            } else if (w < 0 && errno != EAGAIN && errno != EINTR) {
                close(cn->fd);
                cn->fd = -1;
            }
        }
    }
    return NULL;
}

/* ------------------------------------------------------------- lifecycle */

void vicectl_init(void)
{
    struct sockaddr_un sa;
    long fps;
    long hold_ms, gap_ms;
    int i;

    if (!vicectl_enabled()) {
        return;
    }

    ctl_log = log_open("VICEctl");

    for (i = 0; i < VICECTL_MAX_CONN; i++) {
        ctl_conns[i].fd = -1;
    }

    /* pacing is expressed in ms but enforced in FRAMES, because the guest
       scans its matrix once per frame and a finer gate would be a lie */
    hold_ms = env_long("VICE_CTL_KEY_HOLD", 60);
    gap_ms = env_long("VICE_CTL_KEY_GAP", 60);
    fps = machine_get_cycles_per_frame() > 0
        ? (machine_get_cycles_per_second() / machine_get_cycles_per_frame()) : 50;
    if (fps <= 0) {
        fps = 50;
    }
    ctl_hold_frames = (hold_ms * fps + 999) / 1000;
    ctl_gap_frames = (gap_ms * fps + 999) / 1000;
    if (ctl_hold_frames < 1) {
        ctl_hold_frames = 1;
    }
    if (ctl_gap_frames < 1) {
        ctl_gap_frames = 1;
    }
    ctl_excl = env_long("VICE_CTL_KEY_EXCL", 0) != 0;
    ctl_trace = env_long("VICE_CTL_TRACE", 0) != 0;

    if (strlen(ctl_path) >= sizeof(sa.sun_path)) {
        log_error(ctl_log, "VICE_CTL_SOCK path too long: %s", ctl_path);
        return;
    }

    unlink(ctl_path);
    ctl_listen_fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (ctl_listen_fd < 0) {
        log_error(ctl_log, "socket(): %s", strerror(errno));
        return;
    }
    memset(&sa, 0, sizeof(sa));
    sa.sun_family = AF_UNIX;
    strncpy(sa.sun_path, ctl_path, sizeof(sa.sun_path) - 1);
    if (bind(ctl_listen_fd, (struct sockaddr *)&sa, sizeof(sa)) < 0
        || listen(ctl_listen_fd, 4) < 0) {
        log_error(ctl_log, "bind/listen %s: %s", ctl_path, strerror(errno));
        close(ctl_listen_fd);
        ctl_listen_fd = -1;
        return;
    }
    fcntl(ctl_listen_fd, F_SETFL, O_NONBLOCK);

    if (pthread_create(&ctl_thread, NULL, ctl_thread_main, NULL) != 0) {
        log_error(ctl_log, "cannot start control thread");
        close(ctl_listen_fd);
        ctl_listen_fd = -1;
        return;
    }
    ctl_thread_running = 1;
    log_message(ctl_log,
                VICECTL_PROTO " listening on %s (hold=%ld gap=%ld frames, excl=%d, keys=%d)",
                ctl_path, ctl_hold_frames, ctl_gap_frames, ctl_excl,
                keyconvmap != NULL ? keyconvmap_num_keys : 0);
}

void vicectl_shutdown(void)
{
    int i;

    if (!ctl_thread_running) {
        return;
    }
    ctl_stop = 1;
    pthread_join(ctl_thread, NULL);
    ctl_thread_running = 0;
    for (i = 0; i < VICECTL_MAX_CONN; i++) {
        if (ctl_conns[i].fd >= 0) {
            close(ctl_conns[i].fd);
            ctl_conns[i].fd = -1;
        }
        lib_free(ctl_conns[i].out);
        ctl_conns[i].out = NULL;
    }
    if (ctl_listen_fd >= 0) {
        close(ctl_listen_fd);
        ctl_listen_fd = -1;
    }
    if (ctl_path != NULL) {
        unlink(ctl_path);
    }
}
