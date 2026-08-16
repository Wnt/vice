/*
 * vicectl.h - vicectl: a unix-socket control channel for headless VICE.
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

#ifndef VICE_VICECTL_H
#define VICE_VICECTL_H

/* Is the control channel configured (VICE_CTL_SOCK set and non-empty)?
   Cheap, side-effect free, and safe to call before vicectl_init(): the
   keymap subsystem uses it to decide whether the headless build needs a
   host keymap at all. */
int vicectl_enabled(void);

/* Start the listener. Called once from main_program() after init_main().
   A no-op when !vicectl_enabled(). */
void vicectl_init(void);

/* Emulation-thread drain, called once per emulated frame from
   vsync_do_vsync(). A no-op when !vicectl_enabled(). */
void vicectl_frame(void);

void vicectl_shutdown(void);

#endif
