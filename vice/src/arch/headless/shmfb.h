/** \file   shmfb.h
 * \brief   Shared-memory framebuffer publisher for the headless UI
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

#ifndef VICE_HEADLESS_SHMFB_H
#define VICE_HEADLESS_SHMFB_H

struct video_canvas_s;
struct palette_s;

/** \brief  Pick up VICE_SHM_PATH; call once from video_init(). */
void shmfb_init(void);

/** \brief  Non-zero when VICE_SHM_PATH was set, i.e. publishing is on. */
int shmfb_enabled(void);

/** \brief  Program the canvas' render tables for host-endian XRGB8888. */
void shmfb_set_palette(struct video_canvas_s *canvas);

/** \brief  Render one refreshed rectangle into the mapping. */
void shmfb_refresh(struct video_canvas_s *canvas,
                   unsigned int xs, unsigned int ys,
                   unsigned int xi, unsigned int yi,
                   unsigned int w, unsigned int h);

/** \brief  Unmap; call from video_shutdown(). */
void shmfb_shutdown(void);

#endif
