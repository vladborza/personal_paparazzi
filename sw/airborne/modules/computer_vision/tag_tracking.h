/*
 * Copyright (C) 2020 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *
 * This file is part of paparazzi
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

/** @file "modules/tracking/tag_tracking.h"
 * @author Gautier Hattenberger <gautier.hattenberger@enac.fr>
 * Track poistion of a tag (ArUco, QRcode, ...) detected by an onboard camera
 * The tag detection and pose computation is done outside of the module, only the estimation
 * by fusion of AHRS and visual detection with a Kalman filter is performed in this module
 */

#ifndef TAG_TRACKING_H
#define TAG_TRACKING_H

#include "std.h"

#define TAG_TRACKING_SEARCHING  0
#define TAG_TRACKING_RUNNING    1
#define TAG_TRACKING_LOST       2

struct tag_tracking_public {
  float cmd_roll;     ///< roll command [rad]
  float cmd_pitch;    ///< pitch command [rad]
  float cmd_climb;    ///< vertical speed command [m/s]
  float kp;           ///< horizontal proportional gain
  float kd;           ///< horizontal derivative gain
  float kp_climb;     ///< vertical proportional gain
  uint8_t status;     ///< tracking status flag
};

extern struct tag_tracking_public tag_tracking;

extern void tag_tracking_init(void);
extern void tag_tracking_propagate(void);
extern void tag_tracking_propagate_start(void);
extern void tag_tracking_report(void);

#endif  // TAG_TRACKING_H

