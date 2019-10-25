/*
 * Copyright (C) 2018-2019 Gautier Hattenberger <gautier.hattenberger@enac.fr>
 *                         Titouan Verdu <titouan.verdu@enac.fr>
 *
 * This file is part of paparazzi.
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

/**
 * @file modules/nav/nav_lace.c
 * @author VERDU Titouan
 *
 * Adaptive border pattern for cloud exploration
 */

#include "modules/nav/nav_lace.h"

#include "firmwares/fixedwing/nav.h"
#include "state.h"
#include "autopilot.h"
#include "generated/flight_plan.h"
#include "subsystems/abi.h"

enum LaceStatus {
  LACE_ENTER,
  LACE_INSIDE,
  LACE_OUTSIDE
};

enum RotationDir {
  LACE_LEFT,
  LACE_RIGHT
};

struct NavLace {
  enum LaceStatus status;
  enum RotationDir rotation;
  bool inside_cloud;
  struct EnuCoor_f actual;
  struct EnuCoor_f target;
  struct EnuCoor_f circle;
  struct FloatVect3 pos_incr;
  float direction;
  float radius;
  float radius_sign;
};

static struct NavLace nav_lace;

static const float nav_dt = 1.f / NAVIGATION_FREQUENCY;

static float change_rep(float dir)
{
  return M_PI_2 - dir;
}

static struct EnuCoor_f process_new_point_lace(struct EnuCoor_f *position, float uav_direction)
{
  struct EnuCoor_f new_point;
  float rot_angle;

  if (nav_lace.rotation == LACE_RIGHT) {
    rot_angle = -M_PI_2;
    nav_lace.rotation = LACE_LEFT;
  } else{
    rot_angle = M_PI_2;
    nav_lace.rotation = LACE_RIGHT;
  }

  new_point.x = position->x + (cos(rot_angle + uav_direction) * nav_lace.radius);
  new_point.y = position->y + (sin(rot_angle + uav_direction) * nav_lace.radius);
  new_point.z = position->z + ground_alt;

  return new_point;
}

#if USE_MISSION
#include "modules/mission/mission_common.h"

static bool nav_lace_mission(uint8_t nb, float *params, enum MissionRunFlag flag)
{
  if (nb != 8) {
    return false; // wrong number of parameters
  }
  if (flag == MissionInit) {
    float start_x = params[0];
    float start_y = params[1];
    float start_z = params[2];
    int first_turn = params[3];
    float circle_radius = params[4];
    float vx = params[5];
    float vy = params[6];
    float vz = params[7];
    nav_lace_setup(start_x, start_y, start_z, first_turn, circle_radius, vx, vy, vz);
  }
  return nav_lace_run();
}
#endif

// ABI message

#ifndef NAV_LACE_LWC_ID
#define NAV_LACE_LWC_ID ABI_BROADCAST
#endif

static abi_event lwc_ev;

static void lwc_cb(uint8_t sender_id UNUSED, uint32_t stamp UNUSED, int32_t data_type, uint32_t size, uint8_t * data) {
  if (data_type == 1 && size == 1) {
    nav_lace.inside_cloud = (bool) data[0];
  }
}

void nav_lace_init(void)
{
  nav_lace.status = LACE_ENTER;
  nav_lace.radius = DEFAULT_CIRCLE_RADIUS;
  nav_lace.inside_cloud = false;

  AbiBindMsgPAYLOAD_DATA(NAV_LACE_LWC_ID, &lwc_ev, lwc_cb);

#if USE_MISSION
  mission_register(nav_lace_mission, "LACE");
#endif
}

void nav_lace_setup(float init_x, float init_y, 
                    float init_z, int turn, 
                    float desired_radius, float vx,
                    float vy, float vz)
{
  struct EnuCoor_f start = {init_x, init_y, init_z};
  // increment based on speed
  VECT3_ASSIGN(nav_lace.pos_incr, vx*nav_dt, vy*nav_dt, vz*nav_dt);

  nav_lace.target = start;
  nav_lace.status = LACE_ENTER;
  nav_lace.inside_cloud = false;
  nav_lace.radius = desired_radius;
  

  if (turn == 1) {
    nav_lace.rotation = LACE_RIGHT;
    nav_lace.radius_sign = 1.0f;
  } else {
    nav_lace.rotation = LACE_LEFT;
    nav_lace.radius_sign = -1.0f;
  }

  nav_lace.actual = *stateGetPositionEnu_f();
}

bool nav_lace_run(void)
{
  float pre_climb = 0.f;

  NavVerticalAutoThrottleMode(0.f); /* No pitch */

  switch (nav_lace.status) {
    case LACE_ENTER:
      nav_route_xy(nav_lace.actual.x, nav_lace.actual.y, nav_lace.target.x, nav_lace.target.y);
      NavVerticalAltitudeMode(nav_lace.target.z, pre_climb);

      if (nav_lace.inside_cloud) {
        nav_lace.status = LACE_INSIDE;
        nav_lace.actual = *stateGetPositionEnu_f();
        nav_lace.direction = change_rep(stateGetHorizontalSpeedDir_f());
        nav_lace.circle = process_new_point_lace(&nav_lace.actual, nav_lace.direction);
      }
      break;
    case LACE_INSIDE:
      // increment center position
      VECT3_ADD(nav_lace.circle, nav_lace.pos_incr);
      nav_circle_XY(nav_lace.circle.x, nav_lace.circle.y , nav_lace.radius_sign * nav_lace.radius);
      NavVerticalAltitudeMode(nav_lace.circle.z, pre_climb);

      if (!nav_lace.inside_cloud) {
        nav_lace.status = LACE_OUTSIDE;
        nav_lace.actual = *stateGetPositionEnu_f();
        nav_lace.direction = change_rep(stateGetHorizontalSpeedDir_f());
        nav_lace.circle = process_new_point_lace(&nav_lace.actual, nav_lace.direction);
        nav_lace.radius_sign = -1.0 * nav_lace.radius_sign;
      }
      pre_climb = nav_lace.pos_incr.z / nav_dt;
      break;
    case LACE_OUTSIDE:
      VECT3_ADD(nav_lace.circle, nav_lace.pos_incr);
      nav_circle_XY(nav_lace.circle.x, nav_lace.circle.y , nav_lace.radius_sign * nav_lace.radius);
      NavVerticalAltitudeMode(nav_lace.circle.z, pre_climb);
      
      if(nav_lace.inside_cloud){
        nav_lace.status = LACE_INSIDE;
        nav_lace.actual = *stateGetPositionEnu_f();
        nav_lace.direction = change_rep(stateGetHorizontalSpeedDir_f());
        nav_lace.circle = process_new_point_lace(&nav_lace.actual, nav_lace.direction);
        nav_lace.radius_sign = -1.0 * nav_lace.radius_sign;
      }
      pre_climb = nav_lace.pos_incr.z / nav_dt;
      break;
    default:
      // error, leaving
      return false;
  }

  return true;
}
