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
 * @file modules/nav/nav_rosette.c
 *
 * Adaptive flower pattern for cloud exploration
 */

#include "modules/nav/nav_rosette.h"

#include "firmwares/fixedwing/nav.h"
#include "state.h"
#include "autopilot.h"
#include "generated/flight_plan.h"
#include <stdio.h>

enum RosetteStatus {
  RSTT_ENTER,
  RSTT_CROSSING,
  RSTT_TURNING
};

enum RotationDir {
  RSTT_LEFT,
  RSTT_RIGHT
};

struct NavRosette {
  enum RosetteStatus status;
  enum RotationDir rotation;
  bool inside_cloud;
  struct EnuCoor_f actual;
  struct EnuCoor_f target;
  struct EnuCoor_f circle;
  struct EnuCoor_f barycenter;
  struct FloatVect3 pos_incr;
  float direction;
  float radius;
  float radius_sign;
  int nb_border_point;
};

static struct NavRosette nav_rosette;

static const float nav_dt = 1.f / NAVIGATION_FREQUENCY;

static float change_rep(float dir)
{
  return M_PI_2 - dir;
}

static float calc_dist(struct EnuCoor_f *pos_actual, struct EnuCoor_f *pos_wanted){
  float res = sqrtf(powf((pos_wanted->x - pos_actual->x),2) + powf((pos_wanted->y - pos_actual->y),2));

  return res;
}

static void update_barycenter(struct EnuCoor_f *new_coord, struct EnuCoor_f *bary){
  struct EnuCoor_f new_point;
    nav_rosette.nb_border_point += 1;

  if(nav_rosette.nb_border_point == 1){
    bary->x = new_coord->x;
    bary->y = new_coord->y;
    bary->z = new_coord->z + ground_alt;

    printf("[1]CordActual X = %f ; Y = %f ; Z = %f \n", new_coord->x, new_coord->y, new_coord->z + ground_alt);
    printf("[1]New bary X = %f ; Y = %f ; Z = %f \n", bary->x, bary->y, bary->z);

  } else if(nav_rosette.nb_border_point >= 2){
    //new_point.x = ((nav_rosette.barycenter.x * (nav_rosette.nb_border_point - 1)) + new_coord->x ) / nav_rosette.nb_border_point;
    //new_point.y = ((nav_rosette.barycenter.y * (nav_rosette.nb_border_point - 1)) + new_coord->y ) / nav_rosette.nb_border_point;
    bary->x = ((bary->x * 0.5) + (new_coord->x * 0.5));
    bary->y = ((bary->y * 0.5) + (new_coord->y * 0.5));
    bary->z = new_coord->z + ground_alt;

    printf("[2+]CordActual X = %f ; Y = %f ; Z = %f \n", new_coord->x, new_coord->y, new_coord->z + ground_alt);
    printf("[2+]New bary X = %f ; Y = %f ; Z = %f \n", bary->x, bary->y, bary->z);

  }
}

//static struct EnuCoor_f update_point(struct EnuCoor_f *pos_actual){
//  struct EnuCoor_f new_point;
//  float distance;
//
//  new_point.x = truncf(nav_rosette.target.x + (2 * (nav_rosette.target.x - pos_actual->x)));
//  new_point.y = truncf(nav_rosette.target.y + (2 * (nav_rosette.target.y - pos_actual->y)));
//  distance = calc_dist(pos_actual, &new_point);
//
//  while(distance < nav_rosette.dis_min){
//    new_point.x = truncf(new_point.x + ((1/2) * (new_point.x - pos_actual->x)));
//    new_point.y = truncf(new_point.y + ((1/2) * (new_point.y - pos_actual->y)));
//    distance = calc_dist(pos_actual, &new_point);
//  }
//
//  new_point.z = nav_rosette.target.z + ground_alt;
//
//  return new_point;
//}

static struct EnuCoor_f process_new_point_rosette(struct EnuCoor_f *position, float uav_direction)
{
  struct EnuCoor_f new_point;
  float rot_angle;

  if (nav_rosette.rotation == RSTT_RIGHT)
  {
    rot_angle = -M_PI_2;
  } else{
    rot_angle = M_PI_2;
  }

  if(nav_rosette.inside_cloud == true && nav_rosette.nb_border_point > 2){
    new_point.x = nav_rosette.barycenter.x;
    new_point.y = nav_rosette.barycenter.y;
    new_point.z = nav_rosette.barycenter.z;
  }
  else if(nav_rosette.inside_cloud == false){
    new_point.x = position->x + (cos(rot_angle + uav_direction) * nav_rosette.radius);
    new_point.y = position->y + (sin(rot_angle + uav_direction) * nav_rosette.radius);
    new_point.z = position->z + ground_alt;
  }

  return new_point;
}

#if USE_MISSION
#include "modules/mission/mission_common.h"

static bool nav_rosette_mission(uint8_t nb, float *params, enum MissionRunFlag flag)
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
    nav_rosette_setup(start_x, start_y, start_z, first_turn, circle_radius, vx, vy, vz);
  }
  return nav_rosette_run();
}
#endif

// ABI message

#ifndef NAV_ROSETTE_LWC_ID
#define NAV_ROSETTE_LWC_ID ABI_BROADCAST
#endif

static abi_event lwc_ev;

static void lwc_cb(uint8_t sender_id UNUSED, uint32_t stamp UNUSED, int32_t data_type, uint32_t size, uint8_t * data)
{
  if (data_type == 1 && size == 1) {
    nav_rosette.inside_cloud = (bool) data[0];
  }
}

void nav_rosette_init(void)
{
  nav_rosette.status = RSTT_ENTER;
  nav_rosette.radius = DEFAULT_CIRCLE_RADIUS;
  nav_rosette.inside_cloud = false;

  AbiBindMsgPAYLOAD_DATA(NAV_ROSETTE_LWC_ID, &lwc_ev, lwc_cb);

#if USE_MISSION
  mission_register(nav_rosette_mission, "RSTT");
#endif
}

void nav_rosette_setup(float init_x, float init_y, float init_z,
                       int turn, float desired_radius,
                       float vx, float vy, float vz)
{
  struct EnuCoor_f start = {init_x, init_y, init_z};
  // increment based on speed
  VECT3_ASSIGN(nav_rosette.pos_incr, vx*nav_dt, vy*nav_dt, vz*nav_dt);

  nav_rosette.target = start;
  nav_rosette.status = RSTT_ENTER;
  nav_rosette.inside_cloud = false;
  nav_rosette.nb_border_point = 0;
  nav_rosette.radius = desired_radius;

  if (turn == 1)
  {
    nav_rosette.radius_sign = 1.0f;
    nav_rosette.rotation = RSTT_RIGHT;
  } else {
    nav_rosette.radius_sign = -1.0f;
    nav_rosette.rotation = RSTT_LEFT;
  }

  nav_rosette.actual = *stateGetPositionEnu_f();
}

bool nav_rosette_run(void)
{
  float pre_climb = 0.f;

  NavVerticalAutoThrottleMode(0.f); /* No Pitch */

  switch (nav_rosette.status) {
    case RSTT_ENTER:
      nav_route_xy(nav_rosette.actual.x, nav_rosette.actual.y, nav_rosette.target.x, nav_rosette.target.y);
      NavVerticalAltitudeMode(nav_rosette.target.z, pre_climb);

      if (nav_rosette.inside_cloud)
      {
        nav_rosette.status = RSTT_CROSSING;
        nav_rosette.actual = *stateGetPositionEnu_f();
        update_barycenter(&nav_rosette.actual, &nav_rosette.barycenter);
      }
      break;
    case RSTT_CROSSING:
      nav_route_xy(nav_rosette.actual.x, nav_rosette.actual.y, nav_rosette.target.x, nav_rosette.target.y);
      NavVerticalAltitudeMode(nav_rosette.target.z, pre_climb);

      if (!nav_rosette.inside_cloud)
      {
        nav_rosette.status = RSTT_TURNING;
        nav_rosette.actual = *stateGetPositionEnu_f();
        update_barycenter(&nav_rosette.actual, &nav_rosette.barycenter);
        nav_rosette.direction = change_rep(stateGetHorizontalSpeedDir_f());
        nav_rosette.circle = process_new_point_rosette(&nav_rosette.actual, nav_rosette.direction);
      }
      pre_climb = nav_rosette.pos_incr.z / nav_dt;
      break;
    case RSTT_TURNING:
      VECT3_ADD(nav_rosette.circle, nav_rosette.pos_incr);
      nav_circle_XY(nav_rosette.circle.x, nav_rosette.circle.y , nav_rosette.radius_sign * nav_rosette.radius);
      NavVerticalAltitudeMode(nav_rosette.circle.z, pre_climb);

      if (nav_rosette.inside_cloud)
      {
        nav_rosette.status = RSTT_CROSSING;
        nav_rosette.actual = *stateGetPositionEnu_f();
        update_barycenter(&nav_rosette.actual, &nav_rosette.barycenter);
        nav_rosette.direction = change_rep(stateGetHorizontalSpeedDir_f());
        nav_rosette.target = nav_rosette.barycenter;
      }
      pre_climb = nav_rosette.pos_incr.z / nav_dt;
      break;
    default:
      // error, leaving
      return false;
  }

  return true;
}
