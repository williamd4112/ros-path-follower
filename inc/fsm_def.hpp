#ifndef _FSM_DEF_HPP_
#define _FSM_DEF_HPP_

#include "fsm.hpp"

typedef enum state_t
{
    STATE_NORMAL = 0,
#ifdef MOTION_DETECT
    STATE_MOTION,
#endif
#ifdef OBJECT_DETECT
    STATE_TRAFFIC_LIGHT,
#endif
#ifdef IMU
    STATE_SEESAW_UP,
    STATE_SEESAW_DOWN,
    STATE_SEESAW_OFFSET,
#endif
    STATE_ROAD_NOT_FOUND
} state_t;

/*  State */
state_id_t state_normal;
#ifdef MOTION_DETECT
state_id_t state_motion;
#endif
#ifdef OBJECT_DETECT
state_id_t state_traffic_light;
#endif
#ifdef IMU
state_id_t state_seesaw_up;
state_id_t state_seesaw_down;
state_id_t state_seesaw_offset;
#endif
state_id_t state_road_not_found;

/*  Event */
#ifdef MOTION_DETECT
event_id_t event_normal_to_motion;
event_id_t event_motion_to_normal;
#endif
#ifdef OBJECT_DETECT
event_id_t event_normal_to_traffic_light;
event_id_t event_traffic_light_to_normal;
#endif
#ifdef IMU
event_id_t event_normal_to_seesaw_up;
event_id_t event_seesaw_up_to_seesaw_down;
event_id_t event_seesaw_down_to_seesaw_offset;
#endif
event_id_t event_normal_to_road_not_found;
event_id_t event_road_not_found_to_normal;

#endif
