#ifndef _MODULE_DEF_HPP
#define _MODULE_DEF_HPP

#include "config.h"

#ifdef GPU
#define USE_GPU "GPU : On"
#else
#define USE_GPU "GPU : Off"
#endif

#ifdef ROS_ADAPTER
#include "ros_adapter.h"
#define USE_ROS_ADAPTER "ROS Adapter : On"
#else
#define USE_ROS_ADAPTER "ROS Adapter : Off"
#endif

#ifdef VIDEO_PIPELINE
#include "video_stream.h"
#define USE_VIDEO_PIPELINE "Video Pipeline : On"
#else
#define USE_VIDEO_PIPELINE "Video Pipeline : Off"
#endif

#ifdef MOTION_DETECT
#include "motion.h"
#define USE_MOTION_DETECT "Motion Detect : On"
#else
#define USE_MOTION_DETECT "Motion Detect : Off"

#endif

#ifdef LANE_DETECT
#include "lane.hpp"
#define USE_LANE_DETECT "Lane Detect : On"
#else
#define USE_LANE_DETECT "Lane Detect : Off"

#endif

#ifdef MOMENT_DETECT
#include "moment.hpp"
#define USE_MOMENT_DETECT "Moment Detect : On"
#else
#define USE_MOMENT_DETECT "Moment Detect : Off"
#endif

#ifdef OBJECT_DETECT
#include "object.hpp"
#define USE_OBJECT_DETECT "Object Detect : On"
#define OBJECT_DETECT_TRAFFIC_LIGHT_CASCADE_FILE_PATH "data/trafficver_v1.xml"
#define OBJECT_DETECT_REDCIRCLE_BLUR_SIZE 3
int32_t redcircle_find(const videoframe_t & frame, const videoframe_t & frame_hsv, std::vector<cv::Rect> & targets);
#else
#define USE_OBJECT_DETECT "Object Detect : Off"
#endif

#endif
