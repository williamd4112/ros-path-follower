#include <iostream>
#include <vector>
#include <deque>

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <unistd.h>
#include <sys/select.h>
#include <termios.h>
#include <pthread.h>

#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio.hpp>

#include "config.h"
#include "keyboard.h"

#ifdef ROS_ADAPTER
#include "ros_adapter.h"
#endif

#ifdef VIDEO_PIPELINE
#include "video_stream.h"
#endif

#ifdef MOTION_DETECT
#include "motion.h"
#endif

#ifdef LANE_DETECT
#include "lane.hpp"
#endif

#ifdef MOMENT_DETECT
#include "moment.hpp"
#endif

#ifdef OBJECT_DETECT
#include "object.hpp"
#endif

#ifdef MOTION_DETECT
static motion_detector motion(70, 25, 50);
#endif

#ifdef LANE_DETECT
static lane_detector lane;
#endif

#ifdef MOMENT_DETECT
static moment_detector moment_white;
#endif

#ifdef OBJECT_DETECT
static object_haar_detector object("data/traffic_light.xml");

static int32_t redcircle_find(const videoframe_t & frame, const videoframe_t & frame_hsv, std::vector<cv::Rect> & targets)
{
	int32_t ret = 0;

	cv::Mat mask_low;
	cv::inRange(frame_hsv, 
		cv::Scalar(0, 100, 100),
		cv::Scalar(10, 255, 255),
		mask_low);	
	cv::Mat mask_upper;
	cv::inRange(frame_hsv, 
		cv::Scalar(160, 100, 100),
		cv::Scalar(179, 255, 255),
		mask_upper);
	cv::Mat mask;
	cv::add(mask_low, mask_upper, mask);
	cv::GaussianBlur(mask, mask, 
		cv::Size(OBJECT_DETECT_REDCIRCLE_BLUR_SIZE, OBJECT_DETECT_REDCIRCLE_BLUR_SIZE), 0);

#ifdef DEBUG_OBJECT_DETECT_REDCIRCLE
	cv::Mat _frame;
	cv::Mat _mask;
	frame.copyTo(_frame);
	mask.copyTo(_mask);
#endif
	for(cv::Rect & target : targets) {
		std::vector<cv::Vec3f> circles;
		cv::HoughCircles(mask(target), circles, CV_HOUGH_GRADIENT, 1, 
			mask.rows / 8,
			5, 20, 0, 1000);
#ifdef DEBUG_OBJECT_DETECT_REDCIRCLE
		for (cv::Vec3f circle : circles) {
			cv::Point center(circle[0], circle[1]);
			int r = circle[2];
			cv::circle(_frame(target), center, r, cv::Scalar(0, 255, 0), 2);		
		}
#endif
		if (circles.size() > 0) {
			ret++;
		}
	}

#ifdef DEBUG_OBJECT_DETECT_REDCIRCLE
	cv::imshow("redcircle_find-mask", _mask);
	cv::imshow("redcircle_find-frame", _frame);
#endif

	return ret;
}

#endif

static void self_check()
{
	std::cout << "Path-Follower running on CPU " << sched_getcpu() << std::endl;
#ifdef GPU
	std::cout << "Use GPU : On" << std::endl;
#else
	std::cout << "Use GPU : Off" << std::endl;
#endif
}

static void process(videoframe_t & frame_front, videoframe_t & frame_ground)
{
	videoframe_t frame_front_hsv;
	videoframe_t frame_ground_hsv;
	cv::cvtColor(frame_front, frame_front_hsv, CV_BGR2HSV);
	cv::cvtColor(frame_ground, frame_ground_hsv, CV_BGR2HSV);

	videoframe_t frame_front_gray;
	videoframe_t frame_ground_gray;
	cv::cvtColor(frame_front, frame_front_gray, CV_BGR2GRAY);
	cv::cvtColor(frame_ground, frame_ground_gray, CV_BGR2GRAY);

#ifdef MOTION_DETECT
	motion.detect(frame_front);	
#endif

#ifdef LANE_DETECT
	lane.detect(frame_ground, frame_ground_hsv);
#endif

#ifdef MOMENT_DETECT
	moment_white.detect(frame_ground, frame_ground_hsv, cv::Rect(0, VIDEO_GROUND_HEIGHT - MOMENT_DETECT_Y, VIDEO_GROUND_WIDTH, MOMENT_DETECT_HEIGHT));
#endif

#ifdef OBJECT_DETECT
	int32_t traffic_light_count = 0;
	std::vector<cv::Rect> targets;
	object.detect(frame_front, frame_front_gray, targets);
#ifndef DEBUG_OBJECT_DETECT_REDCIRCLE
	if (targets.size() > 0) {
#endif
		traffic_light_count = redcircle_find(frame_front, frame_front_hsv, targets);
		if (traffic_light_count > 0) {
			printf("\t%18s.","\tRed Light Found.");
		}
		else {
			printf("\t%-20s.","");
		}
#ifndef DEBUG_OBJECT_DETECT_REDCIRCLE
	}
#endif
#endif
}

int main(int argc, char * argv[])
{
	self_check();
	
#ifdef ROS_ADAPTER
	ros_adapter::init(argc, argv);
#endif

	int ground_cam_id = atoi(argv[1]);
	int front_cam_id = atoi(argv[2]);

#ifdef VIDEO_PIPELINE	
	videostream video_front(front_cam_id, VIDEO_FRONT_WIDTH, VIDEO_FRONT_HEIGHT);
	videostream video_ground(ground_cam_id, VIDEO_GROUND_WIDTH, VIDEO_GROUND_HEIGHT);
#else
	videosource_t video_front(front_cam_id);
	video_front.set(CV_CAP_PROP_FRAME_WIDTH, VIDEO_FRONT_WIDTH);
	video_front.set(CV_CAP_PROP_FRAME_HEIGHT, VIDEO_FRONT_HEIGHT);
	
	videosource_t video_ground(ground_cam_id);
	video_ground.set(CV_CAP_PROP_FRAME_WIDTH, VIDEO_GROUND_WIDTH);
	video_ground.set(CV_CAP_PROP_FRAME_HEIGHT, VIDEO_GROUND_HEIGHT);
#endif

	videoframe_t frame_front, frame_ground;
	for(;;) {
		int key = keyboard::getKey();
		if (key == KEYCODE_ESC) {
			break;
		}
	
#ifdef DEBUG_FPS			
		double t = (double)cv::getTickCount();  
#endif

#ifdef VIDEO_PIPELINE
		video_front.read(frame_front);
		video_ground.read(frame_ground);
#else
		fetch_frame(video_front, frame_front);
		fetch_frame(video_ground, frame_ground);
#endif
		if (!frame_front.empty() && !frame_ground.empty()) {
			process(frame_front, frame_ground);
#ifdef DEBUG_FPS
			t = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
			std::cout << "\rFPS : " << (1.0 / t);
#endif
#ifdef DEBUG
#ifdef DEBUG_MAIN
			cv::imshow("frame-front", frame_front);
			cv::imshow("frame-ground", frame_ground):
#endif
			cv::waitKey(1);
#endif
		}
	}
	cv::destroyAllWindows();
#ifdef ROS_ADAPTER
	ros_adapter::shutdown();
#endif

	return 0;
}
