#include "config.h"
#include "keyboard.h"

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
#else
#define USE_OBJECT_DETECT "Object Detect : Off"
#endif


static float g_linear_init = 0.1f;

#ifdef MOTION_DETECT
static motion_detector motion(50, 50, 1000);
#endif

#ifdef LANE_DETECT
static lane_detector lane;
#endif

#ifdef MOMENT_DETECT
static moment_detector moment_white;
#endif

#ifdef OBJECT_DETECT
static object_haar_detector object(OBJECT_DETECT_TRAFFIC_LIGHT_CASCADE_FILE_PATH);

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
	std::cout << USE_GPU << std::endl 
			<< USE_ROS_ADAPTER << std::endl
			<< USE_VIDEO_PIPELINE << std::endl
			<< USE_MOTION_DETECT << std::endl
			<< USE_LANE_DETECT << std::endl
			<< USE_MOMENT_DETECT << std::endl
			<< USE_OBJECT_DETECT << std::endl;
}

static void process(videoframe_t & frame_front, videoframe_t & frame_ground)
{
	/* Frame pre-processing */
	videoframe_t frame_front_hsv;
	videoframe_t frame_ground_hsv;
	cv::cvtColor(frame_front, frame_front_hsv, CV_BGR2HSV);
	cv::cvtColor(frame_ground, frame_ground_hsv, CV_BGR2HSV);

	videoframe_t frame_front_gray;
	videoframe_t frame_ground_gray;
	cv::cvtColor(frame_front, frame_front_gray, CV_BGR2GRAY);
	cv::cvtColor(frame_ground, frame_ground_gray, CV_BGR2GRAY);

	/* FSM Variable */
	bool isTrafficLight = false;
	bool isMotion = false;
	// bool isApproaching = false;
	// bool isCliff = false;
	// bool isSlope = false;

	float linear = 0;
	float angular = 0;	

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
	std::vector<cv::Rect> targets;
	object.detect(frame_front, frame_front_gray, targets);
#ifndef DEBUG_OBJECT_DETECT_REDCIRCLE
	if (targets.size() > 0) {
#endif
		int32_t traffic_light_count  = redcircle_find(frame_front, frame_front_hsv, targets);
		if (traffic_light_count > 0) {
			isTrafficLight = true;
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
	
	if (argc >= 4) {
		sscanf(argv[3], "%f", &g_linear_init);
		std::cout << "Init linear : " << g_linear_init << std::endl;
	}
	
	std::cout << "Path-Follower running on CPU " << sched_getcpu() << std::endl;
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
