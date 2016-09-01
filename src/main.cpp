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
#include "ros_adapter.h"
#include "keyboard.h"

#ifdef VIDEO_PIPELINE
#include "video_stream.h"
#endif

#ifdef MOTION_DETECT
#include "motion.h"
#endif

#ifdef LANE_DETECT
#include "lane.hpp"
#endif

#ifdef MOTION_DETECT
static motion_detector motion(70, 25, 50);
#endif

#ifdef LANE_DETECT
static lane_detector lane;
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

#ifdef MOTION_DETECT
	motion.detect(frame_front);	
#endif

#ifdef LANE_DETECT
	lane.detect(frame_ground, frame_ground_hsv);
#endif
}

int main(int argc, char * argv[])
{
	self_check();
	
	ros_adapter::init(argc, argv);

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
			cv::imshwo("frame-ground", frame_ground):
#endif
			cv::waitKey(1);
#endif
		}
	}
	cv::destroyAllWindows();
	ros_adapter::shutdown();

	return 0;
}
