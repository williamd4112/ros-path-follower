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

#ifdef MOTION_DETECT
static motion_detector motion(70, 25, 50);
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

static void process(videoframe_t & frame)
{
#ifdef MOTION_DETECT
	motion.detect(frame);	
#endif
}

int main(int argc, char * argv[])
{
	self_check();
	
	ros_adapter::init(argc, argv);

	int ground_cam_id = atoi(argv[1]);
	int front_cam_id = atoi(argv[2]);

#ifdef VIDEO_PIPELINE	
	videostream video(front_cam_id);
#else
	videosource_t video(front_cam_id);
	video.set(CV_CAP_PROP_FRAME_WIDTH, VIDEO_WIDTH);
	video.set(CV_CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT);
#endif

	videoframe_t frame;
	for(;;) {
		int key = keyboard::getKey();
		if (key == KEYCODE_ESC) {
			break;
		}
	
#ifdef DEBUG_FPS			
		double t = (double)cv::getTickCount();  
#endif

#ifdef VIDEO_PIPELINE
		video.read(frame);
#else
		fetch_frame(video, frame);
#endif
		if (!frame.empty()) {
			process(frame);
#ifdef DEBUG_FPS
			t = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
			std::cout << "\rFPS : " << (1.0 / t);
#endif
#ifdef DEBUG
#ifdef DEBUG_MAIN
			cv::imshow("frame", frame);
#endif
			cv::waitKey(1);
#endif
		}
	}
	
	ros_adapter::shutdown();

	return 0;
}
