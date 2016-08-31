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
static motion_detector motion;
#endif

static void process(videoframe_t & frame)
{
#ifdef MOTION_DETECT
	motion.detect(frame);	
#endif
}

int main(int argc, char * argv[])
{
	std::cout << "Path-Follower running on CPU " << sched_getcpu() << std::endl;
	
	//keyboard::set_conio_terminal_mode();	
	ros_adapter::init(argc, argv);

#ifdef VIDEO_PIPELINE	
	videostream video(0);
#else
	videosource_t video(0);
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
			std::cout << "FPS : " << (1.0 / t) << std::endl;
#endif
#ifdef DEBUG
			cv::imshow("frame", frame);
			cv::waitKey(1);
#endif
		}
	}
	
	ros_adapter::shutdown();

	return 0;
}
