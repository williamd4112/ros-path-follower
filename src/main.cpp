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
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio.hpp>

#include "ros_adapter.h"
#include "keyboard.h"

std::deque<cv::Mat> frame_queue;
pthread_t video_thread;

void *video_process(void * args)
{
	std::cout << "Video thread starts." << std::endl;
	cv::VideoCapture capture(0);
	capture.set(CAP_PROP_FRAME_WIDTH, 320);
	capture.set(CAP_PROP_FRAME_HEIGHT, 240);
	cv::Mat frame;
	
	for(;;) {
		capture >> frame;
	}
}

int main(int argc, char * argv[])
{
	std::cout << "Path-Follower starts." << std::endl;
	keyboard::set_conio_terminal_mode();	
	ros_adapter::init(argc, argv);
	
	int ret = pthread_create( &video_thread, NULL, video_process, NULL);	

	for(;;) {
		int key = keyboard::getKey();
		if (key == KEYCODE_ESC) {
			break;
		}

#ifdef DEBUG
		//cv::imshow("frame", frame);
		//cv::waitKey(1);
#endif
	}
	
	ros_adapter::shutdown();

	return 0;
}
