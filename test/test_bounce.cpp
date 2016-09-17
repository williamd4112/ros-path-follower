#include <cassert>
#include <cstdint>

#include <iostream>
#include <algorithm>

#include <vector>
#include <deque>

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "bounce.hpp"

/*  Ground mask (green) */
static int32_t g_green_range = 50;
static cv::Scalar lower_green(60 - g_green_range, 100, 50);
static cv::Scalar upper_green(60 + g_green_range, 255, 255);
static cv::Rect roi(0, 0, 640, 480);

static cv::Point2i center(320, 240);

static Bounce g_bounce(lower_green, upper_green, roi);

int main(int argc, char * argv[])
{
	cv::VideoCapture cap(atoi(argv[1]));
	cv::Mat frame;
	cv::Mat frame_hsv;

	for(;;) {
		cap >> frame;
		
		if (!frame.empty()) {
			cv::cvtColor(frame, frame_hsv, CV_BGR2HSV);
			int32_t offset = g_bounce.detect(frame, frame_hsv, center, 200);
			std::cout << "\r" << offset;
			std::cout << "---------------------------------------------------";
		}

		int key = cv::waitKey(1);
		if (key == 27) {
			break;
		}
	}

	return 0;
}
