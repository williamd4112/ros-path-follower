#include "bounce.hpp"

#include <cassert>
#include <cstdint>

#include <iostream>
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
#include <opencv2/objdetect/objdetect.hpp>


Bounce::Bounce(cv::Scalar lower, cv::Scalar upper, cv::Rect roi):
	m_lower(lower),
	m_upper(upper),
	m_roi(roi)
{
}

Bounce::~Bounce()
{
}

int32_t Bounce::detect(const cv::Mat & frame, const cv::Mat & frame_hsv, const cv::Point2i center, const int32_t area_ts)
{
	int32_t offset = 0;

	/* Color detection */
	cv::Mat mask;
	cv::inRange(frame_hsv, m_lower, m_upper, mask);	
	
	/* Find contour */
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Rect> rects;
	cv::findContours(mask, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	std::vector<cv::Moments> mu;
	std::vector<cv::Point2f> mc;
	for(auto contour : contours) {
		if (cv::contourArea(contour) > area_ts) {
			mu.push_back(cv::moments(contour, false));
		}
	}

	for( int i = 0; i < mu.size(); i++ ) { 
		cv::Point2f p(mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00);
		mc.push_back(p);

		int32_t diff = center.x - p.x;
		offset += diff; 
	}

#ifdef DEBUG_BOUNCE
	cv::Mat _frame;
	_frame = frame(m_roi);
	for (auto p : mc) {
		cv::circle(_frame, p, 3, cv::Scalar(0, 0, 255), 4);
	}
	cv::imshow("bounce", _frame);
#endif
	return offset;
}
