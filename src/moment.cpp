#include "config.h"
#include "moment.hpp"

moment_detector::moment_detector(cv::Scalar lower, cv::Scalar upper): 
	m_lower(lower), m_upper(upper)
{
}

moment_detector::~moment_detector()
{
}	

cv::Point2f moment_detector::detect(const videoframe_t & frame, const videoframe_t & frame_hsv, 
	const cv::Rect & roi)
{
	cv::Moments m;
	cv::Point2f center(0, 0);

#ifdef GPU
	assert(false);
#else
	port_Mat mask;
	cv::inRange(frame_hsv, m_lower, m_upper, mask);
	mask = mask(roi);
	m = cv::moments(mask);
#endif
	if (m.m00 > 0) {
		center = cv::Point2f(m.m10/m.m00, m.m01/m.m00);	
	}	
	
#ifdef DEBUG_MOMENT_DETECT
	cv::Mat _mask;
	cv::Mat _frame_result;
#ifdef GPU
	assert(false);
#else
	mask.copyTo(_mask);
	frame(roi).copyTo(_frame_result);
#endif
	cv::circle(_frame_result, cv::Point2f(frame.cols / 2, roi.height / 2), 3, cv::Scalar(255, 0, 0, 3));
	if (m.m00 > 0) {
		cv::circle(_frame_result, center, 3, cv::Scalar(0, 0, 255), 3);
	}
	cv::imshow("moment-mask", _mask);
	cv::imshow("moment-frame_result", _frame_result);
#endif
	return center;
}
