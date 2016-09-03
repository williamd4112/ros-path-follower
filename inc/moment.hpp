#ifndef _MOMENT_HPP_
#define _MOMENT_HPP_

#include "config.h"

class moment_detector
{
public:
	moment_detector(cv::Scalar lower=cv::Scalar(0, 0, 255-127), cv::Scalar upper=cv::Scalar(180, 80, 255));
	~moment_detector();
	
	int32_t detect(const videoframe_t & frame, const videoframe_t & frame_hsv, const cv::Rect & roi);
private:
	cv::Scalar m_lower;
	cv::Scalar m_upper;
};

#endif
