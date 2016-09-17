#ifndef _MOMENT_HPP_
#define _MOMENT_HPP_

#include "config.h"

class moment_detector
{
public:
	moment_detector(cv::Scalar lower=cv::Scalar(0, 0, 255-127), cv::Scalar upper=cv::Scalar(255, 255, 255), int32_t area_ts=2e6, std::string debug_tag="moment");
	~moment_detector();
	
	std::pair<int32_t, bool> detect(const videoframe_t & frame, const videoframe_t & frame_hsv, const cv::Rect & roi);
private:
	cv::Scalar m_lower;
	cv::Scalar m_upper;
	int32_t m_area_ts;
#ifdef DEBUG_MOMENT_DETECT
	std::string m_debug_tag_mask;
	std::string m_debug_tag_result;
#endif
};

#endif
