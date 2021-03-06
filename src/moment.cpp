#include "config.h"
#include "moment.hpp"

#include <sstream>

moment_detector::moment_detector(cv::Scalar lower, cv::Scalar upper, int32_t area_ts, std::string debug_tag): 
	m_lower(lower), m_upper(upper), m_area_ts(area_ts)
{
#ifdef DEBUG_MOMENT_DETECT
	std::stringstream ss;
	ss << debug_tag << "-mask";
	m_debug_tag_mask = ss.str();

	ss.clear();
	ss << debug_tag << "-result";
	m_debug_tag_result = ss.str();
#endif
}

moment_detector::~moment_detector()
{
}	

std::pair<int32_t, bool> moment_detector::detect(const videoframe_t & frame, const videoframe_t & frame_hsv, 
	const cv::Rect & roi)
{
	bool isFound = false;

	cv::Moments m;
	cv::Point2f center(frame.cols / 2, frame.rows / 2);

#ifdef GPU_MOMENT
	assert(false);
#else
	cv::Mat mask;
	cv::inRange(frame_hsv, m_lower, m_upper, mask);
	mask = mask(roi);
	m = cv::moments(mask);
#endif
	if (m.m00 > m_area_ts) {
		center = cv::Point2f(m.m10/m.m00, m.m01/m.m00);	
		isFound = true;
	}	
	std::cout << "Area : " << m.m00 << "( " << m_area_ts << " )" << " = " << isFound << std::endl; 
#ifdef DEBUG_MOMENT_DETECT 
	cv::Mat _mask;
	cv::Mat _frame_result;
#ifdef GPU_MOMENT
	assert(false);
#else
	mask.copyTo(_mask);
	frame(roi).copyTo(_frame_result);
#endif
	cv::circle(_frame_result, cv::Point2f(frame.cols / 2, roi.height / 2), 3, cv::Scalar(255, 0, 0, 3));
	if (m.m00 > m_area_ts) {
		cv::circle(_frame_result, center, 3, cv::Scalar(0, 0, 255), 3);
	}
	cv::imshow(m_debug_tag_mask, _mask);
	cv::imshow(m_debug_tag_result, _frame_result);
#endif
	
	return std::pair<int32_t, bool>(center.x - frame.cols / 2, isFound);
}
