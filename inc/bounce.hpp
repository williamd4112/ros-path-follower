#ifndef _BOUNCE_HPP_
#define _BOUNCE_HPP_

#include <opencv2/core/core.hpp>

class Bounce
{
public:
	Bounce(cv::Scalar lower, cv::Scalar upper, cv::Rect roi);
	~Bounce();

	int32_t detect(const cv::Mat & frame, const cv::Mat & frame_hsv, const cv::Point2i center, const int32_t area_ts);
private:
	cv::Scalar m_lower;
	cv::Scalar m_upper;
	cv::Rect m_roi;
};

#endif
