#ifndef _LANE_HPP_
#define _LANE_HPP_

#include "config.h"

class lane_detector
{
public:
	lane_detector(cv::Scalar lane_color_lower=cv::Scalar(0, 0, 0), cv::Scalar lane_color_upper=cv::Scalar(255, 255, 50), uint32_t vote_balance=8, uint32_t vote_init=100, uint32_t vote_step=5, uint32_t m_vote_baseline=10);
	~lane_detector();
	
	int32_t detect(const videoframe_t & frame, const videoframe_t & hsv);
private:
	uint32_t m_vote;
	uint32_t m_vote_step;
	uint32_t m_vote_balance;
	uint32_t m_vote_baseline;
	cv::Scalar m_lane_color_lower;
	cv::Scalar m_lane_color_upper;
};

#endif
