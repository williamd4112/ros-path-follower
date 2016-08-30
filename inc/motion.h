#ifndef _MOTION_H_
#define _MOTION_H_

#include <stdint.h>

#include <vector>

#include <opencv2/core/core.hpp>

#include "config.h"

class motion_detector
{
public:
	motion_detector(int32_t diff_ts=50, int32_t area_ts=50);
	~motion_detector();

	int32_t detect(const videoframe_t & frame);
private:
	int32_t m_diff_ts;
	int32_t m_area_ts;
	cv::Mat m_homography_mat;
	videoframe_t m_last_frame;	

	void ego_motion_compensate(videoframe_t & src, videoframe_t & dst);	
	void calculate_points_lk(cv::Mat src, cv::Mat dst, std::vector<cv::Point> & src_points, std::vector<cv::Point> & dst_points);
	void frame_diff(videoframe_t & frame1, videoframe_t & frame2);
}

#endif
