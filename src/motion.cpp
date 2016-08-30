#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/feature2d/feature2d.hpp>

#include "motion.h"

motion_detector::motion_detector(int32_t diff_ts, int32_t area_ts):
	m_diff_ts(diff_ts), m_area_ts(area_ts), m_homography_mat(cv::Mat::eye(3, 3, CV_32F));
{

}

motion_detector::~motion_detector()
{

}

int32_t detect(const videoframe_t & frame)
{
	if (m_last_frame.empty()) {
		frame.copyTo(m_last_frame);
		cv::cvtColor(m_last_frame, m_last_frame, cv::CV_BGR2GRAY);
		cv::equalizeHist(m_last_frame, m_last_frame);
	}
	else {
		cv::cvtColor(m_last_frame, m_last_frame, cv::CV_BGR2GRAY);
		cv::equalizeHist(m_last_frame, m_last_frame);	
		ego_motion_compensate(m_last_frame, frame);
		frame.copyTo(frame);
	}
}
