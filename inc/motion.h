#ifndef _MOTION_H_
#define _MOTION_H_

#include <stdint.h>

#include <vector>

#include "config.h"

class motion_detector
{
public:
	motion_detector(int32_t diff_ts=50, int32_t area_ts=50, int maxCorners=1000, double qualityLevel=0.01, double minDistance=3.0, int blockSize=9, bool useHarrisDetector=false, double harrisK=0.04, cv::Size winSize=cv::Size(31, 31), int maxLevel=3, int iters=10, bool useInitialFlow=false);
	~motion_detector();

	int32_t detect(const videoframe_t & frame, const cv::Rect & roi);
private:
	int32_t m_diff_ts;
	int32_t m_area_ts;
	int m_maxCorners;
	double m_qualityLevel;
	double m_minDistance;
	int m_blockSize;
	bool m_useHarrisDetector;
	double m_harrisK; 

	cv::Size m_winSize;
	int m_maxLevel;
	int m_iters;
	bool m_useInitialFlow;
	
#ifdef GPU_MOTION
	cv::Ptr<cv::cuda::CornersDetector> m_feature_detector;
	cv::Ptr<cv::cuda::SparsePyrLKOpticalFlow> m_pyrlk;
	cv::Ptr<cv::cuda::Filter> m_blurFilter; 
#endif
	cv::Mat m_last_homography_mat;
	port_Mat m_last_frame;	

	inline void find_motion_area(const port_Mat & diff, std::vector<cv::Rect> & rects, int area_ts=100);
	inline void ego_motion_compansate(const port_Mat & src, const port_Mat & dst, port_Mat & diff, int diff_ts, const cv::Rect & roi);
	inline void calculate_points_lk(const port_Mat & src, const port_Mat & dst, std::vector<cv::Point2f> & points0, std::vector<cv::Point2f> & points1, std::vector<uchar> & status);
};

#endif
