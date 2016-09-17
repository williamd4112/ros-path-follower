#include <iostream>

#include "motion.h"

//static cv::Rect ROI(MOTION_DETECT_ROI_PADDING, MOTION_DETECT_ROI_PADDING, VIDEO_FRONT_WIDTH-MOTION_DETECT_ROI_PADDING, VIDEO_FRONT_HEIGHT-MOTION_DETECT_ROI_PADDING);

/**
  * GPU helper functions
  */
#ifdef GPU_MOTION
static void download(const cv::cuda::GpuMat & d_mat, std::vector<cv::Point2f> & vec)
{
    vec.resize(d_mat.cols);
    cv::Mat mat(1, d_mat.cols, CV_32FC2, (void*)&vec[0]);
    d_mat.download(mat);
}

static void download(const cv::cuda::GpuMat & d_mat, std::vector<uchar> & vec)
{
    vec.resize(d_mat.cols);
    cv::Mat mat(1, d_mat.cols, CV_8UC1, (void*)&vec[0]);
    d_mat.download(mat);
}
#endif

#ifdef DEBUG_MOTION_DETECT
static void drawArrows(cv::Mat & frame, const std::vector<cv::Point2f> & prevPts, const std::vector<cv::Point2f> & nextPts, const std::vector<uchar> & status, cv::Scalar line_color = cv::Scalar(0, 0, 255))
{
    for (size_t i = 0; i < prevPts.size(); ++i)
    {
        if (status[i])
        {
            int line_thickness = 1;

            cv::Point p = prevPts[i];
            cv::Point q = nextPts[i];

            double angle = atan2((double) p.y - q.y, (double) p.x - q.x);

            double hypotenuse = sqrt( (double)(p.y - q.y)*(p.y - q.y) + (double)(p.x - q.x)*(p.x - q.x) );

            if (hypotenuse < 1.0)
                continue;

            // Here we lengthen the arrow by a factor of three.
            q.x = (int) (p.x - 3 * hypotenuse * cos(angle));
            q.y = (int) (p.y - 3 * hypotenuse * sin(angle));

            // Now we draw the main line of the arrow.
            cv::line(frame, p, q, line_color, line_thickness);

            // Now draw the tips of the arrow. I do some scaling so that the
            // tips look proportional to the main line of the arrow.

            p.x = (int) (q.x + 9 * cos(angle + CV_PI / 4));
            p.y = (int) (q.y + 9 * sin(angle + CV_PI / 4));
            cv::line(frame, p, q, line_color, line_thickness);

            p.x = (int) (q.x + 9 * cos(angle - CV_PI / 4));
            p.y = (int) (q.y + 9 * sin(angle - CV_PI / 4));
            cv::line(frame, p, q, line_color, line_thickness);
        }
    }
}
#endif

motion_detector::motion_detector(int32_t diff_ts, int32_t area_ts, int maxCorners, double qualityLevel, double minDistance, int blockSize, bool useHarrisDetector, double harrisK, cv::Size winSize, int maxLevel, int iters, bool useInitialFlow):
	m_diff_ts(diff_ts), m_area_ts(area_ts), m_maxCorners(maxCorners), 
	m_qualityLevel(qualityLevel), m_minDistance(minDistance), 
	m_blockSize(blockSize), m_useHarrisDetector(useHarrisDetector), 
	m_harrisK(harrisK),  
	m_winSize(winSize), m_maxLevel(maxLevel), m_iters(iters), 
	m_useInitialFlow(useInitialFlow),
	m_last_homography_mat(cv::Mat::ones(3, 3, CV_64FC1))
{
#ifdef GPU_MOTION
	m_feature_detector = 
		cv::cuda::createGoodFeaturesToTrackDetector(
			CV_8UC1, 
			maxCorners, 
			qualityLevel, 
			minDistance,
			blockSize,
			useHarrisDetector,
			harrisK);
 	m_pyrlk = cv::cuda::SparsePyrLKOpticalFlow::create(
                winSize, maxLevel, iters);
	m_blurFilter = cv::cuda::createGaussianFilter(CV_8U, CV_8U, cv::Size(31, 31), 0, 0, cv::BORDER_DEFAULT, -1);
#endif

}

motion_detector::~motion_detector()
{

}

int32_t motion_detector::detect(const videoframe_t & frame, const cv::Rect & roi)
{
	int32_t ret = 0;
	if (m_last_frame.empty()) {
		port_loadMatFromVideo(frame, m_last_frame);	
		port_cvtColor(m_last_frame, m_last_frame, CV_BGR2GRAY);
		port_equalizeHist(m_last_frame, m_last_frame);
	}
	else {	
		port_Mat cur_frame;
#ifdef DEBUG_MOTION_DETECT_BOUNDING_BOX
		cv::Mat _cur_frame;
#endif
		port_Mat diff;
		std::vector<cv::Rect> rects;	
		port_loadMatFromVideo(frame, cur_frame);
#ifdef DEBUG_MOTION_DETECT_BOUNDING_BOX
#ifdef GPU_MOTION
		cur_frame.download(_cur_frame);
#else
		cur_frame.copyTo(_cur_frame);
#endif
#endif
		port_cvtColor(cur_frame, cur_frame, CV_BGR2GRAY);
		port_equalizeHist(cur_frame, cur_frame);

		ego_motion_compansate(m_last_frame, cur_frame, diff, m_diff_ts, roi); 
		find_motion_area(diff, rects, m_area_ts);

#ifdef DEBUG_MOTION_DETECT_BOUNDING_BOX
		_cur_frame = _cur_frame(roi);
		for (auto rect : rects) {
			cv::rectangle(_cur_frame, rect.tl(), rect.br(), cv::Scalar(0, 255, 0), 2);	
		}
		cv::imshow("motion_detector-motion_area", _cur_frame);
#endif
		if (m_last_frame.empty()) {
			cur_frame.copyTo(m_last_frame);
		}
		
#ifdef GPU
		cv::cuda::addWeighted(m_last_frame, 0.2, cur_frame, 0.8, 0, m_last_frame);
#else
		cv::addWeighted(m_last_frame, 0.2, cur_frame, 0.8, 0, m_last_frame);	
#endif
		if (rects.size() > 0) {
			ret = 1;
		}
	}
	return ret;
}

inline void motion_detector::find_motion_area(const port_Mat & diff, std::vector<cv::Rect> & rects, int area_ts)
{
	std::vector<std::vector<cv::Point> > contours;
#ifdef GPU_MOTION_BLOB
#else	

#ifdef GPU_MOTION
	cv::Mat diff_host;
	diff.download(diff_host);
	cv::dilate(diff_host, diff_host, cv::Mat(), cv::Point(-1, -1), 2);
#else
	cv::dilate(diff, diff, cv::Mat());
#endif
	
#ifdef GPU_MOTION
	cv::findContours(diff_host, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
#else
	cv::findContours(diff, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
#endif	
	for(auto contour : contours) {
		if (cv::contourArea(contour) > area_ts) {
			rects.push_back(cv::boundingRect(contour));
		}
	}		
#endif		
}

inline void motion_detector::ego_motion_compansate(const port_Mat & src, const port_Mat & dst, port_Mat & diff, int diff_ts, const cv::Rect & roi)
{
	std::vector<cv::Point2f> points[2];
	std::vector<uchar> status;
	port_Mat mask;
	port_Mat src_warp;
	port_Mat dst_masked;

	calculate_points_lk(src, dst, points[0], points[1], status);
	cv::Mat new_homography_mat = cv::findHomography(points[0], points[1], CV_RANSAC, 1);	
	
	port_warpPerspective(src, src_warp, new_homography_mat, src.size());
	port_threshold(src_warp, mask, 0, 255, CV_THRESH_BINARY);	
	
	dst.copyTo(dst_masked, mask);
#ifdef GPU_MOTION
	m_blurFilter->apply(src_warp, src_warp);
	m_blurFilter->apply(dst_masked, dst_masked);
#else
	cv::GaussianBlur(src_warp, src_warp, cv::Size(31, 31), 0);
	cv::GaussianBlur(dst_masked, dst_masked, cv::Size(31, 31), 0);	
#endif

	port_absdiff(src_warp, dst_masked, diff);
#ifdef GPU_MOTION
	//m_blurFilter->apply(diff, diff);
#else
	//cv::GaussianBlur(diff, diff, cv::Size(31, 31), 0);
#endif
	diff = diff(roi);

#ifdef DEBUG_MOTION_DETECT
	cv::Mat _diff_no_thresh;
#ifdef GPU_MOTION
	diff.download(_diff_no_thresh);
#else
	diff.copyTo(_diff_no_thresh);
#endif
#endif
	port_threshold(diff, diff, diff_ts, 255, CV_THRESH_BINARY);

#ifdef DEBUG_MOTION_DETECT
	cv::Mat _src_warp;
	cv::Mat _dst_masked;
	cv::Mat _mask;
	cv::Mat _diff;
#ifdef GPU_MOTION
	src_warp.download(_src_warp);
	dst_masked.download(_dst_masked);
	mask.download(_mask);
	diff.download(_diff);
#else
	src_warp.copyTo(_src_warp);
	mask.copyTo(_mask);
	dst_masked.copyTo(_dst_masked);
	diff.copyTo(_diff);
#endif
	//drawArrows(visual, points[0], points[1], status);
	cv::imshow("ego-motion-src_warp", _src_warp);
	cv::imshow("ego-motion-mask", _mask);
	cv::imshow("ego-motion-dst_masked", _dst_masked);
	cv::imshow("ego-motion-diff-no-ts", _diff_no_thresh);
	cv::imshow("ego-motion-diff", _diff);
#endif
}

inline void motion_detector::calculate_points_lk(const port_Mat & src, const port_Mat & dst, std::vector<cv::Point2f> & points0, std::vector<cv::Point2f> & points1, std::vector<uchar> & status)
{
	static cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);
	static cv::Size winSize(21, 21);
	static cv::Size subPixWinSize(10, 10);
#ifdef GPU_MOTION
	port_Mat d_points0;
	port_Mat d_points1;
	port_Mat d_status;
	
	m_feature_detector->detect(dst, d_points0);
	m_pyrlk->calc(src, dst, d_points0, d_points1, d_status);

	download(d_points0, points0);
	download(d_points1, points1);
	download(d_status, status);
#else
	std::vector<float> err;
	cv::goodFeaturesToTrack(dst, 
			points0,
			m_maxCorners,
			m_qualityLevel, 
			m_minDistance, 
			cv::Mat(), 
			m_blockSize, 
			m_useHarrisDetector,
			m_harrisK);
	cv::cornerSubPix(dst, points0, subPixWinSize, cv::Size(-1,-1), termcrit);
	cv::calcOpticalFlowPyrLK(src, dst, points0, points1, status, err, winSize,
                                 3, termcrit, 0, 0.001);
#endif		
}

