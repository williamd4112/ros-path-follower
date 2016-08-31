#include <iostream>

#include "motion.h"

/**
  * GPU helper functions
  */
#ifdef GPU
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

motion_detector::motion_detector(int32_t diff_ts, int32_t area_ts, int maxCorners, double qualityLevel, double minDistance, int blockSize, bool useHarrisDetector, double harrisK, cv::Size winSize, int maxLevel, int iters, bool useInitialFlow):
	m_diff_ts(diff_ts), m_area_ts(area_ts), m_maxCorners(maxCorners), 
	m_qualityLevel(qualityLevel), m_minDistance(minDistance), 
	m_blockSize(blockSize), m_useHarrisDetector(useHarrisDetector), 
	m_harrisK(harrisK),  
	m_winSize(winSize), m_maxLevel(maxLevel), m_iters(iters), 
	m_useInitialFlow(useInitialFlow),
	m_last_homography_mat(cv::Mat::ones(3, 3, CV_64FC1))
{
#ifdef GPU
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
#endif

}

motion_detector::~motion_detector()
{

}

int32_t motion_detector::detect(const videoframe_t & frame)
{
	if (m_last_frame.empty()) {
		port_loadMatFromVideo(frame, m_last_frame);	
		port_cvtColor(m_last_frame, m_last_frame, CV_BGR2GRAY);
		port_equalizeHist(m_last_frame, m_last_frame);
	}
	else {	
		port_Mat dst;
		port_loadMatFromVideo(frame, dst);
		port_cvtColor(dst, dst, CV_BGR2GRAY);
		port_equalizeHist(dst, dst);

		ego_motion_compansate(m_last_frame, dst);
		dst.copyTo(m_last_frame);
	}
	return 0;
}

inline void motion_detector::ego_motion_compansate(port_Mat & src, port_Mat & dst)
{
	std::vector<cv::Point2f> points[2];
	std::vector<uchar> status;

	calculate_points_lk(src, dst, points[0], points[1], status);
	cv::Mat new_homography_mat = cv::findHomography(points[0], points[1], CV_RANSAC, 1);	
	
	cv::cuda::warpPerspective(src, src, new_homography_mat, src.size());
	
	m_last_homography_mat = new_homography_mat * m_last_homography_mat;

#ifdef DEBUG_MOTION_DETECTOR
	cv::Mat visual;
#ifdef GPU
	src.download(visual);
#else
	src.copyTo(visual);
#endif
	drawArrows(visual, points[0], points[1], status);
	if (!visual.empty()) {
		cv::imshow("ego-motion-compansate", visual);
		cv::waitKey(1);
	}
#endif
}

inline void motion_detector::calculate_points_lk(port_Mat & src, port_Mat & dst, std::vector<cv::Point2f> & points0, std::vector<cv::Point2f> & points1, std::vector<uchar> & status)
{
	static cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS,20, 0.03);
	static cv::Size winSize(21, 21);
	static cv::Size subPixWinSize(10, 10);
#ifdef GPU
	port_Mat d_points0;
	port_Mat d_points1;
	port_Mat d_status;
	
	m_feature_detector->detect(src, d_points0);
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
	//cv::cornerSubPix(dst, points1, subPixWinSize, cv::Size(-1,-1), termcrit);
	cv::calcOpticalFlowPyrLK(src, dst, points0, points1, status, err, winSize,
                                 3, termcrit, 0, 0.001);
#endif		
}

