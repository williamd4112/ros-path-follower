#include "config.h"

#include "lane.hpp"

#ifdef DEBUG_LANE_DETECT
static void drawline(cv::Mat & cdst, float rho, float theta, cv::Scalar color=cv::Scalar(0,0,255))
{
	cv::Point pt1, pt2;
    double a = cos(theta), b = sin(theta);
    double x0 = a*rho, y0 = b*rho;
    pt1.x = cvRound(x0 + 1000*(-b));
    pt1.y = cvRound(y0 + 1000*(a));
    pt2.x = cvRound(x0 - 1000*(-b));
    pt2.y = cvRound(y0 - 1000*(a));
    cv::line( cdst, pt1, pt2, color, 1, CV_AA);
}

static void drawlines(cv::Mat & cdst, std::vector<cv::Vec2f> & lines, cv::Scalar color=cv::Scalar(0,0,255))
{
	for(cv::Vec2f line : lines) {
		drawline(cdst, line[0], line[1], color);
	}
}
#endif

#ifdef LANE_DETECT_QUANTNIZE
static bool sort_line(cv::Vec2f a, cv::Vec2f b)
{
	return a[1] < b[1];
}

static void quantnize(std::vector<cv::Vec2f> & lines, std::vector<cv::Vec2f> & quant_lines)
{
	static float quant_size = 0.001f;

	if (lines.size() < 2) {
		return;
	}
	std::sort(lines.begin(), lines.end(), sort_line);	
	
	float theta_last = 0.0f;
	float rho_acc = 0.0f;
	float theta_acc = 0.0f;
	int theta_cnt = 0;
	for(cv::Vec2f line : lines) {
		if (std::abs(theta_last - line[1]) > quant_size) {
			quant_lines.push_back(cv::Vec2f(rho_acc / (float) theta_cnt, theta_acc / (float) theta_cnt));
			rho_acc = theta_acc = 0.0f;
			theta_cnt = 0;
		}
		else {
			rho_acc += line[0];
			theta_acc += line[1];
			theta_cnt++;
		}
		theta_last = line[1];
	}	

	if (theta_cnt != 0) {
		quant_lines.push_back(cv::Vec2f(rho_acc / (float) theta_cnt, theta_acc / (float) theta_cnt));
	}

#ifdef DEBUG_LANE_DETECT
	printf("\tLane Before : %-4u, After : %-4u", lines.size(), quant_lines.size());
#endif
	
}
#endif

lane_detector::lane_detector(cv::Scalar lane_color_lower, cv::Scalar lane_color_upper, uint32_t vote_balance, uint32_t vote_init, uint32_t vote_step, uint32_t vote_baseline)
	: m_vote(vote_init),
	m_vote_step(vote_step),
	m_vote_balance(vote_balance),
	m_vote_baseline(vote_baseline),
	m_lane_color_lower(lane_color_lower), 
	m_lane_color_upper(lane_color_upper)
{

}

lane_detector::~lane_detector()
{
}

int32_t lane_detector::detect(const videoframe_t & frame, const videoframe_t & hsv)
{
	std::vector<cv::Vec2f> lines;
#ifdef GPU
	assert(false);
#else
	port_Mat edges;
	port_Mat mask;
	port_Mat frame_masked;

	cv::inRange(hsv, m_lane_color_lower, m_lane_color_upper, mask);
	frame.copyTo(frame_masked, mask);
	cv::Canny(frame_masked, edges, 50, 150);	
	HoughLines(edges, lines, 1, CV_PI/180.0, m_vote, 0, 0);
#ifdef LANE_DETECT_QUANTNIZE
	std::vector<cv::Vec2f> quant_lines;
	quantnize(lines, quant_lines);
#endif
#endif

#ifdef DEBUG_LANE_DETECT
	cv::Mat _frame_masked;
	cv::Mat _frame_lane;
#ifdef GPU
	assert(false);
#else
	frame_masked.copyTo(_frame_masked);
	frame.copyTo(_frame_lane);
#endif
#ifdef LANE_DETECT_QUANTNIZE
	drawlines(_frame_lane, lines, cv::Scalar(255, 0, 0));
	drawlines(_frame_lane, quant_lines);
#else
	drawlines(_frame_lane, lines);
#endif
	cv::imshow("lane-frame_masked", _frame_masked);
	cv::imshow("lane-frame_lane", _frame_lane);
#endif

#ifdef LANE_DETECT_QUANTNIZE
	uint32_t line_cnt = quant_lines.size();
#else
	uint32_t line_cnt = lines.size();
#endif
	if (line_cnt < m_vote_balance) {
		m_vote -= m_vote_step;
		if (m_vote < m_vote_baseline) {
			m_vote = m_vote_baseline;
		} 
	}
	else if(line_cnt > m_vote_balance) {
		m_vote += m_vote_step;
	}

	return 0;	
}

