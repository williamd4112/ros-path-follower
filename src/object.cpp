#include "config.h"

#include "object.hpp"

object_haar_detector::object_haar_detector(std::string classfier_filepath)
{
	assert(m_cascade.load(classfier_filepath));
}

object_haar_detector::~object_haar_detector()
{
}

void object_haar_detector::detect(
	const videoframe_t & frame, 
	const videoframe_t & frame_gray,
	std::vector<cv::Rect> & targets,
	double scaleFactor, 
	int minNeighbors,
	int flags, 
	cv::Size minSize,
	cv::Size maxSize)
{
#ifdef GPU_OBJECT_HAAR
#else
#ifdef DEBUG_OBJECT_HAAR_DETECT
	cv::Mat _frame;
	frame.copyTo(_frame);
#endif
	std::vector<cv::Rect> objs;
	m_cascade.detectMultiScale(
		frame_gray, 
		objs, 
		scaleFactor, 
		minNeighbors, 
		flags, 
		minSize, 
		maxSize);
	targets.assign(objs.begin(), objs.end());
#ifdef DEBUG_OBJECT_HAAR_DETECT
	for(auto obj : objs) {
		cv::rectangle(_frame, obj, cv::Scalar(0, 255, 0), 2);		
	}
#endif
#ifdef DEBUG_OBJECT_HAAR_DETECT
	cv::imshow("object-haar-detector: frame", _frame);
#endif
#endif
}

