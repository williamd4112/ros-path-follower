#ifndef _OBJECT_HPP_
#define _OBJECT_HPP_

#include "config.h"

class object_haar_detector
{
public:
	object_haar_detector(std::string classfier_filepath);
	~object_haar_detector();

	int32_t detect(
		const videoframe_t & frame, 
		const videoframe_t & frame_gray, 
		double scaleFactor=1.1, 
		int minNeighbors=3,
		int flags=0, 
		cv::Size minSize=cv::Size(30, 30),
		cv::Size maxSize=cv::Size());
private:
	cv::CascadeClassifier m_cascade;
	
};

#endif
