#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio.hpp>

#include <opencv2/cudacodec.hpp>

typedef cv::Mat videoframe_t;
typedef cv::VideoCapture videosource_t;

#define videosource_init(src, id) ((src).open((id)))

#define fetch_frame(src, dst) ((src) >> (dst)) 


#endif
