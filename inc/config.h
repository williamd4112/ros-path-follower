#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/cudacodec.hpp>

#include "opencv2/cudaoptflow.hpp"
#include "opencv2/cudaimgproc.hpp"
#include "opencv2/cudawarping.hpp"

typedef cv::Mat videoframe_t;
typedef cv::VideoCapture videosource_t;

#ifdef GPU
typedef cv::cuda::GpuMat port_Mat;
#define port_cvtColor(src, dst, code) (cv::cuda::cvtColor((src), (dst), (code)))
#define port_equalizeHist(src, dst) (cv::cuda::equalizeHist((src), (dst)))
#define port_loadMatFromVideo(video_frame, mat) ((mat).upload((video_frame)))
#else
typedef cv::Mat port_Mat;
#define port_cvtColor(src, dst, code) (cv::cvtColor((src), (dst), (code)))
#define port_equalizeHist(src, dst) (cv::equalizeHist((src), (dst)))
#define port_loadMatFromVideo(video_frame, mat) ((video_frame).copyTo((mat)))
#endif


#define videosource_init(src, id) ((src).open((id)))
#define fetch_frame(src, dst) ((src) >> (dst)) 


#endif
