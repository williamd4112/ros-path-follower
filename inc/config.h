#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <cassert>
#include <cstdint>

#include <iostream>
#include <algorithm>

#include <vector>
#include <deque>

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/cudacodec.hpp>

#include "opencv2/cudaarithm.hpp"
#include "opencv2/cudafilters.hpp"  
#include "opencv2/cudaoptflow.hpp"
#include "opencv2/cudaimgproc.hpp"
#include "opencv2/cudawarping.hpp"

#define VIDEO_GROUND_WIDTH 320
#define VIDEO_GROUND_HEIGHT 240
#define VIDEO_FRONT_WIDTH 160
#define VIDEO_FRONT_HEIGHT 120
#define MOTION_DETECT_ROI_PADDING 20
#define MOMENT_DETECT_HEIGHT 50
#define MOMENT_DETECT_Y 100
#define OBJECT_DETECT_REDCIRCLE_BLUR_SIZE 3

#define OBJECT_DETECT_TRAFFIC_LIGHT_CASCADE_FILE_PATH "data/traffic_light.xml"

typedef cv::Mat videoframe_t;
typedef cv::VideoCapture videosource_t;

#ifdef GPU
typedef cv::cuda::GpuMat port_Mat;
#define port_cvtColor(src, dst, code) (cv::cuda::cvtColor((src), (dst), (code)))
#define port_equalizeHist(src, dst) (cv::cuda::equalizeHist((src), (dst)))
#define port_loadMatFromVideo(video_frame, mat) ((mat).upload((video_frame)))
#define port_warpPerspective(src, src_warp, new_homography_mat, size) cv::cuda::warpPerspective((src), (src_warp), (new_homography_mat), (size))
#define port_threshold(src_warp, mask, low, val, type) cv::cuda::threshold((src_warp), (mask), (low), (val), (type))
#define port_bitwise_and(src, dst, dst_masked, mask) cv::cuda::bitwise_and((src), (dst), (dst_masked), (mask))
#define port_absdiff(src, dst, diff) cv::cuda::absdiff((src), (dst), (diff))
#else
typedef cv::Mat port_Mat;
#define port_cvtColor(src, dst, code) (cv::cvtColor((src), (dst), (code)))
#define port_equalizeHist(src, dst) (cv::equalizeHist((src), (dst)))
#define port_loadMatFromVideo(video_frame, mat) ((video_frame).copyTo((mat)))
#define port_warpPerspective(src, src_warp, new_homography_mat, size) cv::warpPerspective((src), (src_warp), (new_homography_mat), (size))
#define port_threshold(src_warp, mask, low, val, type) cv::threshold((src_warp), (mask), (low), (val), (type))
#define port_bitwise_and(src, dst, dst_masked, mask) cv::bitwise_and((src), (dst), (dst_masked), (mask))
#define port_absdiff(src, dst, diff) cv::absdiff((src), (dst), (diff))
#endif


#define videosource_init(src, id) ((src).open((id)))
#define fetch_frame(src, dst) ((src) >> (dst)) 


#endif
