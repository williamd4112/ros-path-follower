#include "module_def.hpp"

#ifdef OBJECT_DETECT
static int32_t circle_find(const videoframe_t & frame, const videoframe_t & frame_hsv, const cv::Mat & frame_hsv_median_blur, const cv::Mat & mask_low, const cv::Mat & mask_upper, std::vector<cv::Rect> & targets, cv::Scalar debug_color)
{
    int32_t ret = 0;	
	
	cv::Mat mask;
    cv::add(mask_low, mask_upper, mask);
    cv::GaussianBlur(mask, mask, 
        cv::Size(OBJECT_DETECT_REDCIRCLE_BLUR_SIZE, OBJECT_DETECT_REDCIRCLE_BLUR_SIZE), 0);

#ifdef DEBUG_OBJECT_DETECT_REDCIRCLE
    cv::Mat _frame;
    cv::Mat _mask;
    frame.copyTo(_frame);
    mask.copyTo(_mask);
#endif
    for(cv::Rect & target : targets) {
        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(mask(target), circles, CV_HOUGH_GRADIENT, 1, 
            mask(target).rows / 8,
            20, 10, 2, 10000);
#ifdef DEBUG_OBJECT_DETECT_REDCIRCLE
        for (cv::Vec3f circle : circles) {
            cv::Point center(circle[0], circle[1]);
            int r = circle[2];
            cv::circle(_frame(target), center, r, debug_color, 2);        
        }
#endif
        if (circles.size() > 0) {
            ret++;
        }
    }

#ifdef DEBUG_OBJECT_DETECT_REDCIRCLE
    cv::imshow("circle_find-mask", _mask);
    cv::imshow("circle_find-frame", _frame);
#endif

    return ret;
}

int32_t redcircle_find(const videoframe_t & frame, const videoframe_t & frame_hsv, std::vector<cv::Rect> & targets)
{
    int32_t ret = 0;	
	cv::Mat frame_hsv_median_blur;
	cv::medianBlur(frame_hsv, frame_hsv_median_blur, 9);

    cv::Mat mask_low;
    cv::inRange(frame_hsv_median_blur, 
        cv::Scalar(0, 100, 100),
        cv::Scalar(10, 255, 255),
        mask_low);  
    cv::Mat mask_upper;
    cv::inRange(frame_hsv_median_blur, 
        cv::Scalar(160, 100, 100),
        cv::Scalar(179, 255, 255),
        mask_upper);
    

    return circle_find(frame, frame_hsv, frame_hsv_median_blur, mask_low, mask_upper, targets, cv::Scalar(0, 0, 255));
}

int32_t greencircle_find(const videoframe_t & frame, const videoframe_t & frame_hsv, std::vector<cv::Rect> & targets)
{
    int32_t ret = 0;	
	int32_t sensitivity = 10;

	cv::Mat frame_hsv_median_blur;
	cv::medianBlur(frame_hsv, frame_hsv_median_blur, 9);

    cv::Mat mask_low;
    cv::inRange(frame_hsv_median_blur, 
        cv::Scalar(60 - sensitivity, 100, 100),
        cv::Scalar(60 + sensitivity, 255, 255),
        mask_low);   

    return circle_find(frame, frame_hsv, frame_hsv_median_blur, mask_low, mask_low, targets, cv::Scalar(0, 255, 0));
}

#endif

