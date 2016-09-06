#include "module_def.hpp"

#ifdef OBJECT_DETECT
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
            5, 10, 0, 10000);
#ifdef DEBUG_OBJECT_DETECT_REDCIRCLE
        for (cv::Vec3f circle : circles) {
            cv::Point center(circle[0], circle[1]);
            int r = circle[2];
            cv::circle(_frame(target), center, r, cv::Scalar(0, 255, 0), 2);        
        }
#endif
        if (circles.size() > 0) {
            ret++;
        }
    }

#ifdef DEBUG_OBJECT_DETECT_REDCIRCLE
    cv::imshow("redcircle_find-mask", _mask);
    cv::imshow("redcircle_find-frame", _frame);
#endif

    return ret;
}
#endif

