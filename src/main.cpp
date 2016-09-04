#include "config.h"
#include "util.hpp"
#include "keyboard.h"

#include "fsm.hpp"
#include "fsm_def.hpp"

#include "module_def.hpp"

#include <thread>

/*  Config variables */
static float g_linear_init = 0.1f;
static float g_angular_scale = 0.0055f;
static int g_green_range = 50;

/*  Global variables */
static fsm<state_t> g_fsm;

/*  Module instance */
#ifdef MOTION_DETECT
motion_detector motion(50, 50, 1000);
#endif

#ifdef LANE_DETECT
lane_detector lane;
#endif

#ifdef MOMENT_DETECT
moment_detector moment_white;
#endif

#ifdef OBJECT_DETECT
object_haar_detector object(OBJECT_DETECT_TRAFFIC_LIGHT_CASCADE_FILE_PATH);
#endif

#ifdef IMU
#endif

#define MODULE_ID_MOTION 0
#define MODULE_ID_LANE 1
#define MODULE_ID_MOMENT 2
#define MODULE_ID_OBJECT 3
#define MODULE_ID_IMU 4
#define is_module_enable(id) (g_module_enable[(id)])

static const char * g_module_name[] = {
    "Motion",
    "Lane",
    "Moment",
    "Object",
    "IMU"
};

static bool g_module_enable[] = {
#ifdef MOTION_DETECT
    true,
#else
    false,
#endif
#ifdef LANE_DETECT
    true,
#else
    false,
#endif
#ifdef MOMENT_DETECT
    true,
#else
    false,
#endif
#ifdef OBJECT_DETECT
    true,
#else
    false,
#endif
#ifdef IMU
    true
#else
    false
#endif
};

static size_t g_module_num = sizeof(g_module_enable) / sizeof(g_module_enable[0]);

static void self_check()
{
    std::cout << USE_GPU << std::endl 
            << USE_ROS_ADAPTER << std::endl
            << USE_VIDEO_PIPELINE << std::endl
            << USE_MOTION_DETECT << std::endl
            << USE_LANE_DETECT << std::endl
            << USE_MOMENT_DETECT << std::endl
            << USE_OBJECT_DETECT << std::endl;
}

static void check_module_enable()
{
    for (size_t i = 0; i < g_module_num; i++) {
        printf("%-10s = %-5s\n", g_module_name[i], bool2str(g_module_enable[i]));
    }
}

#ifdef DEBUG_FSM
static void check_fsm_table()
{
    std::cout << "State " << state_normal << " : Normal" << std::endl;
#ifdef MOTION_DETECT
    std::cout << "State " << state_motion << " : Motion" << std::endl;
#endif
#ifdef OBJECT_DETECT
    std::cout << "State " << state_traffic_light << " : Traffic light" << std::endl;
#endif
#ifdef IMU
    std::cout << "State " << state_seesaw_up << " : Seesaw up" << std::endl;
    std::cout << "State " << state_seesaw_up << " : Seesaw down" << std::endl;
    std::cout << "State " << state_seesaw_offset << " : Seesaw offset" << std::endl;
#endif
    std::cout << "State " << state_road_not_found << " : Road not found" << std::endl;
}
#endif

static void setup_fsm()
{
    /* Setup states */
    state_normal = g_fsm.set_state(STATE_NORMAL);
#ifdef MOTION_DETECT
    state_motion = g_fsm.set_state(STATE_MOTION);
#endif
#ifdef OBJECT_DETECT
    state_traffic_light = g_fsm.set_state(STATE_TRAFFIC_LIGHT);
#endif
#ifdef IMU
    state_seesaw_up = g_fsm.set_state(STATE_SEESAW_UP);
    state_seesaw_down = g_fsm.set_state(STATE_SEESAW_DOWN);
    state_seesaw_offset = g_fsm.set_state(STATE_SEESAW_OFFSET);
#endif
    state_road_not_found = g_fsm.set_state(STATE_ROAD_NOT_FOUND);

    /* Setup events */
#ifdef MOTION_DETECT
    event_normal_to_motion = g_fsm.set_event(state_normal, state_motion);
    event_motion_to_normal = g_fsm.set_event(state_motion, state_normal);
#endif
#ifdef OBJECT_DETECT
    event_normal_to_traffic_light = g_fsm.set_event(state_normal, state_traffic_light);
    event_traffic_light_to_normal = g_fsm.set_event(state_traffic_light, state_normal);
#endif
#ifdef IMU
    event_normal_to_seesaw_up = g_fsm.set_event(state_normal, state_seesaw_up);
    event_seesaw_up_to_seesaw_down = g_fsm.set_event(state_seesaw_up, state_seesaw_down);
    event_seesaw_down_to_seesaw_offset = g_fsm.set_event(state_seesaw_down, state_seesaw_offset);
#endif
    event_normal_to_road_not_found = g_fsm.set_event(state_normal, state_road_not_found);
    event_road_not_found_to_normal = g_fsm.set_event(state_road_not_found, state_normal);

    /* Setup initial state */
    g_fsm.set_init_state(state_normal);
}

static void process_fsm_state(const state_t & state)
{

}

static void process(videoframe_t & frame_front, videoframe_t & frame_ground)
{
    /*  Ground mask (green) */
    static cv::Scalar lower_green(60 - g_green_range, 100, 50);
    static cv::Scalar upper_green(60 + g_green_range, 255, 255);

    /* Frame pre-processing */
    videoframe_t frame_front_hsv;
    videoframe_t frame_ground_hsv;
    cv::cvtColor(frame_front, frame_front_hsv, CV_BGR2HSV);
    cv::cvtColor(frame_ground, frame_ground_hsv, CV_BGR2HSV);

    videoframe_t frame_front_gray;
    videoframe_t frame_ground_gray;
    cv::cvtColor(frame_front, frame_front_gray, CV_BGR2GRAY);
    cv::cvtColor(frame_ground, frame_ground_gray, CV_BGR2GRAY);

#ifdef MOMENT_DETECT
    float linear_moment = 1.0f;
    float angular_moment = 0.0f;
#endif
    float linear = g_linear_init;
    float angular = 0.0f;  

#ifdef IMU
#endif

    /*  Motion detect */
#ifdef MOTION_DETECT
    if (is_module_enable(MODULE_ID_MOTION)) {
        bool isMotion = false;
        if (motion.detect(frame_front) > 0) {
            g_fsm.fire_event(event_normal_to_motion);
            isMotion = true;
        }
        else {
            g_fsm.fire_event(event_motion_to_normal);
        }
        std::cout << "Motion (Front) : " << bool2str(isMotion) << std::endl;
    }
#endif
    /*  Lane detect */
#ifdef LANE_DETECT
    if (is_module_enable(MODULE_ID_LANE)) {
        lane.detect(frame_ground, frame_ground_hsv);
    }
#endif
    /*  Moment detect */
#ifdef MOMENT_DETECT
    if (is_module_enable(MODULE_ID_MOMENT)) {
        cv::Mat mask_ground;
        cv::Mat frame_ground_hsv_masked;
        int32_t moment_offset = 0;
        float moment_offset_scale = 0;

        cv::inRange(frame_ground, lower_green, upper_green, mask_ground);
        cv::bitwise_not(mask_ground, mask_ground);
        frame_ground_hsv.copyTo(frame_ground_hsv_masked, mask_ground);
        moment_offset = moment_white.detect(frame_ground, frame_ground_hsv_masked, cv::Rect(0, MOMENT_DETECT_Y, VIDEO_GROUND_WIDTH, MOMENT_DETECT_HEIGHT));
        if (moment_offset) {
            moment_offset_scale = normalize(moment_offset, VIDEO_GROUND_WIDTH >> 1);
        
            linear_moment = std::pow(moment_offset_scale + sign_float(moment_offset_scale) * 1.0f, -10);
            angular_moment = -((float)moment_offset) * g_angular_scale; 
        }
#ifdef DEBUG_MOMENT_DETECT
        std::cout << "Moment offset : " << moment_offset << "\tMoment offset scale : " << moment_offset_scale << "\tLinear moment : " << linear_moment << "\tAngular moment : " << angular_moment << std::endl;
#endif
    }
#endif
    /*  Object detect */
#ifdef OBJECT_DETECT
    if (is_module_enable(MODULE_ID_OBJECT)) {
        std::vector<cv::Rect> targets;
        object.detect(frame_front, frame_front_gray, targets);
#ifndef DEBUG_OBJECT_DETECT_REDCIRCLE
        if (targets.size() > 0) {
#endif
            int32_t traffic_light_count  = redcircle_find(frame_front, frame_front_hsv, targets);
            if (traffic_light_count > 0) {
                /// TODO : Trigget traffic light event
            }
#ifndef DEBUG_OBJECT_DETECT_REDCIRCLE
        }
#endif
    }
#endif

    /*  FSM */
#ifdef DEBUG_FSM
    check_fsm_table();
#endif
    g_fsm.update();

#ifdef MOMENT_DETECT
    linear *= linear_moment;
    angular = angular_moment;
#endif
    check_module_enable();
    std::cout << "Linear : " << linear << "\tAngular : " << angular << std::endl;
    
    process_fsm_state(g_fsm.peek());

#ifdef ROS_ADAPTER
    ros_adapter::update(linear, angular);
#endif
}

int main(int argc, char * argv[])
{
    /* Check module status */
    self_check();
    
    /* Setup FSM */
    setup_fsm();

#ifdef ROS_ADAPTER
    ros_adapter::init(argc, argv);
#endif

    int ground_cam_id = atoi(argv[1]);
    int front_cam_id = atoi(argv[2]);
    
    if (argc >= 4) {
        sscanf(argv[3], "%f", &g_linear_init);
        std::cout << "Init linear : " << g_linear_init << std::endl;
    }
    
    std::cout << "Path-Follower running on CPU " << sched_getcpu() << std::endl;

#ifdef VIDEO_PIPELINE   
    videostream video_front(front_cam_id, VIDEO_FRONT_WIDTH, VIDEO_FRONT_HEIGHT);
    videostream video_ground(ground_cam_id, VIDEO_GROUND_WIDTH, VIDEO_GROUND_HEIGHT);
#else
    videosource_t video_front(front_cam_id);
    video_front.set(CV_CAP_PROP_FRAME_WIDTH, VIDEO_FRONT_WIDTH);
    video_front.set(CV_CAP_PROP_FRAME_HEIGHT, VIDEO_FRONT_HEIGHT);
    
    videosource_t video_ground(ground_cam_id);
    video_ground.set(CV_CAP_PROP_FRAME_WIDTH, VIDEO_GROUND_WIDTH);
    video_ground.set(CV_CAP_PROP_FRAME_HEIGHT, VIDEO_GROUND_HEIGHT);
#endif

    videoframe_t frame_front, frame_ground;
    for(;;) {
        int key = keyboard::getKey();
        if (key == KEYCODE_ESC) {
            break;
        }
    
#ifdef DEBUG_FPS            
        double t = (double)cv::getTickCount();  
#endif

#ifdef VIDEO_PIPELINE
        video_front.read(frame_front);
        video_ground.read(frame_ground);
#else
        fetch_frame(video_front, frame_front);
        fetch_frame(video_ground, frame_ground);
#endif
        if (!frame_front.empty() && !frame_ground.empty()) {
            process(frame_front, frame_ground);
#ifdef DEBUG_FPS
            t = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
            std::cout << "FPS : " << (1.0 / t);
#endif
#ifdef DEBUG
#ifdef DEBUG_MAIN
            cv::imshow("frame-front", frame_front);
            cv::imshow("frame-ground", frame_ground):
#endif
            cv::waitKey(1);
#endif
        }
        printf("\033[H\033[J");
    }
    cv::destroyAllWindows();

#ifdef ROS_ADAPTER
    ros_adapter::shutdown();
#endif
    return 0;
}
