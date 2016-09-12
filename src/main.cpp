#include "config.h"
#include "util.hpp"
#include "keyboard.h"

#include "fsm.hpp"
#include "fsm_def.hpp"

#include "module_def.hpp"

#include <thread>
#include <sys/time.h> 

/*  Config variables */
static float g_linear_init = 0.1f;
static float g_angular_scale = 0.0060f;
static float g_linear_slow_rate = -6.0f;

static int g_green_range = 50;

/*  Global variables */
static fsm<state_t> g_fsm;

/*  Module instance */
#ifdef MOTION_DETECT
motion_detector motion(35, 50, 1000);
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
#define ON true
#define OFF false
#define set_module(id, en) (g_module_enable[(id)] = (en))

#define RED_LIGHT_TIMEOUT 20

static int g_screenshot_cnt = 0;

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
	sleep(3);
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
	switch(state) {
		case STATE_NORMAL:
#ifdef MOTION_DETECT
			set_module(MODULE_ID_MOTION, ON);
#endif
#ifdef LANE_DETECT
			set_module(MODULE_ID_LANE, ON);
#endif
#ifdef MOMENT_DETECT
			set_module(MODULE_ID_MOMENT, ON);
#endif
#ifdef OBJECT_DETECT
			set_module(MODULE_ID_OBJECT, ON);
#endif
#ifdef IMU
			set_module(MODULE_ID_IMU, ON);
#endif
			break;
#ifdef MOTION_DETECT
		case STATE_MOTION:
			set_module(MODULE_ID_MOTION, ON);
#ifdef LANE_DETECT
			set_module(MODULE_ID_LANE, OFF);
#endif
#ifdef MOMENT_DETECT
			set_module(MODULE_ID_MOMENT, OFF);
#endif
#ifdef OBJECT_DETECT
			set_module(MODULE_ID_OBJECT, OFF);
#endif
#ifdef IMU
			set_module(MODULE_ID_IMU, OFF);
#endif
			break;
#endif
#ifdef OBJECT_DETECT
		case STATE_TRAFFIC_LIGHT:
#ifdef MOTION_DETECT
			set_module(MODULE_ID_MOTION, ON);
#endif
#ifdef LANE_DETECT
			set_module(MODULE_ID_LANE, OFF);
#endif
#ifdef MOMENT_DETECT
			set_module(MODULE_ID_MOMENT, OFF);
#endif
			set_module(MODULE_ID_OBJECT, ON);
#ifdef IMU
			set_module(MODULE_ID_IMU, OFF);
#endif
			break;
#endif
#ifdef IMU
		case STATE_SEESAW_UP:
#ifdef MOTION_DETECT
			set_module(MODULE_ID_MOTION, OFF);
#endif
#ifdef LANE_DETECT
			set_module(MODULE_ID_LANE, ON);
#endif
#ifdef MOMENT_DETECT
			set_module(MODULE_ID_MOMENT, ON);
#endif
#ifdef OBJECT_DETECT
			set_module(MODULE_ID_OBJECT, OFF);
#endif
			set_module(MODULE_ID_IMU, ON);
			break;
		case STATE_SEESAW_DOWN:
#ifdef MOTION_DETECT
			set_module(MODULE_ID_MOTION, OFF);
#endif
#ifdef LANE_DETECT
			set_module(MODULE_ID_LANE, ON);
#endif
#ifdef MOMENT_DETECT
			set_module(MODULE_ID_MOMENT, OFF);
#endif
#ifdef OBJECT_DETECT
			set_module(MODULE_ID_OBJECT, OFF);
#endif
			set_module(MODULE_ID_IMU, ON);
			break;
		case STATE_SEESAW_OFFSET:
#ifdef MOTION_DETECT
			set_module(MODULE_ID_MOTION, OFF);
#endif
#ifdef LANE_DETECT
			set_module(MODULE_ID_LANE, OFF);
#endif
#ifdef MOMENT_DETECT
			set_module(MODULE_ID_MOMENT, ON);
#endif
#ifdef OBJECT_DETECT
			set_module(MODULE_ID_OBJECT, OFF);
#endif
			set_module(MODULE_ID_IMU, ON);
			break;
#endif
		case STATE_ROAD_NOT_FOUND:
#ifdef MOTION_DETECT
			set_module(MODULE_ID_MOTION, ON);
#endif
#ifdef LANE_DETECT
			set_module(MODULE_ID_LANE, OFF);
#endif
#ifdef MOMENT_DETECT
			set_module(MODULE_ID_MOMENT, ON);
#endif
#ifdef OBJECT_DETECT
			set_module(MODULE_ID_OBJECT, ON);
#endif
#ifdef IMU
			set_module(MODULE_ID_IMU, OFF);
#endif
			break;
		default:
			std::cout << "Unhandled state." << std::endl;
			break;	
	}
}

static void process(videoframe_t & frame_front, videoframe_t & frame_ground, double elapse_time)
{
	static double delay_cnt = 0.0;
#ifdef SMOOTH
	static float last_linear = 0.0f;
#endif
    /*  Ground mask (green) */
    static cv::Scalar lower_green(60 - g_green_range, 100, 50);
    static cv::Scalar upper_green(60 + g_green_range, 255, 255);

    /*	Frame pre-processing */
    videoframe_t frame_front_hsv;
    videoframe_t frame_ground_hsv;
    cv::cvtColor(frame_front, frame_front_hsv, CV_BGR2HSV);
    cv::cvtColor(frame_ground, frame_ground_hsv, CV_BGR2HSV);

    videoframe_t frame_front_gray;
    videoframe_t frame_ground_gray;
    cv::cvtColor(frame_front, frame_front_gray, CV_BGR2GRAY);
    cv::cvtColor(frame_ground, frame_ground_gray, CV_BGR2GRAY);
	
	/*	Special purpose delay	*/
	float delay = 0.0f;
#ifdef MOTION_DETECT
	float linear_motion = 0.0f;
	float angular_motion = 0.0f;
#endif
#ifdef MOMENT_DETECT
    float linear_moment = 0.0f;
    float angular_moment = 0.0f;
#endif
#ifdef OBJECT_DETECT
	float linear_object = 0.0f;
	float angular_object = 0.0f;
#endif
    float linear = g_linear_init;
    float angular = 0.0f;  

	/* Delay counter */
	if (delay_cnt > 0) {
		std::cout << "Delay " << delay_cnt << std::endl;
		delay_cnt -= elapse_time;
		if (delay_cnt <= 0) {
			delay_cnt= 0;
		}
		return;
	}
	
	/*	IMU */
#ifdef IMU
#endif

    /*  Motion detect */
#ifdef MOTION_DETECT
    if (is_module_enable(MODULE_ID_MOTION)) {
        bool isMotion = false;
        if (motion.detect(frame_ground, cv::Rect(MOTION_DETECT_ROI_PADDING, MOTION_DETECT_ROI_PADDING, 
						VIDEO_GROUND_HEIGHT - MOTION_DETECT_ROI_PADDING, 
						VIDEO_GROUND_HEIGHT - MOTION_DETECT_ROI_PADDING)) > 0) {
            g_fsm.fire_event(event_normal_to_motion);
            isMotion = true;
			linear_motion = 0.0f;
			angular_motion = 0.0f;
			delay_cnt += 0.01;
        }
        else {
            g_fsm.fire_event(event_motion_to_normal);
			linear_motion = 1.0f;
			angular_motion = 0.0f;
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
        
		auto moment_ret = moment_white.detect(frame_ground, frame_ground_hsv_masked, cv::Rect(0, MOMENT_DETECT_Y, VIDEO_GROUND_WIDTH, MOMENT_DETECT_HEIGHT));       
		
		/*	Road found */
		if (moment_ret.second) {
			moment_offset = moment_ret.first;
            moment_offset_scale = normalize(moment_offset, VIDEO_GROUND_WIDTH >> 1);
        
            linear_moment = std::pow(moment_offset_scale + sign_float(moment_offset_scale) * 1.0f, -6);
            angular_moment = -((float)moment_offset) * g_angular_scale; 
			g_fsm.fire_event(event_road_not_found_to_normal);
#ifdef DEBUG_MOMENT_DETECT
			std::cout << "Road : found" << std::endl;
#endif
        }
		/*	Road not found */
		else {
			linear_moment = 0.0f;
			angular_moment = 0.0f;
			g_fsm.fire_event(event_normal_to_road_not_found);
#ifdef DEBUG_MOMENT_DETECT
			std::cout << "Road : not found" << std::endl;
#endif
		}
#ifdef DEBUG_MOMENT_DETECT
        std::cout << "Moment offset : " << moment_offset 
				<< "\tMoment offset scale : " << moment_offset_scale 
				<< "\tLinear moment : " << linear_moment 
				<< "\tAngular moment : " << angular_moment << std::endl;
#endif
    }
#endif
    /*  Object detect */
#ifdef OBJECT_DETECT
    if (is_module_enable(MODULE_ID_OBJECT)) {
		bool isTrafficLight = false | (g_fsm.peek() == STATE_TRAFFIC_LIGHT);
        std::vector<cv::Rect> targets;
        object.detect(frame_front, frame_front_gray, targets);
		linear_object = 1.0f;
		angular_object = 0.0f;
        if (targets.size() > 0) {
			int32_t traffic_light_count  = redcircle_find(frame_front, frame_front_hsv, targets);
			if (traffic_light_count > 0) {
				linear_object = 0.0f;
				angular_object = 0.0f;
				isTrafficLight = true;

				g_fsm.fire_event(event_normal_to_traffic_light);

#ifdef DEBUG_OBJECT_RECORD
				char buff[100];
				sprintf(buff, "object%d.jpg", g_screenshot_cnt++);
				cv::imwrite(buff, frame_front);
#endif
			}
			else if(greencircle_find(frame_front, frame_front_hsv, targets)){
				g_fsm.fire_event(event_traffic_light_to_normal);			
			}

        }
		std::cout << "Traffic Light : " << bool2str(isTrafficLight) << std::endl;
    }	
#endif

    /*  FSM */
#ifdef DEBUG_FSM
    check_fsm_table();
#endif
    bool is_diff = g_fsm.update();

#ifdef MOMENT_DETECT
    linear *= linear_moment;
    angular += angular_moment;
#endif
#ifdef MOTION_DETECT
	linear *= linear_motion;
	angular += angular_motion;
#endif
#ifdef OBJECT_DETECT
	linear *= linear_object;
	angular += angular_object;
#endif
	if (g_fsm.peek() == STATE_ROAD_NOT_FOUND) {
		linear = -0.1f;
		delay_cnt = 0.1;
	}

#ifdef DEBUG
    check_module_enable();
    std::cout << "Linear : " << linear << "\tAngular : " << angular << std::endl;
#endif
    process_fsm_state(g_fsm.peek());

#ifdef ROS_ADAPTER
#ifdef SMOOTH
	float quant = 100.0f;
	float linear_diff = linear - last_linear;
	for (int i = 0; i < (int)quant; i++) {
		ros_adapter::update(last_linear + linear_diff * (float)i / quant, angular);
	}
	last_linear = linear;
#else
	ros_adapter::update(linear, angular);
#endif
#endif
	/*	State transition time */
	if (is_diff) {
#ifdef DEBUG
		std::cout << "State transition ..." << std::endl;
#endif
		delay_cnt = 0.001;
	}
	std::cout << " Elapse : " << elapse_time << std::endl;
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

#ifdef VIDEO_PLAYBACK
	cv::VideoWriter playback(argv[4],CV_FOURCC('M','J','P','G'), 10, cv::Size(VIDEO_FRONT_WIDTH, VIDEO_FRONT_HEIGHT),true);
#endif

    videoframe_t frame_front, frame_ground;
	double elapse = 0;
    for(;;) {
        int key = keyboard::getKey();
        if (key == KEYCODE_ESC) {
            break;
        }
    
        double t = (double)cv::getTickCount();  

#ifdef VIDEO_PIPELINE
        video_front.read(frame_front); video_ground.read(frame_ground);
#else
        fetch_frame(video_front, frame_front);
        fetch_frame(video_ground, frame_ground);
#endif
        if (!frame_front.empty() && !frame_ground.empty()) {
            process(frame_front, frame_ground, elapse);
            elapse = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
#ifdef DEBUG_FPS
            std::cout << "FPS : " << (1.0 / elapse);
#endif
#ifdef VIDEO_PLAYBACK
		playback.write(frame_front);
#endif
#ifdef DEBUG
#ifdef DEBUG_MAIN
            cv::imshow("frame-front", frame_front);
            cv::imshow("frame-ground", frame_ground);
#endif
            cv::waitKey(1);
#endif
        }
#ifdef DEBUG
		printf("\n==========================================\n");
        printf("\033[H\033[J");
#endif
    }
    cv::destroyAllWindows();

#ifdef ROS_ADAPTER
    ros_adapter::shutdown();
#endif
    return 0;
}
