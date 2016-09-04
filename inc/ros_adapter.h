#ifndef _ROS_ADAPTER_H_
#define _ROS_ADAPTER_H_

namespace ros_adapter
{
	void init(int argc, char * argv[]);
	void update(float linear, float angular);
    void move_back(float t, float linear=-0.1f, float angular=0.0f);
	void shutdown();
}

#endif
