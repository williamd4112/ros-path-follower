#ifndef _ROS_ADAPTER_H_
#define _ROS_ADAPTER_H_

namespace ros_adapter
{
	void init(int argc, char * argv[]);
	void update(float linear, float angular);
	void shutdown();
}

#endif
