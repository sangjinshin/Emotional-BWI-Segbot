#include <ros/ros.h>
#include <std_msgs/String.h>

void emotion_gui_callback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("%s", msg->data.c_str());
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "emotion_gui");
	
	ros::NodeHandle nh_;
	
	ros::Subscriber emotion_gui_sub_ = nh_.subscribe("emotion/gui", 1, emotion_gui_callback);
	
	ros::spin();
	
	return 0;
}
