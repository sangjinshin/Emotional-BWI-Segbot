#include <ros/ros.h>
#include <std_msgs/String.h>

void emotion_face_callback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("%s", msg->data.c_str());
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "emotion_face");
	
	ros::NodeHandle nh_;
	
	ros::Subscriber emotion_face_sub_ = nh_.subscribe("emotion/face", 1, emotion_face_callback);
	
	ros::spin();
	
	return 0;
}
