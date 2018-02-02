#include <ros/ros.h>
#include <std_msgs/String.h>

void emotion_info_callback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO("%s", msg->data.c_str());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "emotion_info");

  ros::NodeHandle nh_;

  ros::Subscriber emotion_info_sub_ = nh_.subscribe("emotion/info", 1,
                                                    emotion_info_callback);

  ros::spin();

  return 0;
}
