#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

// For random number generator
#include <stdlib.h>
#include <time.h>

enum Movement
{
	MOVE_FORWARD = 0,
	TURN_LEFT = 1,
	TURN_RIGHT = 2,
	MOVE_BACKWARD = 3,
	STOP = 4
};

enum Emotion
{
	NEUTRAL = 0,
	HAPPY = 1,
	SAD = 2,
	ANGRY = 3
};

geometry_msgs::Twist velocity;
Movement current_movement;
Emotion current_emotion;

bool isMessagedReceived;
bool isCommandFollowed;
unsigned short int total_num_commands;
unsigned short int num_yes;
unsigned short int num_no;
float happy_threshold;
float sad_threshold;
float angry_threshold;
float current_mood_level;
float diff_percentage;
float desire_level;

void segbot_teleop_handler(const geometry_msgs::Twist& msg);
float getNextCommand();
bool checkResponse();
Emotion determineMoodLevel(bool isCommandFollowed);
void doBehavior(Emotion emotion);
void resetVelocity();

int main(int argc, char** argv)
{
	//init the ROS node
	ros::init(argc, argv, "emotion_driver");
	ros::NodeHandle nh_;
	ros::Publisher vel_pub_ = nh_.advertise<geometry_msgs::Twist>("emotion/command_velocity", 1);
	ros::Publisher emotion_info_pub_ = nh_.advertise<std_msgs::String>("emotion/info", 1000);
	ros::Subscriber segbot_teleop_sub_ = nh_.subscribe("cmd_vel", 1000, segbot_teleop_handler);
	
	// Seed random number generator
	srand(static_cast<unsigned>(time(0)));
		
    	// Initialize variables to default values
    	current_movement = STOP;
	current_emotion = NEUTRAL;
    	total_num_commands = 0;
    	num_yes = 0;
    	num_no = 0;
    	current_mood_level = 0.0;
    	diff_percentage = 0.0;
    	desire_level = 0.0;
    	happy_threshold = static_cast<float>(rand())/(static_cast<float>(RAND_MAX/(1.1-0.1))); // 0.1 to 1.1, inclusive
    	sad_threshold = -(static_cast<float>(rand())/(static_cast<float>(RAND_MAX/(1.1-0.1)))); // 0.1 to -1.1, inclusive
    	angry_threshold = -(static_cast<float>(rand())/(static_cast<float>(RAND_MAX/(2.1-(0.1-sad_threshold))))); // (sad_threshold - 0.1) to -2.1, inclusive
    	
    	std_msgs::String msg;
	std::ostringstream oss;
    	bool isFirst = true;
	while (nh_.ok())
	{	
		if (isFirst)
		{
			// Robot commands what movement to do next and returns the desire_level of that command
			desire_level = getNextCommand();
		}
		
		// Acquire linear and angular data from callback handler
		ros::spinOnce();
		
		if (isMessagedReceived)
		{	
			// Stores velocity from the teleop node and checks if the user followed the robot's command
			isCommandFollowed = checkResponse();
		
			// Calculates change in mood level and determines the Emotion
			current_emotion = determineMoodLevel(isCommandFollowed);
		
			// Executes corresponding emotional behavior
			doBehavior(current_emotion);
		
			// Resets velocity stored
			resetVelocity();
			
			// Ouput Info
			oss << "Happy Threshold: " << happy_threshold << "\n"
				<< "Sad Threshold: " << sad_threshold << "\n"
				<< "Angry Threshold: " << angry_threshold << "\n"
				<< "Current Movement: " << current_movement << "\n"
				<< "Current Emotion: " << current_emotion << "\n"
				<< "Desire Level: " << desire_level << "\n"
				<< "Total # Commands: " << total_num_commands << "\n"
				<< "# Commands Obeyed: " << num_yes << "\n"
				<< "# Commands Disobeyed: " << num_no << "\n"
				<< "Current Mood Level: " << current_mood_level << "\n"
				<< "Diff Percentage: " << diff_percentage << "\n\n";
			msg.data = oss.str();
			emotion_info_pub_.publish(msg);
			
			isMessagedReceived = false;
			
			if (!isFirst)
			{
				desire_level = getNextCommand();
				isFirst = false;
			}
		}
	}
    
	return 0;
}

void segbot_teleop_handler(const geometry_msgs::Twist& msg)
{
	velocity.linear.x = msg.linear.x;
    	velocity.angular.z = msg.angular.z;
	isMessagedReceived = true;
}

float getNextCommand()
{
	total_num_commands++;
    	int val = rand() % 4; // range 0 to 3
    	desire_level = static_cast<float>(rand())/static_cast<float>(RAND_MAX);
    
    	if (val == MOVE_FORWARD)
    	{
    		ROS_INFO("Move forward.");
        	current_movement = MOVE_FORWARD;
    	}
    	else if (val == TURN_LEFT)
    	{
		ROS_INFO("Turn left.");
        	current_movement = TURN_LEFT;
    	}
    	else if (val == TURN_RIGHT)
    	{
        	ROS_INFO("Turn right.");
        	current_movement = TURN_RIGHT;
    	}
    	else if (val == MOVE_BACKWARD)
    	{
        	ROS_INFO("Move backward.");
        	current_movement = MOVE_BACKWARD;
    	}
    
    	return desire_level;
}

bool checkResponse()
{
	if (current_movement == STOP && (velocity.linear.x != 0.0 || velocity.angular.z != 0.0))
    	{
        	num_no++;
        	return isCommandFollowed = false;
    	}
    	else if (current_movement == MOVE_FORWARD && (velocity.linear.x <= 0.0 || velocity.angular.z != 0.0))
	{
        	num_no++;
        	return isCommandFollowed = false;
    	}
	else if (current_movement == TURN_LEFT && (velocity.linear.x != 0.0 || velocity.angular.z >= 0.0))
    	{
        	num_no++;
        	return isCommandFollowed = false;
    	}
    	else if (current_movement == TURN_RIGHT && (velocity.linear.x != 0.0 || velocity.angular.z <= 0.0))
    	{
        	num_no++;
        	return isCommandFollowed = false;
    	}
    	else if (current_movement == MOVE_BACKWARD && (velocity.linear.x >= 0.0 || velocity.angular.z != 0.0))
    	{
        	num_no++;
        	return isCommandFollowed = false;
    	}
    	else
    	{
		num_yes++;
		return isCommandFollowed = true;
    	}
}

Emotion determineMoodLevel(bool isCommandFollowed)
{
	// Calculate the change in the current_mood_level based on isCommandFollowed and percentage of commands followed and unfollowed
	diff_percentage = (static_cast<float>(num_yes)/static_cast<float>(total_num_commands)) - (static_cast<float>(num_no)/static_cast<float>(total_num_commands));
	if (isCommandFollowed)
	{
		current_mood_level += (desire_level * diff_percentage);
	}
	else
	{
		current_mood_level -= (desire_level * diff_percentage);
	}
	
	// Determine new Emotion
	Emotion new_emotion = current_emotion;
	if (current_mood_level >= 0.0 && current_mood_level < happy_threshold)
	{
		new_emotion = NEUTRAL;
	}
	else if (current_mood_level >= happy_threshold)
	{
		new_emotion = HAPPY;
	}
	else if (current_mood_level < 0.0 && current_mood_level > angry_threshold) {
		new_emotion = SAD;
	}
	else if (current_mood_level <= angry_threshold) {
		new_emotion = ANGRY;
	}
	
	return new_emotion;
}

void doBehavior(Emotion emotion)
{
	geometry_msgs::Twist vel_msg;
    	if (emotion == NEUTRAL)
    	{
		ROS_INFO("Meh..");
    	}
    	else if (emotion == HAPPY) {
        	ROS_INFO("I am happy!");
    	}
    	else if (emotion == SAD) {
        	ROS_INFO("I am sad...");
    	}
    	else if (emotion == ANGRY) {
        	ROS_INFO("I AM ANGRY!!!");
    	}
}

void resetVelocity()
{
	velocity.linear.x = 0.0;
    	velocity.angular.z = 0.0;
}
