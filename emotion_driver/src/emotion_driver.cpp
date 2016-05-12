#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

// For random number generator
#include <stdlib.h>
#include <time.h>

// Enumerated data types to define robot's movement commands
enum Movement
{
	MOVE_FORWARD = 0,
	TURN_LEFT = 1,
	TURN_RIGHT = 2,
	MOVE_BACKWARD = 3,
	STOP = 4
};

// Enumerated data types to define robot's emotion state
enum Emotion
{
	NEUTRAL = 0,
	HAPPY = 1,
	SAD = 2,
	ANGRY = 3
};

// Global variables instantiations
geometry_msgs::Twist velocity;
Movement current_movement;
Emotion current_emotion;
bool isMessagedReceived; 
bool isCommandFollowed;
unsigned long long int total_num_commands;
unsigned long long int num_yes;
unsigned long long int num_no;
double current_mood_level;
double diff_percentage;
double desire_level;
double happy_threshold;
double sad_threshold;
double angry_threshold;

// Methods instantiations
void segbot_teleop_handler(const geometry_msgs::Twist& msg);
float getNextCommand();
bool checkResponse();
Emotion determineMoodLevel(bool isCommandFollowed);
void resetVelocity();
void publish_emotion_face(ros::Publisher emotion_face_pub_, Emotion current_emotion);
void publish_emotion_info(ros::Publisher emotion_info_pub_);

int main(int argc, char** argv)
{
	// Initialize the ROS node
	ros::init(argc, argv, "emotion_driver");
	ros::NodeHandle nh_;
	
	// Initialize publishers and subscriber
	ros::Publisher vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 5);
	ros::Publisher emotion_info_pub_ = nh_.advertise<std_msgs::String>("emotion/info", 5);
	ros::Publisher emotion_face_pub_ = nh_.advertise<std_msgs::String>("emotion/face", 5);
	ros::Subscriber segbot_teleop_sub_ = nh_.subscribe("cmd_vel", 5, segbot_teleop_handler);

    // Seed random number generator
    srand(static_cast<unsigned>(time(0)));
		
    // Initialize global variables
    current_movement = STOP; // 4
    current_emotion = NEUTRAL; // 0
    total_num_commands = 0;
    num_yes = 0;
    num_no = 0;
    current_mood_level = 0.0;
    diff_percentage = 0.0;
    desire_level = 0.0;
    happy_threshold = static_cast<float>(rand())/(static_cast<float>(RAND_MAX/(1.1-0.1))); // 0.1 to 1.1, inclusive
    sad_threshold = (static_cast<float>(rand())/(static_cast<float>(RAND_MAX/(1.1-0.1)))) * -1; // -0.1 to -1.1, inclusive
    angry_threshold = sad_threshold - 1.0 - (static_cast<float>(rand())/(static_cast<float>(RAND_MAX/(1.1-0.1)))); // always less than (sad_threshold - 1.0)
    
    // ---------------START---------------
    
    // Robot commands what movement to do next and returns the desire_level of that command
    publish_emotion_face(emotion_face_pub_, current_emotion);
	desire_level = getNextCommand();
	ROS_INFO("Desire Level: [%f]\n", desire_level);
	
    while (nh_.ok())
	{		
		// Acquire linear and angular data from callback handler
		ros::spinOnce();
		
		// Wait until subscribed data has been received
		if (isMessagedReceived)
		{	
			// Stores velocity from the teleop node and checks if the user followed the robot's command
			isCommandFollowed = checkResponse();
		
			// Calculates change in mood level and determines the Emotion
			current_emotion = determineMoodLevel(isCommandFollowed);
		
			// Publishes information about the current state of robot's emotion
			publish_emotion_info(emotion_info_pub_);
			
			// Publishes face face of current emotion
			publish_emotion_face(emotion_face_pub_, current_emotion);
			
			// Resets velocity stored
			resetVelocity();
			
			// ---------------END---------------
			
			// ---------------REPEAT VIA LOOP---------------
			// Get robot's next command
			desire_level = getNextCommand();
			ROS_INFO("Desire Level: [%f]\n", desire_level);
			isMessagedReceived = false;
			
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
    int command = rand() % 4; // range 0 to 3
    desire_level = static_cast<float>(rand())/static_cast<float>(RAND_MAX);
    
    if (command == MOVE_FORWARD)
    {
        ROS_INFO("Move forward.");
        current_movement = MOVE_FORWARD;
    }
    else if (command == TURN_LEFT)
    {
        ROS_INFO("Turn left.");
        current_movement = TURN_LEFT;
    }
    else if (command == TURN_RIGHT)
    {
        ROS_INFO("Turn right.");
        current_movement = TURN_RIGHT;
    }
    else if (command == MOVE_BACKWARD)
    {
        ROS_INFO("Move backward.");
        current_movement = MOVE_BACKWARD;
    }
    
    return desire_level;
}

bool checkResponse()
{
    if (current_movement == MOVE_FORWARD && (velocity.linear.x <= 0.0 || velocity.angular.z != 0.0))
    {
        num_no++;
        return isCommandFollowed = false;
    }
    else if (current_movement == TURN_LEFT && (velocity.linear.x != 0.0 || velocity.angular.z <= 0.0))
    {
        num_no++;
        return isCommandFollowed = false;
    }
    else if (current_movement == TURN_RIGHT && (velocity.linear.x != 0.0 || velocity.angular.z >= 0.0))
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
		if (diff_percentage >= 0.0)
		{
			current_mood_level += (desire_level * diff_percentage);
		}
		else
		{
			current_mood_level -= (desire_level * diff_percentage);
		}
	}
	else
	{
		if (diff_percentage >= 0.0)
		{
			current_mood_level -= (desire_level * diff_percentage);
		}
		else
		{
			current_mood_level += (desire_level * diff_percentage);
		}
	}
	
	// Determine new Emotion
	Emotion new_emotion = current_emotion;
	if (current_mood_level >= 0.0 && current_mood_level < happy_threshold)
	{
		new_emotion = NEUTRAL;
		ROS_INFO("Meh. :|\n");
	}
	else if (current_mood_level >= happy_threshold)
	{
		new_emotion = HAPPY;
		ROS_INFO("I am happy! :)\n");
		
	}
	else if (current_mood_level < 0.0 && current_mood_level > angry_threshold) {
		new_emotion = SAD;
		ROS_INFO("I am sad...\n :'(");
	}
	else if (current_mood_level <= angry_threshold) {
		new_emotion = ANGRY;
		ROS_INFO("I AM ANGRY!!! >:(\n");
	}

	return new_emotion;
}

void resetVelocity()
{
    velocity.linear.x = 0.0;
    velocity.angular.z = 0.0;
}

void publish_emotion_face(ros::Publisher emotion_face_pub_, Emotion current_emotion)
{
	std_msgs::String emotion_face;
	std::ostringstream oss;
	if (current_emotion == NEUTRAL)
	{
		oss <<"\n"
			<<"                  000000000000000                 \n"
			<<"                 00000000000000000                \n"
			<<"            000000000000000000000000000           \n"
			<<"           0000000               0000000          \n"
			<<"        00000                         00000       \n"
			<<"       00000                           00000      \n"
			<<"     0000                                 0000    \n"
			<<"     0000                                 0000    \n"
			<<"     0000     0000000         0000000     0000    \n"
			<<"    000       0000000         0000000       000000\n"
			<<"  0000        0000000         0000000         0000\n"
			<<"  0000                                        0000\n"
			<<"  0000                                        0000\n"
			<<"  0000                                        0000\n"
			<<"  0000                                        0000\n"
			<<"  0000                                        0000\n"
			<<"  0000                                        0000\n"
			<<"    000                                     000000\n"
			<<"     0000                                 0000    \n"
			<<"     0000                                 0000    \n"
			<<"     0000         000000000000000         0000    \n"
			<<"       00000                           00000      \n"
			<<"        00000                         00000       \n"
			<<"           0000000               0000000          \n"
			<<"            000000               000000           \n"
			<<"                  000000000000000                 \n"
			<<"                  000000000000000                 \n\n";
	}
	else if (current_emotion == HAPPY)
	{
		oss <<"\n"
			<<"                  000000000000000                 \n"
			<<"                 00000000000000000                \n"
			<<"            000000000000000000000000000           \n"
			<<"           0000000               0000000          \n"
			<<"        00000                         00000       \n"
			<<"       00000                           00000      \n"
			<<"     0000                                 0000    \n"
			<<"     0000                                 0000    \n"
			<<"     0000     0000000         0000000     0000    \n"
			<<"    000       0000000         0000000       000000\n"
			<<"  0000        0000000         0000000         0000\n"
			<<"  0000                                        0000\n"
			<<"  0000                                        0000\n"
			<<"  0000                                        0000\n"
			<<"  0000                                        0000\n"
			<<"  0000      0                         0       0000\n"
			<<"  0000     0000                     0000      0000\n"
			<<"    000     00000                 00000     000000\n"
			<<"     0000     0000               0000     0000    \n"
			<<"     0000       0000000000000000000       0000    \n"
			<<"     0000         000000000000000         0000    \n"
			<<"       00000                           00000      \n"
			<<"        00000                         00000       \n"
			<<"           0000000               0000000          \n"
			<<"            000000               000000           \n"
			<<"                  000000000000000                 \n"
			<<"                  000000000000000                 \n\n";
	}
	else if (current_emotion == SAD)
	{
		oss <<"\n"
			<<"             000000000000000            \n"
			<<"          0000             0000         \n"
			<<"        000                   000       \n"
			<<"       00                       000     \n"
			<<"      00          00    0000000   00    \n"
			<<"     00          00           00   00   \n"
			<<"    00  00    000              000  00  \n"
			<<"   00     0000    00   00        00  0  \n"
			<<"   0           000 0   0 000         0  \n"
			<<"   0     000000    0   0    000000   0  \n"
			<<"   0       0    0000   0000    0     0  \n"
			<<"   0        000000       000000 0    0  \n"
			<<"   0                            0    0  \n"
			<<"   0                           0 0   0  \n"
			<<"   00                          0 0  00  \n"
			<<"    00           00000000      00  00   \n"
			<<"     00      00000      00000     00    \n"
			<<"      00   000     0000     000  00     \n"
			<<"       000        0    0        00      \n"
			<<"         000                  000       \n"
			<<"           00000         000000         \n"
			<<"               000000000000             \n\n";
	}
	else if (current_emotion == ANGRY)
	{
		oss <<"\n"
			<<"                    00000000000\n" 
			<<"               00000000000000000000\n" 
			<<"            00000000          00000000\n"
			<<"          00000                    000000\n"
			<<"        00000                         00000\n"
			<<"       0000                             0000\n" 
			<<"     0000    000                   000   0000\n" 
			<<"    0000     000                   000     000\n" 
			<<"   000       000                   000      000\n" 
			<<"  0000    000000                   000000    000\n" 
			<<"  000   00000000                   00000000   000\n" 
			<<" 000   0000   000                 000   0000  000\n" 
			<<" 000  000    00000                0000    000  00\n" 
			<<" 00  000     000000              00000     00  000\n" 
			<<"000  000     00000000           000000     000 000\n" 
			<<"000  000     0000000000000000000000000     00   00\n" 
			<<"000   000    0000000000000000000000000    000   00\n" 
			<<"000   0000   0000000000     0000000000   000   000\n" 
			<<"000    0000   00000000      000000000   0000   000\n" 
			<<" 00      000000000000         000000000000     000\n" 
			<<" 000       00000000             00000000       00\n" 
			<<" 000                                          000\n" 
			<<"  000                000000000               000\n" 
			<<"   000              00000000000              000\n" 
			<<"    000           0000       0000           000\n" 
			<<"    0000          000         000         0000\n" 
			<<"      0000                               0000\n" 
			<<"       0000                            0000\n" 
			<<"         00000                       00000\n" 
			<<"           000000                 000000\n" 
			<<"             000000000000000000000000\n\n"; 

	}
	
	emotion_face.data = oss.str();
	emotion_face_pub_.publish(emotion_face);
}

void publish_emotion_info(ros::Publisher emotion_info_pub_)
{
	std_msgs::String emotion_info;
	std::ostringstream oss;
	oss << "\nHappy Threshold: " << happy_threshold << "\n"
		<< "Sad Threshold: " << sad_threshold << "\n"
		<< "Angry Threshold: " << angry_threshold << "\n\n"
		<< "Current Movement: " << current_movement << "\n"
		<< "Current Emotion: " << current_emotion << "\n"
		<< "Current Mood Level: [" << current_mood_level << "]\n\n"
		<< "Desire Level: " << desire_level << "\n"
		<< "Total # Commands: " << total_num_commands << "\n"
		<< "# Commands Obeyed: " << num_yes << "\n"
		<< "# Commands Disobeyed: " << num_no << "\n"
		<< "Diff Percentage: [" << diff_percentage << "]\n";

	emotion_info.data = oss.str();
	emotion_info_pub_.publish(emotion_info);
}
