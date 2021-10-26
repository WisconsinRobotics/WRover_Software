#include "ros/ros.h"
#include "wr_drive_msgs/DriveTrainCmd.h"
#include "wr_drive_msgs/CamMastCmd.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "Watchdog.hpp"

#include <stdlib.h>
#include <stdio.h>

//Variables to hold the speeds and speed ratios for each side
float speedRatio[] = {0.0, 0.0};
float speedRaw[] = {0.0, 0.0};
//Holds the constant speed ratios (provided by parameters)
float SPEED_RATIO_VALUES[] = {0.25, 0.5, 0.75, 1.0};
//The camera mast speed value
float speedCamMast = 0.0;
//Cache for msg reception: [0] is left, [1] is right
bool msgCache[] = {false, false};

#define Std_Bool std_msgs::Bool::ConstPtr&
#define Std_Float32 std_msgs::Float32::ConstPtr&

/*
 * Drive train message
 */

//Define the output message
wr_drive_msgs::DriveTrainCmd output;

Watchdog dog(1);

//////////////////////////////////////////////////////////////////
//			BUTTON CALLBACKS			//
//								//
//	These update the speed ratios for their respective side //
// when their button is pressed.				//
//////////////////////////////////////////////////////////////////

//Generic Button Callback
void genCallback(const Std_Bool msg, int ind1, int ind2){
	if(msg->data) speedRatio[ind1]=SPEED_RATIO_VALUES[ind2];
	dog.pet();
}

//Left Drive Joystick
boost::function<void(const Std_Bool)>  L_S3_cb = [](const Std_Bool msg)->void{genCallback(msg, 0, 3);};
boost::function<void(const Std_Bool)>  L_S2_cb = [](const Std_Bool msg)->void{genCallback(msg, 0, 2);};
boost::function<void(const Std_Bool)>  L_S1_cb = [](const Std_Bool msg)->void{genCallback(msg, 0, 1);};
boost::function<void(const Std_Bool)>  L_S0_cb = [](const Std_Bool msg)->void{genCallback(msg, 0, 0);};

//Right Drive Joystick
boost::function<void(const Std_Bool)>  R_S0_cb = [](const Std_Bool msg)->void{genCallback(msg, 1, 0);};
boost::function<void(const Std_Bool)>  R_S1_cb = [](const Std_Bool msg)->void{genCallback(msg, 1, 1);};
boost::function<void(const Std_Bool)>  R_S2_cb = [](const Std_Bool msg)->void{genCallback(msg, 1, 2);};
boost::function<void(const Std_Bool)>  R_S3_cb = [](const Std_Bool msg)->void{genCallback(msg, 1, 3);};

void cachePublish();
ros::Publisher driveCommand;
ros::Publisher camCommand;

//////////////////////////////////////////////////////////////////
//		      JOYSTICK CALLBACKS		      //
//							      //
//      These update the raw speed for their respective side as //
// the joysticks are moved.					//
//////////////////////////////////////////////////////////////////

//Left Drive Joystick

void djL_axY_callback(const std_msgs::Float32::ConstPtr& msg){
	//speedRaw[0] = msg->data;
	output.left_value = msg->data*speedRatio[0];
	msgCache[0] = true;
	cachePublish();
	dog.pet();
}

//Right Drive Joystick

void djR_axY_callback(const std_msgs::Float32::ConstPtr& msg){
	//speedRaw[1] = msg->data;
	output.right_value = msg->data*speedRatio[1];
	msgCache[1] = true;
	cachePublish();
	dog.pet();
}

void djR_camMast_callback(const std_msgs::Float32::ConstPtr& msg){
	speedCamMast = msg->data;

	/*
	 * Camera mast message
	 */

	wr_drive_msgs::CamMastCmd cam_cmd;
	cam_cmd.turn_speed = speedCamMast;
	camCommand.publish(cam_cmd);

	dog.pet();
}

//////////////////////////////////////////////////////////////////
//			HELPER METHODS				//
//////////////////////////////////////////////////////////////////

void cachePublish() {
	// if both a left and right msg have been received
	if (msgCache[0] && msgCache[1]) {
		driveCommand.publish(output); // publish output values
		msgCache[0] = false; // reset the msg cache
		msgCache[1] = false;
	}
}

//Main Method

int main(int argc, char** argv){

	//ROS initialization
	ros::init(argc,argv,"TeleopDriveTrainLogic");

	//Handler to the current node
	ros::NodeHandle n;
	//Handler for private namespace
	ros::NodeHandle nh("~");

	//Get Custom Speed Parameters
	nh.getParam("speed_step1", SPEED_RATIO_VALUES[0]);
	nh.getParam("speed_step2", SPEED_RATIO_VALUES[1]);
	nh.getParam("speed_step3", SPEED_RATIO_VALUES[2]);
	nh.getParam("speed_step4", SPEED_RATIO_VALUES[3]);
	
	//Publisher for output data
	driveCommand = n.advertise<wr_drive_msgs::DriveTrainCmd>("/control/drive_system/cmd", 1000);
	camCommand = n.advertise<wr_drive_msgs::CamMastCmd>("/control/camera/cam_mast_cmd", 1000);
	
	//Set up dummy subscribers for input data
	ros::Subscriber s1, s2, s3, s4, s5, s6, s7, s8, sL, sR, sCamMast;
	
	//Assign the button callbacks to their respective topics
	s1 = n.subscribe("/logic/drive_system/joystick_left/button/3", 1000, L_S2_cb);
	s2 = n.subscribe("/logic/drive_system/joystick_left/button/4", 1000, L_S1_cb);
	s3 = n.subscribe("/logic/drive_system/joystick_left/button/5", 1000, L_S3_cb);
	s4 = n.subscribe("/logic/drive_system/joystick_left/button/6", 1000, L_S0_cb);
	s5 = n.subscribe("/logic/drive_system/joystick_right/button/3", 1000, R_S1_cb);
	s6 = n.subscribe("/logic/drive_system/joystick_right/button/4", 1000, R_S2_cb);
	s7 = n.subscribe("/logic/drive_system/joystick_right/button/5", 1000, R_S0_cb);
	s8 = n.subscribe("/logic/drive_system/joystick_right/button/6", 1000, R_S3_cb);

	//Assign the joystick callbacks to their respective topics
	sL = n. subscribe("/logic/drive_system/joystick_left/axis/stick_y", 1000, djL_axY_callback);
	sR = n. subscribe("/logic/drive_system/joystick_right/axis/stick_y", 1000, djR_axY_callback);

	//Subscriber for camera mast control
	sCamMast = n.subscribe("/logic/drive_system/joystick_right/axis/pov_x", 1000, djR_camMast_callback);

	//ROS Update cycler
	ros::spin();
}
