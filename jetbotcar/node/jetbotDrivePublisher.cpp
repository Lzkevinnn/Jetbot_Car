#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <chrono>
#include <iostream>
#include <cstdlib>
#include <unistd.h>
#include <time.h>
//#include "~/newJetpackage/devel/include/jetbotcar/jetdrivemsg.h"
#include "jetbotcar/Jetdrivemsg.h"
#include <iostream>
#include <cstdio>
#include <ctime>
#include "ros/ros.h"

using namespace std;

class jetbotDriveCmd
{
private:
    ros::NodeHandle n;


    ros::Subscriber key_sub;

    ros::Publisher diff_drive_pub;

    ros::Subscriber radius_sub;

    ros::Subscriber e_stop_sub; //subscriber for the e-stop button

    ros::Subscriber loopdrive_sub; // subscriber for the loopdrive function

    double prev_key_velocity = 0.0;
    double keyboard_max_speed = 1.0;
    double turnRadius = 1;
    double rotationWheelSpeedScale, trackWidth;
    double constantRadiusLeftWheelSpeed, constantRadiusRightWheelSpeed;

    bool OpenLoopDrive;
    double leftWheelSpeed;
    double rightWheelSpeed;

public:
    jetbotDriveCmd()
    {
        n = ros::NodeHandle("~");

        std::string diff_drive_topic, mux_topic, joy_topic, key_topic , radiusTopic;
	    n.getParam("diff_drive_topic", diff_drive_topic);
        n.getParam("keyboard_topic", key_topic);
        n.getParam("jetbot_rotation_wheel_speed_scale", rotationWheelSpeedScale); // rotationWheelSpeedScale = 0.5
        n.getParam("jetbot_width", trackWidth);
        n.getParam("radius_topic" , radiusTopic);


        //diff_drive_pub = n.advertise<std_msgs::Float64MultiArray>(diff_drive_topic, 10);
        // jetbot:jetdrivemsg: templete including type of message
        diff_drive_pub = n.advertise<jetbotcar::Jetdrivemsg>(diff_drive_topic , 10);

        // The second parameter is the size of the message queue,
        // If messages are arriving faster than they are being processed, this
        // is the number of messages that will be buffered up before beginning to throw
        // away the oldest ones.

        // key_sub is the subscriber, and subscribe the topic named "key_topic"
        // radius_sub is the subscriber, and subscribe the topic named "radius_sub"
        // follow & is the address of 
        key_sub = n.subscribe(key_topic, 1, &jetbotDriveCmd::key_callback, this);
        radius_sub = n.subscribe(radiusTopic , 1, &jetbotDriveCmd::radiusCalc, this);
        e_stop_sub = n.subscribe(key_topic, 1, &jetbotDriveCmd::e_stop, this);
        loopdrive_sub = n.subscribe(key_topic, 1, &jetbotDriveCmd::loopdrive, this);
    }

    void radiusCalc(const std_msgs::Float64 & msg){
        if (msg.data > 0){
            constantRadiusRightWheelSpeed = 1;
           
            constantRadiusLeftWheelSpeed = \
            ((msg.data-trackWidth)/(msg.data + trackWidth));
           
            publish_to_diff_drive(constantRadiusRightWheelSpeed, constantRadiusLeftWheelSpeed);  
        }else if (msg.data < 0){ 
            constantRadiusLeftWheelSpeed = 1;

            constantRadiusRightWheelSpeed = \
            ((std::abs(msg.data)-trackWidth)/(std::abs(msg.data) + trackWidth));
           
            publish_to_diff_drive(constantRadiusRightWheelSpeed, constantRadiusLeftWheelSpeed);       

        }else{
            ROS_INFO("enter non-zero turn message");

        }

    }
    void publish_to_diff_drive(double rightWheelTrq,double leftWheelTrq)
    {
        /*std_msgs::Float64MultiArray diffDriveMsg;
	diffDriveMsg.data.clear();
	diffDriveMsg.data.push_back(rightWheelTrq);
	diffDriveMsg.data.push_back(leftWheelTrq);
        diff_drive_pub.publish(diffDriveMsg);*/
        jetbotcar::Jetdrivemsg diffdrivemsg;
        diffdrivemsg.left = leftWheelTrq;
        diffdrivemsg.right  = rightWheelTrq;
        diff_drive_pub.publish(diffdrivemsg);
    }

    void e_stop(const std_msgs::String & msg){
        double leftWheelSpeed;
        double rightWheelSpeed;
        if (msg.data == "h"){
            // leftWheelSpeed = 0.0;
            // rightWheelSpeed = 0.0;
            // publish_to_diff_drive(rightWheelSpeed , leftWheelSpeed);
            // exit(0);
            ros::shutdown();
        }
    }
    void loopdrive(const std_msgs::String & msg){

        // OpenLoopDrive = true;
        
        if(msg.data == "y"){
        
            ros::Timer timer = n.createTimer(ros::Duration(3),\
             &jetbotDriveCmd::timerCallback, this); //timer-set

            OpenLoopDrive = true; //set OpenLoopDrive to true

            leftWheelSpeed = 0.5;
            rightWheelSpeed = 0.5;           
            publish_to_diff_drive(rightWheelSpeed , leftWheelSpeed);
        }
        // if(OpenLoopDrive){
        //     publish_to_diff_drive(rightWheelSpeed , leftWheelSpeed);
        // } 

    }

    void timerCallback(const ros::TimerEvent& event){
        OpenLoopDrive = false;
        leftWheelSpeed = 0.0;
        rightWheelSpeed = 0.0;
        publish_to_diff_drive(rightWheelSpeed , leftWheelSpeed);

    }

    void key_callback(const std_msgs::String & msg){
        // double leftWheelSpeed;
        // double rightWheelSpeed;
        bool publish = true;

        if (msg.data == "w"){
            leftWheelSpeed = 1.0;
            rightWheelSpeed = 1.0;
        
        }else if(msg.data=="s"){
            leftWheelSpeed = -1.0;
            rightWheelSpeed = -1.0;

        }else if(msg.data == "a"){
            leftWheelSpeed = -1.0*rotationWheelSpeedScale;
            rightWheelSpeed = 1.0*rotationWheelSpeedScale;

        }else if(msg.data == "d") {
            leftWheelSpeed = 1.0*rotationWheelSpeedScale;
            rightWheelSpeed = -1.0*rotationWheelSpeedScale;
        }else if (msg.data ==" "){
            leftWheelSpeed = 0.0;
            rightWheelSpeed = 0.0;
        }else if(msg.data == "q"){
            leftWheelSpeed =   constantRadiusLeftWheelSpeed;
            rightWheelSpeed =  constantRadiusRightWheelSpeed;
        }else if(msg.data == "o"){ //open loop drive in a circle
            leftWheelSpeed = 0.8*rotationWheelSpeedScale;
            rightWheelSpeed = 1.1*rotationWheelSpeedScale;
        }
        else{
            publish = false; // no action while pressing other
        }
        if (publish){ // transmit speed command to left and right motors
            publish_to_diff_drive(rightWheelSpeed , leftWheelSpeed);
        }else if(msg.data == "e"){            
            leftWheelSpeed = 0.0;
            rightWheelSpeed = 0.0;
            publish_to_diff_drive(rightWheelSpeed , leftWheelSpeed);
            ros::shutdown();
        }
          
    }
   
};
int main(int argc, char ** argv){
    ros::init(argc, argv, "jetbotDriveCmd");
    jetbotDriveCmd jetDriver;
    ros::spin(); // infinte loop waiting for callback funtion
    return 0;
}


