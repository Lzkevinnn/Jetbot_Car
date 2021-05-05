#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

#include "jetbotcar/Jetdrivemsg.h"
#include "jetbotcar/jetRacerDriveMsg.h"
class jetracerDriveCmd
{
private:
    ros::NodeHandle n;


    ros::Subscriber key_sub;

    ros::Publisher diff_drive_pub;
    ros::Publisher jetracer_drive_pub;
    
    ros::Subscriber radius_sub;


    double prev_key_velocity = 0.0;
    double keyboard_max_speed = 1.0;
    double turnRadius = 1;
    double rotationWheelSpeedScale, trackWidth;
    double constantRadiusLeftWheelSpeed, constantRadiusRightWheelSpeed;
    double openloopdrive;
    double throttle;
    double steering;

public:
    jetracerDriveCmd()
    {
        n = ros::NodeHandle("~");

        std::string diff_drive_topic, mux_topic, joy_topic, key_topic , radiusTopic, jetracer_drive_topic;
	    n.getParam("diff_drive_topic", diff_drive_topic);
        n.getParam("jetracer_drive_topic", jetracer_drive_topic);
        n.getParam("keyboard_topic", key_topic);
        n.getParam("jetbot_rotation_wheel_speed_scale", rotationWheelSpeedScale);
        n.getParam("jetbot_width", trackWidth);
        n.getParam("radius_topic" , radiusTopic);


        
        diff_drive_pub = n.advertise<jetbotcar::Jetdrivemsg>(diff_drive_topic , 10);
        jetracer_drive_pub = n.advertise<jetbotcar::jetRacerDriveMsg>(jetracer_drive_topic, 10);

        key_sub = n.subscribe(key_topic, 1, &jetracerDriveCmd::key_callback, this);
        radius_sub = n.subscribe(radiusTopic , 1, &jetracerDriveCmd::radiusCalc, this);

    }

    // this fucntion has to be rewritten for jetracer
    void radiusCalc(const std_msgs::Float64 & msg){
        if (msg.data > 0){
            constantRadiusRightWheelSpeed = 1;
            constantRadiusLeftWheelSpeed = ((msg.data-trackWidth)/(msg.data + trackWidth));
        }else if(msg.data< 0){
            constantRadiusLeftWheelSpeed = 1;
            constantRadiusRightWheelSpeed = ((std::abs(msg.data)-trackWidth)/(std::abs(msg.data) + trackWidth));
        }else{
            ROS_INFO("turn radius topic read");
        }
        

    }
    /*void publish_to_diff_drive(double rightWheelTrq,double leftWheelTrq)
    {
        
        jetbotcar::Jetdrivemsg diffdrivemsg;
        diffdrivemsg.left = leftWheelTrq;
        diffdrivemsg.right  = rightWheelTrq;
        diff_drive_pub.publish(diffdrivemsg);
    }*/




    /* publishing function */
    void publishtoJetracer(double throttle, double steering){
        // jetbotcar::jetRacerDriveMsg jetracerMsg;
        // jetracerMsg.throttle = throttle;
        // jetracerMsg.steering = steering;
        // jetracer_drive_pub.publish(jetracerMsg);
    }

    void key_callback(const std_msgs::String & msg){
        /*double leftWheelSpeed;
        double rightWheelSpeed;*/


        bool publish = true;

        if (msg.data == "w"){
            throttle = -0.8;
            steering = 0.0;

        }else if(msg.data=="s"){
            throttle = 0.8;
            steering = 0;

        }else if(msg.data == "a"){
            throttle = -0.55;
            steering = -1;

        }else if(msg.data == "d") {
            throttle = -0.55;
            steering = 1;
        }else if (msg.data ==" "){
            throttle = 0.0;
            steering = 0.0;
        }else if(msg.data == "q"){
            throttle =   0.5;
            steering =  0.5;
        }else if(msg.data == "l"){
            ros::Timer timer = n.createTimer(ros::Duration(3),\
             &jetracerDriveCmd::timerCallback, this); //timer-set

            openloopdrive = true;
            
            throttle = 0.5;
            steering = 0;

        }else {
            publish = false;
        }
        if (publish){
            publishtoJetracer(throttle , steering);

        }else if(msg.data == "e"){            
            throttle = 0.0;
            steering = 0.0;
            publishtoJetracer(throttle , steering);
            ros::shutdown();
        }
    }
    void timerCallback(const ros::TimerEvent& ){
        openloopdrive = false;
        throttle = 0.0;
        steering = 0.0;
        publishtoJetracer(throttle , steering);

    }
};
int main(int argc, char ** argv){
  ros::init(argc, argv, "jetRacerDriveCmd");
  jetracerDriveCmd jetDriver;
  ros::spin();
  return 0;
  }


