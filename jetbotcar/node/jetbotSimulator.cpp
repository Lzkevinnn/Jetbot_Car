#include <ros/ros.h>

// interactive marker
#include <interactive_markers/interactive_marker_server.h>

#include <tf2/impl/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>



#include "jetbotcar/car_state.hpp"
#include "jetbotcar/car_params.hpp"
#include "jetbotcar/jetbotDynamics.hpp"
#include "jetbotcar/Jetdrivemsg.h"
#include <iostream>
#include <math.h>

using namespace racecar_simulator;

class jetbotSimulator{
private:
     // A ROS node
    ros::NodeHandle n;

    // The transformation frames used
    std::string map_frame, base_frame, scan_frame;

    // obstacle states (1D index) and parameters
    std::vector<int> added_obs;
    // listen for clicked point for adding obstacles
    ros::Subscriber obs_sub;
    
    int obstacle_size;

    // interactive markers' server
    interactive_markers::InteractiveMarkerServer im_server;

    // The car state and parameters
    twoWheelBotState jetbotState;
    double previous_seconds;
    double previousTime;  // required to udpate keyboard input timestep.NASTY, DO IT BETTER
    double max_wheel_speed;
    double max_wheel_torque;
    //double desired_speed, desired_curvature;
    double accel;
    double angularVelocityThreshold;
    double rightWheelTorqueCommand;
    double leftWheelTorqueCommand;
    double rightWheelSpeed = 0;
    double leftWheelSpeed = 0;
    double rightWheelSpeedReference = 0.0;
    double leftWheelSpeedReference = 0.0;
    double max_car_speed;
    double max_steer_angle;
    double motorTimeConstant;
    double rotationWheelSpeedScale;
    std::string keyboardCommand;
    twoWheelBotParameters jetbotParameters;
    lowPassFilter rightWheelFilter;
    lowPassFilter leftWheelFilter;
    // For publishing transformations
    tf2_ros::TransformBroadcaster br;

    // A timer to update the pose
    ros::Timer update_pose_timer;

    // Listen for drive commands
    ros::Subscriber drive_sub;
    //ros::Subscriber key_sub;

    // Listen for a map
    ros::Subscriber map_sub;
    bool map_exists = false;

    // Listen for updates to the pose
    ros::Subscriber pose_sub;
    ros::Subscriber pose_rviz_sub;

    // Publish a scan, odometry, and imu data
    bool broadcast_transform;
    bool pub_gt_pose;
    ros::Publisher pose_pub;
    ros::Publisher odom_pub;

    // publisher for map with obstacles
    ros::Publisher map_pub;

    // for obstacle collision
    int map_width, map_height;
    double map_resolution, origin_x, origin_y;

    // safety margin for collisions
    double thresh;
    double speed_clip_diff;

    // pi
    const double PI = 3.1415;

    // for collision check
    bool TTC = false;
    double ttc_threshold;

public:

    jetbotSimulator(): im_server("jetbot_sim"), rightWheelFilter(motorTimeConstant, rightWheelSpeed), leftWheelFilter(motorTimeConstant, leftWheelSpeed){
        n = ros::NodeHandle("~");

        jetbotState = {.x=0.0, .y=0.0, .theta=0.0, .velocity=0.0, .angular_velocity=0.0, .leftWheelSpeed=0.0, .rightWheelSpeed=0.0, .std_dyn=false};

        accel = 0.0;
        //desired_speed = 0.0;
        //desired_curvature = 0.0;
        previous_seconds = ros::Time::now().toSec();
        previousTime = ros::Time::now().toSec();


        // Get the topic names
        std::string drive_topic, map_topic, pose_topic, gt_pose_topic, 
        pose_rviz_topic, odom_topic, diff_drive_topic, keyboard_topic;
        n.getParam("drive_topic", drive_topic);
        n.getParam("map_topic", map_topic);    
        n.getParam("pose_topic", pose_topic);
        n.getParam("odom_topic", odom_topic);
        n.getParam("pose_rviz_topic", pose_rviz_topic);
        n.getParam("ground_truth_pose_topic", gt_pose_topic);
        n.getParam("diff_drive_topic", diff_drive_topic);
        n.getParam("keyboard_topic", keyboard_topic);

        // Get the transformation frame names
        n.getParam("map_frame", map_frame);
        n.getParam("base_frame", base_frame);

        //Fetch car parameters
        double update_pose_rate;
        n.getParam("update_pose_rate", update_pose_rate);
        n.getParam("jetbot_max_wheel_speed", max_wheel_speed);
        n.getParam("jetbot_max_wheel_torque", max_wheel_torque);
        n.getParam("jetbot_max_speed", max_car_speed);
        n.getParam("jetbot_max_steer", max_steer_angle);
        n.getParam("jetbot_mass", jetbotParameters.mass);
        n.getParam("jetbot_wheel_radius", jetbotParameters.wheelRadius);
        n.getParam("jetbot_length" , jetbotParameters.length);
        n.getParam("jetbot_width", jetbotParameters.track);
        n.getParam("jetbot_wheel_damping" , jetbotParameters.wheelDampingFactor);
        n.getParam("jetbot_motor_timeConstant" , motorTimeConstant);
        n.getParam("jetbot_rotation_wheel_speed_scale", rotationWheelSpeedScale);
        jetbotParameters.I_z=(1.0/12.0)*(jetbotParameters.mass)*
            (pow(jetbotParameters.track,2.0) + pow(jetbotParameters.length,2.0));   

        // Determine if we should broadcast
        n.getParam("broadcast_transform", broadcast_transform);
        n.getParam("publish_ground_truth_pose", pub_gt_pose);

        // Get obstacle size parameter
        n.getParam("obstacle_size", obstacle_size);

        // Make a publisher for odometry messages
        odom_pub = n.advertise<nav_msgs::Odometry>(odom_topic, 1);

       
        // Make a publisher for ground truth pose
        pose_pub = n.advertise<geometry_msgs::PoseStamped>(gt_pose_topic, 1);

        // Start a timer to output the pose
        update_pose_timer = n.createTimer(ros::Duration(update_pose_rate),\
         &jetbotSimulator::update_pose, this);

        // Start a subscriber to listen to drive commands
        drive_sub = n.subscribe(diff_drive_topic, 1, &jetbotSimulator::drive_callback, this);
        //key_sub = n.subscribe(keyboard_topic, 1, &jetbotSimulator::key_callback, this);

      
        // Start a subscriber to listen to pose messages
        pose_sub = n.subscribe(pose_topic, 1, &jetbotSimulator::pose_callback, this);
        pose_rviz_sub = n.subscribe(pose_rviz_topic, 1, &jetbotSimulator::pose_rviz_callback, this);

       
        // get collision safety margin
        n.getParam("coll_threshold", thresh);
        n.getParam("ttc_threshold", ttc_threshold);
        n.getParam("angular_velocity_threshold" , angularVelocityThreshold);

        ROS_INFO("Simulator constructed.");
    
    }

    void update_pose(const ros::TimerEvent&){

        // Update the pose 
        ros::Time timestamp = ros::Time::now();
        double current_seconds = timestamp.toSec();
        double dt = current_seconds - previous_seconds;
        rightWheelSpeed = rightWheelFilter.update(dt, rightWheelSpeedReference);
        leftWheelSpeed = leftWheelFilter.update(dt, leftWheelSpeedReference);
        jetbotState = jetbotKinematics::kinematicUpdate(
            jetbotState,
            rightWheelSpeed,
            leftWheelSpeed,
            jetbotParameters,
            dt);
            
        previous_seconds = current_seconds;

        /// Publish the pose as a transformation
        pub_pose_transform(timestamp);

        /// Make an odom message as well and publish it
        pub_odom(timestamp);
        //ROS_INFO("rightWheelSpeed = %f " , jetbotState.rightWheelSpeed);
    
    }
    ///---------PUBLISHIG HELPER FUNCTIONS-------
    void pub_pose_transform(ros::Time timestamp) {
            // Convert the pose into a transformation
            geometry_msgs::Transform t;
            t.translation.x = jetbotState.x;
            t.translation.y = jetbotState.y;
            tf2::Quaternion quat;
            quat.setEuler(0., 0., jetbotState.theta);
            t.rotation.x = quat.x();
            t.rotation.y = quat.y();
            t.rotation.z = quat.z();
            t.rotation.w = quat.w();

            // publish ground truth pose
            geometry_msgs::PoseStamped ps;
            ps.header.frame_id = "/map";
            ps.pose.position.x = jetbotState.x;
            ps.pose.position.y = jetbotState.y;
            ps.pose.orientation.x = quat.x();
            ps.pose.orientation.y = quat.y();
            ps.pose.orientation.z = quat.z();
            ps.pose.orientation.w = quat.w();

            // Add a header to the transformation
            geometry_msgs::TransformStamped ts;
            ts.transform = t;
            ts.header.stamp = timestamp;
            ts.header.frame_id = map_frame;
            ts.child_frame_id = base_frame;

            // Publish them
            if (broadcast_transform) {
                br.sendTransform(ts);
            }
            if (pub_gt_pose) {
                pose_pub.publish(ps);
            }
    }

    void pub_odom(ros::Time timestamp) {
            // Make an odom message and publish it
            nav_msgs::Odometry odom;
            odom.header.stamp = timestamp;
            odom.header.frame_id = map_frame;
            odom.child_frame_id = base_frame;
            odom.pose.pose.position.x = jetbotState.x;
            odom.pose.pose.position.y = jetbotState.y;
            tf2::Quaternion quat;
            quat.setEuler(0., 0., jetbotState.theta);
            odom.pose.pose.orientation.x = quat.x();
            odom.pose.pose.orientation.y = quat.y();
            odom.pose.pose.orientation.z = quat.z();
            odom.pose.pose.orientation.w = quat.w();
            odom.twist.twist.linear.x = jetbotState.velocity;
            odom.twist.twist.angular.z = jetbotState.angular_velocity;
            odom_pub.publish(odom);
    }
        //--Callback Functions-----//
    //void drive_callback(const std_msgs::Float64MultiArray &msg){
    void drive_callback(const jetbotcar::Jetdrivemsg &msg){ 
            //rightWheelSpeedReference = msg.data[0]*max_wheel_speed;
            //leftWheelSpeedReference = msg.data[1]*max_wheel_speed;
            rightWheelSpeedReference = msg.right*max_wheel_speed;
            leftWheelSpeedReference = msg.left*max_wheel_speed;
            rightWheelSpeedReference = std::max(std::min(rightWheelSpeedReference,
                                    max_wheel_speed), -max_wheel_speed); 
            leftWheelSpeedReference = std::max(std::min(leftWheelSpeedReference,
                                    max_wheel_speed), -max_wheel_speed);    
    }
    

    
    void pose_callback(const geometry_msgs::PoseStamped & msg) {
        jetbotState.x = msg.pose.position.x;
        jetbotState.y = msg.pose.position.y;
        geometry_msgs::Quaternion q = msg.pose.orientation;
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        jetbotState.theta = tf2::impl::getYaw(quat);
    }
    void pose_rviz_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msg) {
        geometry_msgs::PoseStamped temp_pose;
        temp_pose.header = msg->header;
        temp_pose.pose = msg->pose.pose;
        pose_callback(temp_pose);
    }
       


};
int main(int argc, char ** argv){
    ros::init(argc, argv, "jetbotSimulator");
    jetbotSimulator jetSim;
    ros::spin();
    return 0;

}