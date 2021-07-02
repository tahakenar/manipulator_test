#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float64MultiArray.h>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include "joy_functions.h"
#include "moveit_functions.h"

// TODO: Forward kinematics mode / control using joint space
// TODO: Update according to the last encoder data -> EZ, partly ready
// TODO: Reading data from encoder database
// TODO: Quaternion conversion check
// TODO: Signs of directions for each joint will be determined
// TODO: Joint space constraints will be determined -> can be solved from joint_limits.yaml file
// TODO: Serial mode numbers will be defined

static const std::string PLANNING_GROUP = "manipulator";

std::vector<float> deltaXYZ{0.0,0.0,0.0};
std::vector<float> deltaRPY{0.0,0.0,0.0};
std::vector<double> actuatorVelocities{0.0,0.0,0.0,0.0,0.0,0.0,0.0};
// global variables to control end effector position and orientation

std_msgs::Float64MultiArray array_of_angles;
std_msgs::Float64MultiArray velocity_of_motors;
// global arrays to publish other nodes

std::vector<double> encoder{0.0,0.0,0.0,0.0,0.0,0.0};
bool first_run = true;
bool update_joints = false;
// global variable to store last encoder data

int mode = 0; 
int switch_btn_prev = 0;
float frac = 0.002;
// global variables to achieve a mode switch algorithm, they are being used in joyCallback function

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    int switch_btn_current = joy->buttons[7];
    if (switch_btn_current == 1 && switch_btn_prev == 0){
        mode += 1;
        if (mode == 4)
            mode = 0;
        if (mode == 0){
            frac = 0.002;
            ROS_INFO("Control mode: SLOW \n");
            update_joints = true;
        }
        else if (mode == 1){
            frac = 0.01;
            ROS_INFO("Control mode: MID \n");
        }
        else if (mode == 2){
            frac = 0.04;
            ROS_INFO("Control mode: FAST /tikkatli olun\n");
        } 
        else if (mode == 3){
            ROS_INFO("Control mode: FORWARD KINEMATICS\n");
            // Forward kinematics mode will be added
        } 
    }
    switch_btn_prev = switch_btn_current;

    if (mode == 0 || mode == 1 || mode == 2){
        float x = joy->axes[1];
        float y = joy->axes[0];
        float z = joy->axes[7];
        float roll = joy->axes[6];
        float pitch = joy->axes[4];
        float yaw = joy->axes[3];

        deltaXYZ = JoyFuncs::xyzControl(x,y,z,frac);
        deltaRPY = JoyFuncs::rpyControl(roll,pitch,yaw,frac);
        // cartesian point position and euler angles orientation control
    } 

    if (mode == 3){
        actuatorVelocities = JoyFuncs::forwardKinematicsFunc(joy);
        // Forward kinematics mode
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_controller");
    ros::NodeHandle nh;
    ros::Rate loop_rate(150);
    ros::Subscriber joy_sub = nh.subscribe("joy",1,joyCallback);
    ros::Publisher joint_values_pub = nh.advertise<std_msgs::Float64MultiArray>("joint_states/send", 1000);
    // ROS Procedure

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    //bool success;

    ros::AsyncSpinner spinner(2); 
    spinner.start();

    
    ROS_INFO("Control mode: SLOW \n");
    // for first run

    while (ros::ok())
    {

    ros::spinOnce();

    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose current_pose;
    current_pose = move_group.getCurrentPose().pose;
    waypoints.push_back(current_pose);
    // MoveIt procedure

////////////////////////////// UPDATING /////////////////////////
// Here, the goal is to update joint states according to the encoder data.
// this part should be executed only on first run or after mode is changed from Forward Kine. to something else

    if (first_run || update_joints){
        if (first_run) {
            encoder = JoyFuncs::updateFromFile();
            first_run = false;
        }
        else if (update_joints){
            encoder = JoyFuncs::updateFromNode();
            update_joints = false;
        }
        CustomMoveItFuncs::updateJointStates(move_group,my_plan,encoder);
    }
////////////////////////////////////////////////////////////////

    if (mode != 3){

        ROS_INFO("%f %f %f ----\n %f %f %f",deltaXYZ[0],deltaXYZ[1],deltaXYZ[2],deltaRPY[0],deltaRPY[1],deltaRPY[2]);
    
        geometry_msgs::Pose target_pose = current_pose;

        if(deltaXYZ[0] != 0 || deltaXYZ[1] != 0 || deltaXYZ[2] != 0) 
        {
            CustomMoveItFuncs::ExecuteCartesianPath(deltaXYZ,move_group,waypoints,target_pose);
        }

        else if (deltaRPY[0] != 0 || deltaRPY[1] != 0 || deltaRPY[2] != 0)
        {
            CustomMoveItFuncs::ExecuteRPYGoal(deltaRPY,move_group,target_pose);
        }

        
        array_of_angles = CustomMoveItFuncs::returnJointAngles(move_group,9);
        // the second argument is arbitrarily given as serial mode, will be changed afterwards

        joint_values_pub.publish(array_of_angles);
        // Publishing joint states to the serial message constructor

    }

    else if (mode == 3){

        velocity_of_motors = JoyFuncs::returnActuatorVel(actuatorVelocities);
        joint_values_pub.publish(velocity_of_motors);

    }

    loop_rate.sleep();
    update_joints = false;
    
    }  
    return 0;
}