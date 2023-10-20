#pragma once // Included once

#define CVUI_IMPLEMENTATION // cvui
#include "robot_gui/cvui.h" // Include the cvui library

// ROS message and service includes
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include "std_srvs/Trigger.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

class RobotGUI {
public:
  RobotGUI();  // Constructor
  ~RobotGUI(); // Destructor

  // Callback functions for subscribers
  void
  robotInfoCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg);
  void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg);
  void odomCallback(const nav_msgs::Odometry::ConstPtr &msg);

  // Main loop for the graphical user interface (GUI)
  void run();

private:
  ros::NodeHandle nh; // NodeHandle for ROS

  // ROS subscribers
  ros::Subscriber robotInfoSub;
  ros::Subscriber odomSub;
  ros::Subscriber cmdVelSub;

  // ROS publisher
  ros::Publisher cmdVelPublisher;

  robotinfo_msgs::RobotInfo10Fields robotinfo_msgs; // Robot information message

  geometry_msgs::Twist
      cmdVel; // Twist message for controlling the robot's motion

  cv::Mat frame; // OpenCV matrix for displaying the GUI

  std::string robotInfo; // Robot information string

  // ROS service client for calling a distance service
  ros::ServiceClient distanceServiceClient;

  // Response received from the distance service
  std::string distanceServiceResponse;

  double cmdVelLinear;  // Linear velocity of the robot
  double cmdVelAngular; // Angular velocity of the robot
  double xPos;          // X position of the robot based on odometry
  double yPos;          // Y position of the robot based on odometry
  double zPos;          // Z position of the robot based on odometry
};
