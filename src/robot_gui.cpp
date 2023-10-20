#include "robot_gui/robot_gui.h"

// Constructor
RobotGUI::RobotGUI() : nh(), frame(400, 600, CV_8UC3), cmdVel() {
  // ROS subscribers, publishers, and service clients initialization
  robotInfoSub =
      nh.subscribe("robot_info", 10, &RobotGUI::robotInfoCallback, this);
  odomSub = nh.subscribe("odom", 10, &RobotGUI::odomCallback, this);
  cmdVelPublisher = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  distanceServiceClient = nh.serviceClient<std_srvs::Trigger>("get_distance");
  // Initialize the cvui library
  cvui::init("Robot GUI");
}

// Destructor
RobotGUI::~RobotGUI() {}

void RobotGUI::robotInfoCallback(
    const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg) {
  robotinfo_msgs.data_field_01 = msg->data_field_01;
  robotinfo_msgs.data_field_02 = msg->data_field_02;
  robotinfo_msgs.data_field_03 = msg->data_field_03;
  robotinfo_msgs.data_field_04 = msg->data_field_04;
  robotinfo_msgs.data_field_05 = msg->data_field_05;
  robotinfo_msgs.data_field_06 = msg->data_field_06;
  robotinfo_msgs.data_field_07 = msg->data_field_07;
}

void RobotGUI::odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  cmdVelLinear = msg->twist.twist.linear.x;
  cmdVelAngular = msg->twist.twist.angular.z;
  xPos = msg->pose.pose.position.x;
  yPos = msg->pose.pose.position.y;
  zPos = msg->pose.pose.position.z;
}

void RobotGUI::run() {
  cv::namedWindow("Robot GUI", cv::WINDOW_AUTOSIZE);

  while (ros::ok()) {
    frame = cv::Scalar(49, 52, 49);

    // General Info Window
    cvui::window(frame, 10, 10, 260, 230, "Info");

    cvui::text(frame, 15, 35, "Robot Information:");
    cvui::text(frame, 15, 55, robotinfo_msgs.data_field_01);
    cvui::text(frame, 15, 75, robotinfo_msgs.data_field_02);
    cvui::text(frame, 15, 95, robotinfo_msgs.data_field_03);
    cvui::text(frame, 15, 115, robotinfo_msgs.data_field_04);
    cvui::text(frame, 15, 135, robotinfo_msgs.data_field_05);
    cvui::text(frame, 15, 155, robotinfo_msgs.data_field_06);
    cvui::text(frame, 15, 175, robotinfo_msgs.data_field_07);

    std::istringstream stream(robotInfo);
    std::string line;
    int yOffset = 35;
    while (std::getline(stream, line)) {
      cvui::text(frame, 20, yOffset, line);
      yOffset += 20;
    }

    // Teleoperation Buttons
    cvui::text(frame, 10, 260, "Teleoperation Buttons:");
    if (cvui::button(frame, 90, 285, " FORWARD ")) {
      cmdVel.linear.x += 0.1;
    }
    if (cvui::button(frame, 105, 285 + 30, " STOP")) {
      cmdVel.linear.x = 0;
      cmdVel.angular.z = 0;
    }
    if (cvui::button(frame, 20, 285 + 30, " LEFT ")) {
      cmdVel.angular.z += 0.1;
    }
    if (cvui::button(frame, 190, 285 + 30, " RIGHT ")) {
      cmdVel.angular.z -= 0.1;
    }
    if (cvui::button(frame, 90, 285 + 60, "BACKWARDS")) {
      cmdVel.linear.x -= 0.1;
    }

    // Current Velocity
    cvui::text(frame, 310, 10, "Velocities:");

    cmdVelLinear = cmdVel.linear.x;
    std::ostringstream linV;
    linV << std::fixed << std::setprecision(2) << cmdVelLinear;
    cvui::window(frame, 310, 30, 105, 100, "Linear Velocity");
    cvui::text(frame, 315, 60, linV.str(), 1, 0xFF0000);
    cvui::text(frame, 310, 90, " m/s", 0.7, 0xFF0000);

    cmdVelAngular = cmdVel.angular.z;
    std::ostringstream angV;
    angV << std::fixed << std::setprecision(2) << cmdVelAngular;
    cvui::window(frame, 450, 30, 115, 100, "Angular Velocity");
    cvui::text(frame, 455, 60, angV.str(), 1, 0xFF0000);
    cvui::text(frame, 450, 90, " rad/s", 0.7, 0xFF0000);

    // Robot Position (Odometry based)
    cvui::text(frame, 310, 175, "Estimated Robot Position using odometry");
    cvui::window(frame, 290, 195, 80, 80, "X");
    cvui::printf(frame, 310, 240, "%.2f", xPos);

    cvui::window(frame, 395, 195, 80, 80, "Y");
    cvui::printf(frame, 415, 240, "%.2f", yPos);

    cvui::window(frame, 500, 195, 80, 80, "Z");
    cvui::printf(frame, 520, 240, "%.2f", zPos);

    // Distance traveled service
    cvui::text(frame, 310, 300, "Distance travelled");

    cvui::window(frame, 310, 320, 180, 60, "Distance in meters");
    cvui::printf(frame, 380, 355, "%s m", distanceServiceResponse.c_str());

    if (cvui::button(frame, 500, 320, "Call")) {
      std_srvs::Trigger srv;
      if (distanceServiceClient.call(srv)) {
        distanceServiceResponse = srv.response.message;
      } else {
        distanceServiceResponse = "Service call failed.";
      }
    }

    // Publish the twist message
    cmdVelPublisher.publish(cmdVel);

    // Update cvui and display the frame
    cvui::update();
    cv::imshow("Robot GUI", frame);

    // Check for the ESC key
    if (cv::waitKey(20) == 27) {
      break;
    }

    ros::spinOnce();
  }
}
