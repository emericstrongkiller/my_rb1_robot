#include "geometry_msgs/Twist.h"
#include "my_rb1_ros/Rotate.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "tf/tf.h"
#include <cmath>

// robot's orientation (global variable)
double current_yaw = 0;

// global NodeHandle
ros::NodeHandle *nh;

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg) {
  // get orientation quaternion from odom topic
  tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                   msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch;
  m.getRPY(roll, pitch, current_yaw);

  // if debug needed (there was)
  // ROS_INFO("current yaw %f", current_yaw);
}

bool my_callback(my_rb1_ros::Rotate::Request &req,
                 my_rb1_ros::Rotate::Response &res) {

  // calculate robot angle target (use current robot orientation as rotation
  // origin)
  float target_angle = (req.degrees * 3.14159265359 / 180) + current_yaw;

  if (target_angle > 3.14159265359) {
    target_angle = -3.14159265359 + (target_angle - 3.14159265359);
  } else if (target_angle < -3.14159265359) {
    target_angle = 3.14159265359 - (-3.14159265359 - target_angle);
  } else {
  }

  ROS_INFO("current yaw : %f", current_yaw);
  ROS_INFO("target angle : %f", target_angle);

  // set rotation direction
  int rotation_direction;
  if (req.degrees < 0) {
    rotation_direction = -1;
  } else {
    rotation_direction = 1;
  }

  // create publisher to rotate rb1
  ros::Publisher cmd_pub = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 100);
  geometry_msgs::Twist cmd_val;

  // loop until robot got to desired angle rotation done
  ros::Rate rate(10);
  while (std::abs(current_yaw - target_angle) > 0.05) {
    // apply angular velocity to rb1 depending on the distance to target angle
    // rotation
    cmd_val.angular.z = rotation_direction * 0.1;
    cmd_pub.publish(cmd_val);

    // spin once to get callback data and refresh the current yaw !!
    ros::spinOnce();
    rate.sleep();

    res.result = "success";
  }

  cmd_val.angular.z = 0;
  cmd_pub.publish(cmd_val);
  ROS_INFO("Rotation state : %s", res.result.c_str());
  rate.sleep();

  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "rb1_rotate_server");
  nh = new ros::NodeHandle;

  ros::Subscriber odom_sub = nh->subscribe("/odom", 100, odom_callback);

  ros::ServiceServer rotate_robot =
      nh->advertiseService("/rotate_robot", my_callback);
  ros::spin();

  return 0;
}