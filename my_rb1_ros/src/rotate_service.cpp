#include "geometry_msgs/Twist.h"
#include "my_rb1_ros/Rotate.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "ros/time.h"
#include "tf/tf.h"
#include <cmath>
#include <string>

// robot's orientation (global variable)
double current_yaw = 0;

// global NodeHandle
ros::NodeHandle *nh;

// define pi
const double pi = 3.141592653589793;

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

  ROS_INFO("Service Requested\nRequired rotation : %d degrees", req.degrees);

  // calculate robot angle target (use current robot orientation as rotation
  // origin)
  float target_angle = (req.degrees * pi / 180) + current_yaw;

  while (target_angle > pi) {
    target_angle -= 2 * pi;
  }
  while (target_angle < -pi) {
    target_angle += 2 * pi;
  }

  // set rotation direction
  int rotation_direction;
  std::string rotation_direction_s;
  if (req.degrees < 0) {
    rotation_direction = -1;
    rotation_direction_s = "clockwise";
  } else {
    rotation_direction = 1;
    rotation_direction_s = "counter_clockwise";
  }

  // create publisher to rotate rb1
  ros::Publisher cmd_pub = nh->advertise<geometry_msgs::Twist>("/cmd_vel", 100);
  geometry_msgs::Twist cmd_val;

  // Setup timout to monitor rotation
  double rotation_timeout = 18.0;
  ros::Time start_time = ros::Time::now();

  // if angle already attained, return success
  if (std::abs(target_angle - current_yaw) <= 0.04) {
    res.result = "Service successful : /rotate_robot " +
                 std::to_string(req.degrees) +
                 " degrees (already_at_required_angle)";
  }

  // loop until robot got to desired angle rotation done
  ros::Rate rate(10);
  while (std::abs(current_yaw - target_angle) > 0.04) {

    if ((ros::Time::now() - start_time).toSec() > rotation_timeout) {
      ROS_ERROR(
          "Rotation failed: Timeout exceeded. Unable to complete rotation.");
      res.result = "failure: timeout";
      break;
    }

    // apply angular velocity to rb1 depending on the distance to target
    // angle rotation
    cmd_val.angular.z = rotation_direction * 0.5;
    cmd_pub.publish(cmd_val);

    // values for debug purposes
    /*
    ROS_INFO("current angle : %f", current_yaw);
    ROS_INFO("target angle : %f", target_angle);
    */

    // spin once to get callback data and refresh the current yaw !!
    ros::spinOnce();
    rate.sleep();

    res.result = "Service successful : /rotate_robot " +
                 std::to_string(req.degrees) + " degrees";
  }

  cmd_val.angular.z = 0;
  cmd_pub.publish(cmd_val);
  rate.sleep();

  ROS_INFO("Rotation Completed\n");

  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "rb1_rotate_server");
  nh = new ros::NodeHandle;

  ros::Subscriber odom_sub = nh->subscribe("/odom", 100, odom_callback);

  ROS_INFO("Service ready\n");

  ros::ServiceServer rotate_robot =
      nh->advertiseService("/rotate_robot", my_callback);
  ros::spin();

  return 0;
}