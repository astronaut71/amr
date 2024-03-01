#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Matrix3x3.h>

using std::placeholders::_1;
using namespace std;


class CmdVelToOdomNode : public rclcpp::Node 
{
public:
  CmdVelToOdomNode() 
  : Node("tf_broadcaster") 
  {
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&CmdVelToOdomNode::cmdVelCallback, this, _1));
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::SensorDataQoS());
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    
  }

private:

    double x = 0.0;  // Initial position in x
    double y = 0.0;  // Initial position in y
    double theta = 0.0;  // Initial orientation

void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    double linear_x = msg->linear.x;
    double angular_z = msg->angular.z;

    // Calculate pose updates using simple differential drive kinematics
    double dt = 0.1;  // Assuming a fixed time step of 0.1 seconds
    double dx = linear_x * dt;
    double dy = 0.0;  // Assuming no lateral motion
    double dtheta = angular_z * dt;

    x += dx * cos(theta) - dy * sin(theta);
    y += dx * sin(theta) + dy * cos(theta);
    theta += dtheta;

    // Create and publish odometry message
    nav_msgs::msg::Odometry odom_msg;
    //now = rclcpp::Node::now();

    odom_msg.header.stamp = now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.orientation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), theta));

    //New Create and publish odometry message with RPY orientation
    tf2::Quaternion q;


    odom_msg.twist.twist.linear.x = linear_x;
    odom_msg.twist.twist.angular.z = angular_z;

    odom_pub_->publish(odom_msg);

    // Broadcast TF transform
    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.header.stamp = now();
    transform_stamped.header.frame_id = "odom";
    transform_stamped.child_frame_id = "base_link";

    transform_stamped.transform.translation.x = x;
    transform_stamped.transform.translation.y = y;
    transform_stamped.transform.translation.z = 0.0;
    transform_stamped.transform.rotation = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0, 0, 1), theta));

    tf_broadcaster_->sendTransform(transform_stamped);  // Access member using ->

  }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  };

int main(int argc, char **argv) {
  
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelToOdomNode>());
    rclcpp::shutdown();
  return 0;
}