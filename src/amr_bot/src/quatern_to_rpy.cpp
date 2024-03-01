#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <rclcpp/rclcpp.hpp>

class ConversionNode : public rclcpp::Node {
public:
  ConversionNode() : Node("conversion_node") {
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 1, std::bind(&ConversionNode::odometryCallback, this, std::placeholders::_1));
    publisher_ = this->create_publisher<geometry_msgs::msg::Pose2D>("pose2d", 1);
  }

private:
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    geometry_msgs::msg::Pose2D pose2d;
    pose2d.x = msg->pose.pose.position.x;
    pose2d.y = msg->pose.pose.position.y;

    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    pose2d.theta = yaw;
    publisher_->publish(pose2d);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr publisher_;
};

int
 
main(int argc, char **argv)
 
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ConversionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}