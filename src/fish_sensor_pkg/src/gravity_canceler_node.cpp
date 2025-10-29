#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <Eigen/Dense>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class GravityCancelerNode : public rclcpp::Node
{
public:
  GravityCancelerNode() : Node("gravity_canceler_node")
  {
    sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/fish/imu/data_with_orientation", 10,
      std::bind(&GravityCancelerNode::imuCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/fish/ctrl/imu", 10);

    RCLCPP_INFO(this->get_logger(), "GravityCancelerNode started.");
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    tf2::Quaternion q;
    tf2::fromMsg(msg->orientation, q);

    tf2::Matrix3x3 R(q);
    Eigen::Matrix3d rot;
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        rot(i, j) = R[i][j];

    Eigen::Vector3d g(0.0, 0.0, 9.81);
    Eigen::Vector3d acc(msg->linear_acceleration.x,
                        msg->linear_acceleration.y,
                        msg->linear_acceleration.z);

    Eigen::Vector3d acc_corrected = acc - rot.transpose() * g;

    sensor_msgs::msg::Imu out = *msg;
    out.linear_acceleration.x = acc_corrected.x();
    out.linear_acceleration.y = acc_corrected.y();
    out.linear_acceleration.z = acc_corrected.z();

    pub_->publish(out);
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
};
 
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GravityCancelerNode>());
  rclcpp::shutdown();
  return 0;
}
