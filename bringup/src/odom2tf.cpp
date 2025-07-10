#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

class OdomTopic2TF : public rclcpp::Node {
public:
  OdomTopic2TF(std::string name) : Node(name) {
    // 声明并获取参数
    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<std::string>("base_frame", "base_footprint");
    
    odom_frame_ = get_parameter("odom_frame").as_string();
    base_frame_ = get_parameter("base_frame").as_string();
    
    // 参数验证
    if (base_frame_ != "base_footprint") {
      RCLCPP_FATAL(get_logger(), "base_frame must be 'base_footprint', got '%s'", base_frame_.c_str());
      exit(EXIT_FAILURE);
    }
    
    // 创建里程计订阅者，使用SensorDataQoS
    odom_subscribe_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(),
      std::bind(&OdomTopic2TF::odom_callback_, this, std::placeholders::_1));
    
    // 创建TF广播器
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    
    RCLCPP_INFO(get_logger(), "OdomTopic2TF started: %s -> %s", 
                odom_frame_.c_str(), base_frame_.c_str());
  }

private:
  void odom_callback_(const nav_msgs::msg::Odometry::SharedPtr msg) {
    try {
      geometry_msgs::msg::TransformStamped transform;
      
      // 使用里程计原始时间戳
      transform.header.stamp = msg->header.stamp;
      transform.header.frame_id = odom_frame_;
      transform.child_frame_id = base_frame_;
      
      // 设置变换值
      transform.transform.translation.x = msg->pose.pose.position.x;
      transform.transform.translation.y = msg->pose.pose.position.y;
      transform.transform.translation.z = msg->pose.pose.position.z;
      transform.transform.rotation = msg->pose.pose.orientation;
      
      // 广播坐标变换
      tf_broadcaster_->sendTransform(transform);
      
      // 修正时间戳输出（使用sec和nanosec）
      double time_seconds = transform.header.stamp.sec + 
                           transform.header.stamp.nanosec * 1e-9;
      RCLCPP_DEBUG(get_logger(), "Published TF at time %f", time_seconds);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to publish TF: %s", e.what());
    }
  }
  
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscribe_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  std::string odom_frame_;
  std::string base_frame_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomTopic2TF>("odom2tf");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}