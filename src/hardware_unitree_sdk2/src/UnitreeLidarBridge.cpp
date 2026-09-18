#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <unitree/idl/ros2/Imu_.hpp>
#include <unitree/idl/ros2/PointCloud2_.hpp>
#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

class UnitreeLidarBridge final : public rclcpp::Node
{
public:
  UnitreeLidarBridge() : Node("unitree_lidar_bridge")
  {
    const auto interface = declare_parameter<std::string>("network_interface", "");
    const auto cloud_dds_topic =
      declare_parameter<std::string>("cloud_dds_topic", "rt/utlidar/cloud");
    const auto imu_dds_topic =
      declare_parameter<std::string>("imu_dds_topic", "rt/utlidar/imu");
    cloud_frame_ = declare_parameter<std::string>("cloud_frame", "utlidar_lidar");
    imu_frame_ = declare_parameter<std::string>("imu_frame", "utlidar_imu");
    use_source_stamp_ = declare_parameter<bool>("use_source_stamp", false);
    preserve_source_timing_ = declare_parameter<bool>("preserve_source_timing", true);
    if (interface.empty() || interface == "lo") {
      throw std::runtime_error("network_interface must name the Ethernet adapter connected to GO2");
    }

    cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/unitree/lidar", rclcpp::SensorDataQoS());
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(
      "/unitree/lidar_imu", rclcpp::SensorDataQoS());

    unitree::robot::ChannelFactory::Instance()->Init(0, interface);
    cloud_subscriber_ = std::make_shared<
      unitree::robot::ChannelSubscriber<sensor_msgs::msg::dds_::PointCloud2_>>(
      cloud_dds_topic);
    cloud_subscriber_->InitChannel(
      [this](const void * message) { publish_cloud(message); }, 1);
    imu_subscriber_ = std::make_shared<
      unitree::robot::ChannelSubscriber<sensor_msgs::msg::dds_::Imu_>>(imu_dds_topic);
    imu_subscriber_->InitChannel(
      [this](const void * message) { publish_imu(message); }, 1);

    started_ = std::chrono::steady_clock::now();
    diagnostic_timer_ = create_wall_timer(std::chrono::seconds(5), [this]() {
      if (cloud_count_ == 0) {
        RCLCPP_WARN(
          get_logger(),
          "No GO2 lidar cloud received; check Ethernet, GO2 EDU lidar service, and DDS topic");
      } else {
        RCLCPP_INFO(
          get_logger(), "GO2 lidar bridge healthy: %zu clouds, %zu lidar IMU packets",
          cloud_count_, imu_count_);
      }
    });
    RCLCPP_INFO(
      get_logger(), "Unitree lidar DDS bridge: %s -> /unitree/lidar", cloud_dds_topic.c_str());
  }

private:
  builtin_interfaces::msg::Time converted_stamp(
    const builtin_interfaces::msg::dds_::Time_ & input)
  {
    const int64_t source_ns =
      static_cast<int64_t>(input.sec()) * 1000000000LL + input.nanosec();
    if (use_source_stamp_ && source_ns > 0) {
      builtin_interfaces::msg::Time output;
      output.sec = input.sec();
      output.nanosec = input.nanosec();
      return output;
    }

    const auto receipt = now();
    if (!preserve_source_timing_ || source_ns <= 0) {
      return receipt;
    }

    // Apply one common epoch offset to both L1 streams. This keeps their
    // device-clock intervals and relative timing while making stamps current
    // in the host ROS clock for watchdogs and downstream navigation.
    std::lock_guard<std::mutex> lock(stamp_mutex_);
    if (!source_to_host_offset_ns_) {
      source_to_host_offset_ns_ = receipt.nanoseconds() - source_ns;
    }
    return rclcpp::Time(source_ns + *source_to_host_offset_ns_);
  }

  void publish_cloud(const void * raw)
  {
    if (raw == nullptr) return;
    const auto & input = *static_cast<const sensor_msgs::msg::dds_::PointCloud2_ *>(raw);
    sensor_msgs::msg::PointCloud2 output;
    output.header.stamp = converted_stamp(input.header().stamp());
    output.header.frame_id = cloud_frame_.empty() ? input.header().frame_id() : cloud_frame_;
    output.height = input.height();
    output.width = input.width();
    output.fields.reserve(input.fields().size());
    for (const auto & field : input.fields()) {
      sensor_msgs::msg::PointField converted;
      converted.name = field.name();
      converted.offset = field.offset();
      converted.datatype = field.datatype();
      converted.count = field.count();
      output.fields.push_back(std::move(converted));
    }
    output.is_bigendian = input.is_bigendian();
    output.point_step = input.point_step();
    output.row_step = input.row_step();
    output.data = input.data();
    output.is_dense = input.is_dense();
    cloud_publisher_->publish(std::move(output));
    ++cloud_count_;
  }

  void publish_imu(const void * raw)
  {
    if (raw == nullptr) return;
    const auto & input = *static_cast<const sensor_msgs::msg::dds_::Imu_ *>(raw);
    sensor_msgs::msg::Imu output;
    output.header.stamp = converted_stamp(input.header().stamp());
    output.header.frame_id = imu_frame_.empty() ? input.header().frame_id() : imu_frame_;
    output.orientation.x = input.orientation().x();
    output.orientation.y = input.orientation().y();
    output.orientation.z = input.orientation().z();
    output.orientation.w = input.orientation().w();
    output.orientation_covariance = input.orientation_covariance();
    output.angular_velocity.x = input.angular_velocity().x();
    output.angular_velocity.y = input.angular_velocity().y();
    output.angular_velocity.z = input.angular_velocity().z();
    output.angular_velocity_covariance = input.angular_velocity_covariance();
    output.linear_acceleration.x = input.linear_acceleration().x();
    output.linear_acceleration.y = input.linear_acceleration().y();
    output.linear_acceleration.z = input.linear_acceleration().z();
    output.linear_acceleration_covariance = input.linear_acceleration_covariance();
    imu_publisher_->publish(std::move(output));
    ++imu_count_;
  }

  std::string cloud_frame_;
  std::string imu_frame_;
  bool use_source_stamp_{false};
  bool preserve_source_timing_{true};
  std::mutex stamp_mutex_;
  std::optional<int64_t> source_to_host_offset_ns_;
  size_t cloud_count_{0};
  size_t imu_count_{0};
  std::chrono::steady_clock::time_point started_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  std::shared_ptr<unitree::robot::ChannelSubscriber<sensor_msgs::msg::dds_::PointCloud2_>>
    cloud_subscriber_;
  std::shared_ptr<unitree::robot::ChannelSubscriber<sensor_msgs::msg::dds_::Imu_>> imu_subscriber_;
  rclcpp::TimerBase::SharedPtr diagnostic_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<UnitreeLidarBridge>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("unitree_lidar_bridge"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
