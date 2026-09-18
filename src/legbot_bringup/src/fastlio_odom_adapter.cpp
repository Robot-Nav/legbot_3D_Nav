#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <legbot_runtime/runtime.hpp>
#include <legbot_runtime/transforms.hpp>

#include <string>
#include <cmath>

class FastLioOdomAdapter
{
public:
  FastLioOdomAdapter() : private_nh_("~"), listener_(legbot::Duration(20.0))
  {
    private_nh_.param<std::string>("base_frame", base_frame_, "base");
    private_nh_.param<std::string>("sensor_frame", sensor_frame_, "livox_imu_link");
    private_nh_.param<std::string>("lidar_frame", lidar_frame_, "livox_link");
    private_nh_.param<bool>("publish_tf", publish_tf_, true);
    private_nh_.param<double>("max_abs_position", max_abs_position_, 10000.0);
    private_nh_.param<double>("max_linear_speed", max_linear_speed_, 20.0);
    private_nh_.param<double>("max_angular_speed", max_angular_speed_, 20.0);
    odom_pub_ = nh_.advertise<nav_msgs::msg::Odometry>("output", 20);
    lidar_odom_pub_ = nh_.advertise<nav_msgs::msg::Odometry>("lidar_output", 20);
    odom_sub_ = nh_.subscribe("input", 20, &FastLioOdomAdapter::odometryCallback, this);
  }

private:
  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr& input)
  {
    if (!input || input->header.frame_id.empty())
      return;

    const auto& p = input->pose.pose.position;
    const auto& q = input->pose.pose.orientation;
    const auto& linear = input->twist.twist.linear;
    const auto& angular = input->twist.twist.angular;
    const double q_norm = std::sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w);
    const bool finite =
        std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
        std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w) &&
        std::isfinite(linear.x) && std::isfinite(linear.y) && std::isfinite(linear.z) &&
        std::isfinite(angular.x) && std::isfinite(angular.y) && std::isfinite(angular.z);
    const bool bounded =
        std::abs(p.x) <= max_abs_position_ && std::abs(p.y) <= max_abs_position_ &&
        std::abs(p.z) <= max_abs_position_ &&
        std::hypot(linear.x, linear.y, linear.z) <= max_linear_speed_ &&
        std::hypot(angular.x, angular.y, angular.z) <= max_angular_speed_ &&
        q_norm >= 0.5 && q_norm <= 1.5;
    if (!finite || !bounded)
    {
      ROS_WARN_THROTTLE(1.0,
                        "[fastlio_odom_adapter] rejected invalid FAST-LIO odometry "
                        "position=[%.3g %.3g %.3g], linear_speed=%.3g, quaternion_norm=%.3g",
                        p.x, p.y, p.z, std::hypot(linear.x, linear.y, linear.z), q_norm);
      return;
    }

    try
    {
      legbot_tf::StampedTransform base_from_sensor;
      listener_.lookupTransform(base_frame_, sensor_frame_, legbot::Time(0), base_from_sensor);
      legbot_tf::StampedTransform sensor_from_lidar;
      listener_.lookupTransform(sensor_frame_, lidar_frame_, legbot::Time(0), sensor_from_lidar);

      legbot_tf::Transform odom_from_sensor;
      legbot_tf::poseMsgToTF(input->pose.pose, odom_from_sensor);
      const legbot_tf::Transform odom_from_base = odom_from_sensor * base_from_sensor.inverse();
      const legbot_tf::Transform odom_from_lidar = odom_from_sensor * sensor_from_lidar;

      nav_msgs::msg::Odometry lidar_output = *input;
      lidar_output.child_frame_id = lidar_frame_;
      legbot_tf::poseTFToMsg(odom_from_lidar, lidar_output.pose.pose);
      lidar_odom_pub_.publish(lidar_output);

      nav_msgs::msg::Odometry output = *input;
      output.child_frame_id = base_frame_;
      legbot_tf::poseTFToMsg(odom_from_base, output.pose.pose);

      const legbot_tf::Matrix3x3 rotation = base_from_sensor.getBasis();
      const legbot_tf::Vector3 linear(input->twist.twist.linear.x,
                               input->twist.twist.linear.y,
                               input->twist.twist.linear.z);
      const legbot_tf::Vector3 angular(input->twist.twist.angular.x,
                                input->twist.twist.angular.y,
                                input->twist.twist.angular.z);
      const legbot_tf::Vector3 linear_base = rotation * linear - (rotation * angular).cross(base_from_sensor.getOrigin());
      const legbot_tf::Vector3 angular_base = rotation * angular;
      output.twist.twist.linear.x = linear_base.x();
      output.twist.twist.linear.y = linear_base.y();
      output.twist.twist.linear.z = linear_base.z();
      output.twist.twist.angular.x = angular_base.x();
      output.twist.twist.angular.y = angular_base.y();
      output.twist.twist.angular.z = angular_base.z();
      odom_pub_.publish(output);

      if (publish_tf_)
      {
        broadcaster_.sendTransform(
            legbot_tf::StampedTransform(odom_from_base, output.header.stamp, output.header.frame_id, base_frame_));
      }
    }
    catch (const legbot_tf::TransformException& error)
    {
      ROS_WARN_THROTTLE(1.0, "[fastlio_odom_adapter] cannot transform %s -> %s: %s",
                        sensor_frame_.c_str(), base_frame_.c_str(), error.what());
    }
  }

  legbot::NodeHandle nh_;
  legbot::NodeHandle private_nh_;
  legbot_tf::TransformListener listener_;
  legbot_tf::TransformBroadcaster broadcaster_;
  legbot::Subscriber odom_sub_;
  legbot::Publisher odom_pub_;
  legbot::Publisher lidar_odom_pub_;
  std::string base_frame_;
  std::string sensor_frame_;
  std::string lidar_frame_;
  bool publish_tf_;
  double max_abs_position_;
  double max_linear_speed_;
  double max_angular_speed_;
};

int main(int argc, char** argv)
{
  legbot::init(argc, argv, "fastlio_odom_adapter");
  FastLioOdomAdapter adapter;
  legbot::spin();
  return 0;
}
