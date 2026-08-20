#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <string>

class FastLioOdomAdapter
{
public:
  FastLioOdomAdapter() : private_nh_("~"), listener_(ros::Duration(20.0))
  {
    private_nh_.param<std::string>("base_frame", base_frame_, "base");
    private_nh_.param<std::string>("sensor_frame", sensor_frame_, "livox_imu_link");
    private_nh_.param<bool>("publish_tf", publish_tf_, true);
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("output", 20);
    odom_sub_ = nh_.subscribe("input", 20, &FastLioOdomAdapter::odometryCallback, this);
  }

private:
  void odometryCallback(const nav_msgs::OdometryConstPtr& input)
  {
    if (!input || input->header.frame_id.empty())
      return;

    try
    {
      tf::StampedTransform base_from_sensor;
      listener_.lookupTransform(base_frame_, sensor_frame_, ros::Time(0), base_from_sensor);

      tf::Transform odom_from_sensor;
      tf::poseMsgToTF(input->pose.pose, odom_from_sensor);
      const tf::Transform odom_from_base = odom_from_sensor * base_from_sensor.inverse();

      nav_msgs::Odometry output = *input;
      output.child_frame_id = base_frame_;
      tf::poseTFToMsg(odom_from_base, output.pose.pose);

      const tf::Matrix3x3 rotation = base_from_sensor.getBasis();
      const tf::Vector3 linear(input->twist.twist.linear.x,
                               input->twist.twist.linear.y,
                               input->twist.twist.linear.z);
      const tf::Vector3 angular(input->twist.twist.angular.x,
                                input->twist.twist.angular.y,
                                input->twist.twist.angular.z);
      const tf::Vector3 linear_base = rotation * linear;
      const tf::Vector3 angular_base = rotation * angular;
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
            tf::StampedTransform(odom_from_base, output.header.stamp, output.header.frame_id, base_frame_));
      }
    }
    catch (const tf::TransformException& error)
    {
      ROS_WARN_THROTTLE(1.0, "[fastlio_odom_adapter] cannot transform %s -> %s: %s",
                        sensor_frame_.c_str(), base_frame_.c_str(), error.what());
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  tf::TransformListener listener_;
  tf::TransformBroadcaster broadcaster_;
  ros::Subscriber odom_sub_;
  ros::Publisher odom_pub_;
  std::string base_frame_;
  std::string sensor_frame_;
  bool publish_tf_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fastlio_odom_adapter");
  FastLioOdomAdapter adapter;
  ros::spin();
  return 0;
}
