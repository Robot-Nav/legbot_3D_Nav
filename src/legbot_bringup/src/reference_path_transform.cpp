#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <string>

class ReferencePathTransform
{
public:
  ReferencePathTransform() : private_nh_("~"), listener_(ros::Duration(20.0))
  {
    private_nh_.param<std::string>("target_frame", target_frame_, "odom");
    private_nh_.param("z_offset", z_offset_, 0.0);
    path_pub_ = nh_.advertise<nav_msgs::Path>("output", 1, true);
    path_sub_ = nh_.subscribe("input", 1, &ReferencePathTransform::pathCallback, this);
  }

private:
  void pathCallback(const nav_msgs::PathConstPtr& input)
  {
    if (!input || input->poses.empty())
      return;

    const std::string source_frame = input->header.frame_id.empty()
                                         ? input->poses.front().header.frame_id
                                         : input->header.frame_id;
    if (source_frame.empty())
    {
      ROS_ERROR_THROTTLE(1.0, "[reference_path_transform] input path has no frame_id");
      return;
    }

    nav_msgs::Path output;
    output.header.stamp = ros::Time::now();
    output.header.frame_id = target_frame_;
    output.poses.reserve(input->poses.size());

    try
    {
      if (source_frame != target_frame_)
        listener_.waitForTransform(target_frame_, source_frame, ros::Time(0), ros::Duration(1.0));

      for (const auto& input_pose : input->poses)
      {
        geometry_msgs::PoseStamped source_pose = input_pose;
        if (source_pose.header.frame_id.empty())
          source_pose.header.frame_id = source_frame;
        source_pose.header.stamp = ros::Time(0);

        geometry_msgs::PoseStamped target_pose;
        if (source_pose.header.frame_id == target_frame_)
          target_pose = source_pose;
        else
          listener_.transformPose(target_frame_, source_pose, target_pose);
        target_pose.pose.position.z += z_offset_;
        target_pose.header.stamp = output.header.stamp;
        output.poses.push_back(target_pose);
      }
    }
    catch (const tf::TransformException& error)
    {
      ROS_ERROR("[reference_path_transform] cannot transform %s -> %s: %s",
                source_frame.c_str(), target_frame_.c_str(), error.what());
      return;
    }

    path_pub_.publish(output);
    ROS_INFO("[reference_path_transform] published %zu points in %s",
             output.poses.size(), target_frame_.c_str());
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  tf::TransformListener listener_;
  ros::Subscriber path_sub_;
  ros::Publisher path_pub_;
  std::string target_frame_;
  double z_offset_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "reference_path_transform");
  ReferencePathTransform node;
  ros::spin();
  return 0;
}
