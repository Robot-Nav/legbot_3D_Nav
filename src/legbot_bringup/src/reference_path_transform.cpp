#include <nav_msgs/msg/path.hpp>
#include <legbot_runtime/runtime.hpp>
#include <legbot_runtime/transforms.hpp>

#include <string>

class ReferencePathTransform
{
public:
  ReferencePathTransform() : private_nh_("~"), listener_(legbot::Duration(20.0))
  {
    private_nh_.param<std::string>("target_frame", target_frame_, "odom");
    private_nh_.param("z_offset", z_offset_, 0.0);
    path_pub_ = nh_.advertise<nav_msgs::msg::Path>("output", 1, true);
    retry_timer_ = nh_.createTimer(legbot::Duration(0.2), [this](const legbot::TimerEvent&){ if (pending_) pathCallback(pending_); });
    path_sub_ = nh_.subscribe("input", 1, &ReferencePathTransform::pathCallback, this,
                              legbot::TransportHints().transientLocal());
  }

private:
  void pathCallback(const nav_msgs::msg::Path::ConstSharedPtr& input)
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

    nav_msgs::msg::Path output;
    output.header.stamp = legbot::Time::now();
    output.header.frame_id = target_frame_;
    output.poses.reserve(input->poses.size());

    try
    {
      if (source_frame != target_frame_)
        listener_.waitForTransform(target_frame_, source_frame, legbot::Time(0), legbot::Duration(1.0));

      for (const auto& input_pose : input->poses)
      {
        geometry_msgs::msg::PoseStamped source_pose = input_pose;
        if (source_pose.header.frame_id.empty())
          source_pose.header.frame_id = source_frame;
        source_pose.header.stamp = legbot::Time(0);

        geometry_msgs::msg::PoseStamped target_pose;
        if (source_pose.header.frame_id == target_frame_)
          target_pose = source_pose;
        else
          listener_.transformPose(target_frame_, source_pose, target_pose);
        target_pose.pose.position.z += z_offset_;
        target_pose.header.stamp = output.header.stamp;
        output.poses.push_back(target_pose);
      }
    }
    catch (const legbot_tf::TransformException& error)
    {
      pending_ = input;
      ROS_ERROR("[reference_path_transform] cannot transform %s -> %s: %s",
                source_frame.c_str(), target_frame_.c_str(), error.what());
      return;
    }

    pending_.reset();
    path_pub_.publish(output);
    ROS_INFO("[reference_path_transform] published %zu points in %s",
             output.poses.size(), target_frame_.c_str());
  }

  legbot::NodeHandle nh_;
  legbot::NodeHandle private_nh_;
  legbot_tf::TransformListener listener_;
  nav_msgs::msg::Path::ConstSharedPtr pending_;
  legbot::Timer retry_timer_;
  legbot::Subscriber path_sub_;
  legbot::Publisher path_pub_;
  std::string target_frame_;
  double z_offset_;
};

int main(int argc, char** argv)
{
  legbot::init(argc, argv, "reference_path_transform");
  ReferencePathTransform node;
  legbot::spin();
  return 0;
}
