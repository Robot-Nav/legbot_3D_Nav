#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
class CloudTransform : public rclcpp::Node {
 tf2_ros::Buffer buffer_; tf2_ros::TransformListener listener_;
 rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
 rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
 rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_;
 std::string target_;
 public:
 CloudTransform():Node("cloud_transform"),buffer_(get_clock()),listener_(buffer_) {
  target_=declare_parameter("target_frame",std::string("odom"));
  pose_pub_=create_publisher<nav_msgs::msg::Odometry>("/go2/lidar_pose",rclcpp::SensorDataQoS());
  pub_=create_publisher<sensor_msgs::msg::PointCloud2>("/livox/points_world",rclcpp::SensorDataQoS());
  sub_=create_subscription<sensor_msgs::msg::PointCloud2>("/livox/points_raw",rclcpp::SensorDataQoS(),
   [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr input){
    if(input->header.frame_id.empty())return;
    try {
     auto t=buffer_.lookupTransform(target_,input->header.frame_id,input->header.stamp,rclcpp::Duration::from_seconds(0.05));
     nav_msgs::msg::Odometry pose;pose.header.stamp=input->header.stamp;pose.header.frame_id=target_;pose.child_frame_id=input->header.frame_id;
     pose.pose.pose.position.x=t.transform.translation.x;pose.pose.pose.position.y=t.transform.translation.y;pose.pose.pose.position.z=t.transform.translation.z;
     pose.pose.pose.orientation=t.transform.rotation;pose_pub_->publish(pose);
     sensor_msgs::msg::PointCloud2 out;tf2::doTransform(*input,out,t);out.header.stamp=input->header.stamp;pub_->publish(out);
    }catch(const tf2::TransformException&e){RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Cloud TF unavailable: %s",e.what());}
   });
 }
};
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<CloudTransform>());rclcpp::shutdown();}
