#pragma once
#include <legbot_runtime/runtime.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
namespace legbot_tf {
using Vector3=tf2::Vector3;using Point=tf2::Vector3;using Quaternion=tf2::Quaternion;
using Matrix3x3=tf2::Matrix3x3;using Transform=tf2::Transform;using TransformException=tf2::TransformException;
using tf2::getYaw;
inline void poseMsgToTF(const geometry_msgs::msg::Pose&p,Transform&t){tf2::fromMsg(p,t);}
inline void poseTFToMsg(const Transform&t,geometry_msgs::msg::Pose&p){tf2::toMsg(t,p);}
inline void quaternionMsgToTF(const geometry_msgs::msg::Quaternion&q,Quaternion&t){tf2::fromMsg(q,t);}
inline geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw){Quaternion q;q.setRPY(0,0,yaw);return tf2::toMsg(q);}
struct StampedTransform:Transform {
 legbot::Time stamp_;std::string frame_id_,child_frame_id_;
 StampedTransform()=default;
 StampedTransform(const Transform&t,legbot::Time s,const std::string&f,const std::string&c):Transform(t),stamp_(s),frame_id_(f),child_frame_id_(c){}
};
class TransformListener {
 tf2_ros::Buffer buffer_;tf2_ros::TransformListener listener_;
 public:
 explicit TransformListener(legbot::Duration=legbot::Duration(10)):buffer_(legbot::node()->get_clock()),listener_(buffer_){}
 bool waitForTransform(const std::string&t,const std::string&s,legbot::Time time,legbot::Duration timeout){return buffer_.canTransform(t,s,time.value,timeout.value);}
 void lookupTransform(const std::string&t,const std::string&s,legbot::Time time,StampedTransform&out){auto m=buffer_.lookupTransform(t,s,time.value);tf2::fromMsg(m.transform,static_cast<Transform&>(out));out.frame_id_=m.header.frame_id;out.child_frame_id_=m.child_frame_id;out.stamp_=m.header.stamp;}
 void transformPose(const std::string&t,const geometry_msgs::msg::PoseStamped&s,geometry_msgs::msg::PoseStamped&out){out=buffer_.transform(s,t);}
};
class TransformBroadcaster {
 tf2_ros::TransformBroadcaster broadcaster_{legbot::node()};
 public:
 void sendTransform(const StampedTransform&t){geometry_msgs::msg::TransformStamped m;m.header.frame_id=t.frame_id_;m.child_frame_id=t.child_frame_id_;m.header.stamp=t.stamp_;m.transform=tf2::toMsg(static_cast<const Transform&>(t));broadcaster_.sendTransform(m);}
 void sendTransform(const geometry_msgs::msg::TransformStamped&t){broadcaster_.sendTransform(t);}
};
}
