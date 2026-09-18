#pragma once
// ROS 2 transport for the retained planner algorithms. No ROS 1 runtime or bridge.
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <algorithm>
#include <functional>
#include <sstream>
#include <typeindex>
namespace legbot {
inline rclcpp::Node::SharedPtr &node() { static rclcpp::Node::SharedPtr n; return n; }
inline void init(int argc, char **argv, const std::string &name) {
 rclcpp::init(argc, argv);
 node()=std::make_shared<rclcpp::Node>(name,rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
}
inline bool ok(){return rclcpp::ok();}
inline void shutdown(){rclcpp::shutdown();}
inline void spin(){rclcpp::spin(node());}
inline void spinOnce(){rclcpp::spin_some(node());}
struct Duration {
 rclcpp::Duration value;
 Duration(double seconds=0):value(rclcpp::Duration::from_seconds(seconds)){}
 Duration(rclcpp::Duration v):value(v){}
 double toSec() const{return value.seconds();}
 friend Duration operator+(const Duration&a,const Duration&b){return a.value+b.value;}
 void sleep()const{rclcpp::sleep_for(value.to_chrono<std::chrono::nanoseconds>());}
 operator builtin_interfaces::msg::Duration()const{return value;}
};
struct Time {
 rclcpp::Time value;
 Time(double seconds=0):value(static_cast<int64_t>(seconds*1e9),RCL_ROS_TIME){}
 Time(rclcpp::Time v):value(v){}
 Time(const builtin_interfaces::msg::Time &v):value(v,RCL_ROS_TIME){}
 static Time now(){return node()->now();}
 double toSec()const{return value.seconds();}
 Time& operator+=(const Duration&d){value=value+d.value;return *this;}
 bool isZero()const{return value.nanoseconds()==0;}
 void fromSec(double s){value=rclcpp::Time(static_cast<int64_t>(s*1e9),RCL_ROS_TIME);}
 operator builtin_interfaces::msg::Time()const{return value;}
 friend Duration operator-(const Time&a,const Time&b){return a.value-b.value;}
 friend Time operator+(const Time&a,const Duration&b){return a.value+b.value;}
 friend Time operator-(const Time&a,const Duration&b){return a.value-b.value;}
 friend bool operator<(const Time&a,const Time&b){return a.value<b.value;}
 friend bool operator>(const Time&a,const Time&b){return a.value>b.value;}
 friend bool operator<=(const Time&a,const Time&b){return a.value<=b.value;}
 friend bool operator>=(const Time&a,const Time&b){return a.value>=b.value;}
 friend bool operator==(const Time&a,const Time&b){return a.value==b.value;}
};
struct TimerEvent {};
struct Timer {
 rclcpp::TimerBase::SharedPtr handle;
 void stop(){if(handle)handle->cancel();}
 void start(){if(handle)handle->reset();}
};
struct TransportHints {
 bool transient_local=false;
 TransportHints &tcpNoDelay(){return *this;}
 TransportHints &transientLocal(){transient_local=true;return *this;}
};
struct Subscriber {rclcpp::SubscriptionBase::SharedPtr handle;void shutdown(){handle.reset();}};
struct Publisher {
 rclcpp::PublisherBase::SharedPtr handle;
 template<class M> void publish(const M &msg)const{
  auto p=std::dynamic_pointer_cast<rclcpp::Publisher<M>>(handle);
  if(!p)throw std::logic_error("Publisher message type mismatch");p->publish(msg);
 }
 void shutdown(){handle.reset();}
 size_t getNumSubscribers()const{return handle?handle->get_subscription_count():0;}
};
inline std::string param_name(std::string n){while(!n.empty()&&(n[0]=='/'||n[0]=='~'))n.erase(0,1);std::replace(n.begin(),n.end(),'/','.');return n;}
class NodeHandle {
 bool private_=false;
 std::string topic(std::string n)const {if(private_&&!n.empty()&&n[0]!='/')return "~/"+n;return n;}
 rclcpp::QoS qos(const std::string&n,size_t depth,bool latch=false)const {
  rclcpp::QoS q(depth);
  // Map/path publishers must serve late joining planners. Sensor streams accept best effort DDS.
  if(latch)q.transient_local();
  else if(n.find("cloud")!=std::string::npos||n.find("Pointcloud")!=std::string::npos||n.find("odom")!=std::string::npos||n.find("Odometry")!=std::string::npos||n.find("pose")!=std::string::npos)q.best_effort();
  return q;
 }
 public:
 explicit NodeHandle(const std::string &ns=""):private_(ns=="~"){}
 std::string getNamespace()const{return private_?node()->get_fully_qualified_name():node()->get_namespace();}
 rclcpp::Node* get()const{return node().get();}
 template<class T> void param(const std::string &name,T&value,const T&fallback)const{
  auto key=param_name(name); if(!node()->has_parameter(key))node()->declare_parameter<T>(key,fallback);node()->get_parameter(key,value);
 }
 template<class T> bool getParam(const std::string&name,T&v)const{return node()->get_parameter(param_name(name),v);}
 template<class T> T param(const std::string&name,const T&fallback)const{T v;param(name,v,fallback);return v;}
 template<class M> Publisher advertise(const std::string &n,size_t depth,bool latch=false)const{return {node()->create_publisher<M>(topic(n),qos(n,depth,latch))};}
 template<class M,class F> Subscriber subscribe(const std::string&n,size_t d,F callback,TransportHints hints={})const {
  auto q=qos(n,d); if(n=="/initial_path"||n=="/pct_path"||n=="/navigation/reference_path"||hints.transient_local)q.reliable().transient_local();
  return {node()->create_subscription<M>(topic(n),q,[callback](typename M::ConstSharedPtr m){callback(m);})};
 }
 template<class M> Subscriber subscribe(const std::string&n,size_t d,void(*cb)(const std::shared_ptr<const M>&),TransportHints h={})const{return subscribe<M,decltype(cb)>(n,d,cb,h);}
 template<class M> Subscriber subscribe(const std::string&n,size_t d,void(*cb)(std::shared_ptr<const M>),TransportHints h={})const{return subscribe<M,decltype(cb)>(n,d,cb,h);}
 template<class M,class C> Subscriber subscribe(const std::string&n,size_t d,void(C::*cb)(const std::shared_ptr<const M>&),C*obj,TransportHints h={})const{return subscribe<M>(n,d,[obj,cb](typename M::ConstSharedPtr m){(obj->*cb)(m);},h);}
 template<class F> Timer createTimer(Duration d,F cb)const {return {rclcpp::create_timer(node(),node()->get_clock(),d.value,[cb](){cb(TimerEvent{});})};}
 template<class C> Timer createTimer(Duration d,void(C::*cb)(const TimerEvent&),C*obj)const {return createTimer(d,[obj,cb](const TimerEvent&e){(obj->*cb)(e);});}
};
namespace param {
 template<class T>bool get(const std::string&n,T&v){return NodeHandle().getParam(n,v);}
 template<class T>void param(const std::string&n,T&v,const T&d){NodeHandle().param(n,v,d);}
}
}
#define ROS_INFO(...) RCLCPP_INFO(legbot::node()->get_logger(),__VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(legbot::node()->get_logger(),__VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(legbot::node()->get_logger(),__VA_ARGS__)
#define ROS_DEBUG(...) RCLCPP_DEBUG(legbot::node()->get_logger(),__VA_ARGS__)
#define ROS_INFO_THROTTLE(s,...) RCLCPP_INFO_THROTTLE(legbot::node()->get_logger(),*legbot::node()->get_clock(),static_cast<int64_t>((s)*1000),__VA_ARGS__)
#define ROS_WARN_THROTTLE(s,...) RCLCPP_WARN_THROTTLE(legbot::node()->get_logger(),*legbot::node()->get_clock(),static_cast<int64_t>((s)*1000),__VA_ARGS__)
#define ROS_ERROR_THROTTLE(s,...) RCLCPP_ERROR_THROTTLE(legbot::node()->get_logger(),*legbot::node()->get_clock(),static_cast<int64_t>((s)*1000),__VA_ARGS__)
#define ROS_WARN_STREAM(x) RCLCPP_WARN_STREAM(legbot::node()->get_logger(),x)
#define ROS_INFO_STREAM(x) RCLCPP_INFO_STREAM(legbot::node()->get_logger(),x)
#define ROS_ERROR_STREAM(x) RCLCPP_ERROR_STREAM(legbot::node()->get_logger(),x)
#define ROS_ASSERT(x) assert(x)
