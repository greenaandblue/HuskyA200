// go2_bridge: ROS 2 node (runs on the Go2's Jetson, ROS 2 Foxy) that talks to the Go2
// over unitree_sdk2 (DDS domain 0 on the dog's internal 192.168.123.0/24 network).
//
//   Sub:  cmd_vel   (geometry_msgs/Twist)  -> SportClient::Move(vx, vy, wz)
//   Sub:  command   (std_msgs/String)      -> "stand_up" | "stand_down" | "recovery_stand" | "stop"
//   Pub:  odom      (nav_msgs/Odometry)    <- rt/sportmodestate
//   Srv:  stand_up / stand_down / stop     (std_srvs/Trigger)  -- local use only:
//         services do not cross zenoh-bridge-ros2dds when the RMW is Fast DDS,
//         so remote machines (the Husky) use the `command` topic instead.
//
// Run with ROS_DOMAIN_ID != 0 (domain 0 is the dog's own DDS traffic), e.g.
//   ROS_DOMAIN_ID=1 ros2 run go2_bridge go2_bridge --ros-args -r __ns:=/go2 -p network_interface:=eth0

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <thread>
#include <tf2/LinearMath/Quaternion.h>

#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

using namespace std::chrono_literals;
using SportState = unitree_go::msg::dds_::SportModeState_;

class Go2Bridge : public rclcpp::Node
{
public:
  Go2Bridge() : Node("go2_bridge")
  {
    iface_       = declare_parameter<std::string>("network_interface", "");
    max_vx_      = declare_parameter<double>("max_vx", 0.5);    // m/s
    max_vy_      = declare_parameter<double>("max_vy", 0.3);    // m/s
    max_wz_      = declare_parameter<double>("max_wz", 0.8);    // rad/s
    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.5); // s, stop if no cmd_vel
    odom_frame_  = declare_parameter<std::string>("odom_frame", "go2/odom");
    base_frame_  = declare_parameter<std::string>("base_frame", "go2/base_link");
    double odom_rate = declare_parameter<double>("odom_rate", 50.0); // Hz

    if (iface_.empty()) {
      throw std::runtime_error("parameter 'network_interface' is required (the NIC on 192.168.123.x)");
    }

    // 1) DDS for the Unitree side. Must be initialised before any client/subscriber.
    unitree::robot::ChannelFactory::Instance()->Init(0, iface_);

    // 2) High-level sport client (RPC). Short timeout so a dead link can't block the executor long.
    sport_ = std::make_unique<unitree::robot::go2::SportClient>();
    sport_->SetTimeout(1.0f);
    sport_->Init();

    // 3) State subscriber (runs on an SDK thread; we only copy the latest message here).
    state_sub_.reset(new unitree::robot::ChannelSubscriber<SportState>("rt/sportmodestate"));
    state_sub_->InitChannel(std::bind(&Go2Bridge::onState, this, std::placeholders::_1), 1);

    // ROS side
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&Go2Bridge::onCmdVel, this, std::placeholders::_1));
    mode_sub_ = create_subscription<std_msgs::msg::String>(
      "command", 10, std::bind(&Go2Bridge::onCommand, this, std::placeholders::_1));

    // Explicit callback types: Foxy's rclcpp cannot deduce the signature of generic (auto) lambdas.
    using Trigger = std_srvs::srv::Trigger;
    srv_stand_up_ = create_service<Trigger>("stand_up",
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
        res->success = (sport_->StandUp() == 0); res->message = "StandUp"; });
    srv_stand_down_ = create_service<Trigger>("stand_down",
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
        res->success = (sport_->StandDown() == 0); res->message = "StandDown"; });
    srv_stop_ = create_service<Trigger>("stop",
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
        moving_ = false; res->success = (sport_->StopMove() == 0); res->message = "StopMove"; });

    odom_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / odom_rate), std::bind(&Go2Bridge::publishOdom, this));
    watchdog_timer_ = create_wall_timer(50ms, std::bind(&Go2Bridge::watchdog, this));

    RCLCPP_INFO(get_logger(), "go2_bridge up on interface '%s'", iface_.c_str());
  }

  ~Go2Bridge() override
  {
    if (sport_) sport_->StopMove();
  }

private:
  void onState(const void * msg)
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    state_ = *static_cast<const SportState *>(msg);
    have_state_ = true;
  }

  void publishOdom()
  {
    SportState s;
    {
      std::lock_guard<std::mutex> lk(state_mtx_);
      if (!have_state_) return;
      s = state_;
    }

    const double roll  = s.imu_state().rpy()[0];
    const double pitch = s.imu_state().rpy()[1];
    const double yaw   = s.imu_state().rpy()[2];

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = now();
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    odom.pose.pose.position.x = s.position()[0];
    odom.pose.pose.position.y = s.position()[1];
    odom.pose.pose.position.z = s.position()[2];

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    // sportmodestate velocity is treated as odom/world-frame; Odometry twist is in the
    // child (body) frame, so rotate by -yaw. Verify the sign on the real robot.
    const double vxw = s.velocity()[0], vyw = s.velocity()[1];
    odom.twist.twist.linear.x =  std::cos(yaw) * vxw + std::sin(yaw) * vyw;
    odom.twist.twist.linear.y = -std::sin(yaw) * vxw + std::cos(yaw) * vyw;
    odom.twist.twist.linear.z = s.velocity()[2];
    odom.twist.twist.angular.z = s.yaw_speed();

    odom_pub_->publish(odom);
  }

  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    const float vx = static_cast<float>(std::clamp(msg->linear.x,  -max_vx_, max_vx_));
    const float vy = static_cast<float>(std::clamp(msg->linear.y,  -max_vy_, max_vy_));
    const float wz = static_cast<float>(std::clamp(msg->angular.z, -max_wz_, max_wz_));

    last_cmd_ = now();
    if (vx == 0.f && vy == 0.f && wz == 0.f) {
      if (moving_) { sport_->StopMove(); moving_ = false; }
      return;
    }
    int32_t ret = sport_->Move(vx, vy, wz);
    if (ret != 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Move() returned %d (is the dog standing?)", ret);
    }
    moving_ = true;
  }

  void onCommand(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string & c = msg->data;
    int32_t ret = -1;
    if (c == "stand_up") {
      // Locked stand, then unlock into balance stand so Move() is accepted.
      ret = sport_->StandUp();
      std::this_thread::sleep_for(1500ms);
      if (ret == 0) ret = sport_->BalanceStand();
    } else if (c == "recovery_stand") {
      ret = sport_->RecoveryStand();
    } else if (c == "stand_down") {
      moving_ = false;
      sport_->StopMove();
      ret = sport_->StandDown();
    } else if (c == "stop") {
      moving_ = false;
      ret = sport_->StopMove();
    } else {
      RCLCPP_WARN(get_logger(), "unknown command '%s' (use stand_up | stand_down | recovery_stand | stop)", c.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "command '%s' -> ret=%d", c.c_str(), ret);
  }

  void watchdog()
  {
    if (moving_ && (now() - last_cmd_).seconds() > cmd_timeout_) {
      sport_->StopMove();
      moving_ = false;
      RCLCPP_INFO(get_logger(), "cmd_vel timeout -> StopMove");
    }
  }

  // params
  std::string iface_, odom_frame_, base_frame_;
  double max_vx_, max_vy_, max_wz_, cmd_timeout_;

  // unitree
  std::unique_ptr<unitree::robot::go2::SportClient> sport_;
  unitree::robot::ChannelSubscriberPtr<SportState> state_sub_;
  std::mutex state_mtx_;
  SportState state_;
  bool have_state_ = false;

  // ros
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_stand_up_, srv_stand_down_, srv_stop_;
  rclcpp::TimerBase::SharedPtr odom_timer_, watchdog_timer_;
  rclcpp::Time last_cmd_{0, 0, RCL_ROS_TIME};
  bool moving_ = false;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2Bridge>());
  rclcpp::shutdown();
  return 0;
}
