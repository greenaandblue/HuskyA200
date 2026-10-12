// Minimal read-only test: unitree_sdk2 only, no ROS.
// Subscribes to rt/sportmodestate for 10 s and prints the position.
//   ./go2_state_test eth0
#include <atomic>
#include <iostream>
#include <unistd.h>

#include <unitree/robot/channel/channel_factory.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>

using SportState = unitree_go::msg::dds_::SportModeState_;

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::cout << "usage: " << argv[0] << " <network_interface, e.g. eth0>" << std::endl;
    return 1;
  }
  std::cout << "init DDS on " << argv[1] << std::endl;
  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);

  std::atomic<int> n{0};
  unitree::robot::ChannelSubscriberPtr<SportState> sub;
  sub.reset(new unitree::robot::ChannelSubscriber<SportState>("rt/sportmodestate"));
  sub->InitChannel([&n](const void * msg) {
    const auto & s = *static_cast<const SportState *>(msg);
    if (n++ % 50 == 0) {
      std::cout << "pos x=" << s.position()[0] << " y=" << s.position()[1]
                << " yaw=" << s.imu_state().rpy()[2] << std::endl;
    }
  }, 1);

  std::cout << "listening 10 s..." << std::endl;
  sleep(10);
  std::cout << "received " << n.load() << " messages" << std::endl;
  return 0;
}
