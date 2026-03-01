#include <rclcpp/rclcpp.hpp>
#include <cpdwc/cpdwc.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cpdwc::CPDWController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
