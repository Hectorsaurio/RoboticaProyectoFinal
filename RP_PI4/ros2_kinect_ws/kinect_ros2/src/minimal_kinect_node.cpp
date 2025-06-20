#include "rclcpp/rclcpp.hpp"
#include "kinect_ros2/kinect_ros2_component.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  options.use_intra_process_comms(true);

  auto kinect_component = std::make_shared<kinect_ros2::KinectRosComponent>(options);

  exec.add_node(kinect_component);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}