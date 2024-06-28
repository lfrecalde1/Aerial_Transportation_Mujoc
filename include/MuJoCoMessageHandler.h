#pragma once

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/types.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "communication/msg/actuator_cmds.hpp"
#include "communication/msg/touch_sensor.hpp"
#include "communication/srv/simulation_reset.hpp"
#include "mujoco_msgs/msg/control.hpp"

#include "array_safety.h"
#include "simulate.h"

using namespace rclcpp;

using namespace std::chrono_literals;

namespace deepbreak {
namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

class MuJoCoMessageHandler : public rclcpp::Node {
public:
  struct ActuatorCmds {
    double time = 0.0;
    std::vector<std::string> actuators_name;
    std::vector<float> kp;
    std::vector<float> pos;
    std::vector<float> kd;
    std::vector<float> vel;
    std::vector<float> torque;
  };

  struct Control {
    double time = 0.0;
    float thrust;
    float torque_x;
    float torque_y;
    float torque_z;
  };

  MuJoCoMessageHandler(mj::Simulate *sim);
  ~MuJoCoMessageHandler();

  std::shared_ptr<Control> get_actuator_cmds_ptr();

private:

  void odom_callback();
  void odom_load_callback();
  void imu_callback();
  void actuator_cmd_callback(
      const mujoco_msgs::msg::Control::SharedPtr msg) const;

  mj::Simulate *sim_;
  std::string name_prefix, model_param_name;
  std::vector<rclcpp::TimerBase::SharedPtr> timers_;
  std::once_flag once_flag_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_load_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;

  rclcpp::Subscription<mujoco_msgs::msg::Control>::SharedPtr
      actuator_cmd_subscription_;

  std::shared_ptr<Control> actuator_cmds_ptr_;

  std::thread spin_thread;
};

} // namespace deepbreak
