#include "MuJoCoMessageHandler.h"

#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.hpp"

namespace deepbreak {

MuJoCoMessageHandler::MuJoCoMessageHandler(mj::Simulate *sim)
    : Node("MuJoCoMessageHandler"), sim_(sim), name_prefix("simulation/") {
  model_param_name = name_prefix + "model_file";
  this->declare_parameter(model_param_name, "");
  
  odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 100);
  odom_publisher_load_ = this->create_publisher<nav_msgs::msg::Odometry>("load", 100);
  imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 100);

  timers_.emplace_back(this->create_wall_timer(
      10ms, std::bind(&MuJoCoMessageHandler::odom_callback, this)));

  timers_.emplace_back(this->create_wall_timer(
      10ms, std::bind(&MuJoCoMessageHandler::odom_load_callback, this)));

  timers_.emplace_back(this->create_wall_timer(
      2.5ms, std::bind(&MuJoCoMessageHandler::imu_callback, this)));
    
  actuator_cmd_subscription_ = this->create_subscription<mujoco_msgs::msg::Control>("cmd", 100,
          std::bind(&MuJoCoMessageHandler::actuator_cmd_callback, this,
                    std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Start MuJoCoMessageHandler ...");

  std::string model_file = this->get_parameter(model_param_name)
                               .get_parameter_value()
                               .get<std::string>();
  mju::strcpy_arr(sim_->filename, model_file.c_str());
  sim_->uiloadrequest.fetch_add(1);
}

MuJoCoMessageHandler::~MuJoCoMessageHandler() {
  RCLCPP_INFO(this->get_logger(), "close node ...");
}

void MuJoCoMessageHandler::odom_callback() {
  const std::lock_guard<std::mutex> lock(sim_->mtx);
  if (sim_->d != nullptr) {
    auto message = nav_msgs::msg::Odometry();
    message.header.stamp = this->get_clock()->now();
    message.header.frame_id = "map";

    message.pose.pose.position.x = sim_->d->qpos[0];
    message.pose.pose.position.y = sim_->d->qpos[1];
    message.pose.pose.position.z = sim_->d->qpos[2];

    message.pose.pose.orientation.w = sim_->d->qpos[3];
    message.pose.pose.orientation.x = sim_->d->qpos[4];
    message.pose.pose.orientation.y = sim_->d->qpos[5];
    message.pose.pose.orientation.z = sim_->d->qpos[6];

    // Velocities body frame
    for (int i = 0; i < sim_->m->nsensor; i++) {
      if (sim_->m->sensor_type[i] == mjtSensor::mjSENS_VELOCIMETER) {
          message.twist.twist.linear.x = sim_->d->sensordata[sim_->m->sensor_adr[i]];
          message.twist.twist.linear.y = sim_->d->sensordata[sim_->m->sensor_adr[i] +1];
          message.twist.twist.linear.z = sim_->d->sensordata[sim_->m->sensor_adr[i] + 2];
      } 
      if (sim_->m->sensor_type[i] == mjtSensor::mjSENS_GYRO) {
          message.twist.twist.angular.x = sim_->d->sensordata[sim_->m->sensor_adr[i]];
          message.twist.twist.angular.y = sim_->d->sensordata[sim_->m->sensor_adr[i] + 1];
          message.twist.twist.angular.z = sim_->d->sensordata[sim_->m->sensor_adr[i] + 2];
      } 
    }
    odom_publisher_->publish(message);
  }
}


void MuJoCoMessageHandler::odom_load_callback() {

  const std::lock_guard<std::mutex> lock(sim_->mtx);
  if (sim_->d != nullptr) {
    auto message = nav_msgs::msg::Odometry();
    message.header.stamp = this->get_clock()->now();
    message.header.frame_id = "map";

    message.pose.pose.position.x = sim_->d->qpos[7];
    message.pose.pose.position.y = sim_->d->qpos[8];
    message.pose.pose.position.z = sim_->d->qpos[9];

    message.pose.pose.orientation.w = sim_->d->qpos[10];
    message.pose.pose.orientation.x = sim_->d->qpos[11];
    message.pose.pose.orientation.y = sim_->d->qpos[12];
    message.pose.pose.orientation.z = sim_->d->qpos[13];

    message.twist.twist.linear.x = sim_->d->qvel[6];
    message.twist.twist.linear.y = sim_->d->qvel[7];
    message.twist.twist.linear.z = sim_->d->qvel[8];

    message.twist.twist.angular.x = sim_->d->qvel[9];
    message.twist.twist.angular.y = sim_->d->qvel[10];
    message.twist.twist.angular.z = sim_->d->qvel[11];
    odom_publisher_load_->publish(message);
  }
}

void MuJoCoMessageHandler::actuator_cmd_callback(
    const mujoco_msgs::msg::Control::SharedPtr msg) const {
  if (sim_->d != nullptr) {
    actuator_cmds_ptr_->time = this->now().seconds();
    actuator_cmds_ptr_->thrust = msg->thrust;
    actuator_cmds_ptr_->torque_x = msg->torque_x;
    actuator_cmds_ptr_->torque_y = msg->torque_y;
    actuator_cmds_ptr_->torque_z = msg->torque_z;
    // RCLCPP_INFO(this->get_logger(), "subscribe actuator cmds");
  }
}

void MuJoCoMessageHandler::imu_callback() {
  if (sim_->d != nullptr) {
    auto message = sensor_msgs::msg::Imu();
    message.header.frame_id = "imu_link";
    message.header.stamp = rclcpp::Clock().now();
    const std::lock_guard<std::mutex> lock(sim_->mtx);

    for (int i = 0; i < sim_->m->nsensor; i++) {
      if (sim_->m->sensor_type[i] == mjtSensor::mjSENS_ACCELEROMETER) {
        message.linear_acceleration.x =
            sim_->d->sensordata[sim_->m->sensor_adr[i]];
        message.linear_acceleration.y =
            sim_->d->sensordata[sim_->m->sensor_adr[i] + 1];
        message.linear_acceleration.z =
            sim_->d->sensordata[sim_->m->sensor_adr[i] + 2];
      } else if (sim_->m->sensor_type[i] == mjtSensor::mjSENS_FRAMEQUAT) {
        message.orientation.w = sim_->d->sensordata[sim_->m->sensor_adr[i]];
        message.orientation.x = sim_->d->sensordata[sim_->m->sensor_adr[i] + 1];
        message.orientation.y = sim_->d->sensordata[sim_->m->sensor_adr[i] + 2];
        message.orientation.z = sim_->d->sensordata[sim_->m->sensor_adr[i] + 3];
      } else if (sim_->m->sensor_type[i] == mjtSensor::mjSENS_GYRO) {
        message.angular_velocity.x =
            sim_->d->sensordata[sim_->m->sensor_adr[i]];
        message.angular_velocity.y =
            sim_->d->sensordata[sim_->m->sensor_adr[i] + 1];
        message.angular_velocity.z =
            sim_->d->sensordata[sim_->m->sensor_adr[i] + 2];
      }
    }
    imu_publisher_->publish(message);
  }
}
} // namespace deepbreak
