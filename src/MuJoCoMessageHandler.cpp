#include "MuJoCoMessageHandler.h"

#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/image_encodings.hpp"

namespace deepbreak {

MuJoCoMessageHandler::MuJoCoMessageHandler(mj::Simulate *sim)
    : Node("MuJoCoMessageHandler"), sim_(sim), name_prefix("simulation/") {
  model_param_name = name_prefix + "model_file";
  this->declare_parameter(model_param_name, "");
  
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

  odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(10));
  odom_publisher_load_ = this->create_publisher<nav_msgs::msg::Odometry>("load", rclcpp::QoS(10));
  imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::QoS(10));
  //rgb_img_publisher_ptr_ = this->create_publisher<sensor_msgs::msg::Image>("rgb_image", 100);
  rgb_img_publisher_ptr_ = this->create_publisher<sensor_msgs::msg::Image>(
    "rgb_image", rclcpp::QoS(10));
  
  clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 100);

  timers_.emplace_back(this->create_wall_timer(
      5ms, std::bind(&MuJoCoMessageHandler::odom_callback, this)));

  timers_.emplace_back(this->create_wall_timer(
      5ms, std::bind(&MuJoCoMessageHandler::odom_load_callback, this)));

  timers_.emplace_back(this->create_wall_timer(
      1.5ms, std::bind(&MuJoCoMessageHandler::imu_callback, this)));

  timers_.emplace_back(
  this->create_wall_timer(2000ms, std::bind(&MuJoCoMessageHandler::publish_image, this))); 

  timers_.emplace_back(this->create_wall_timer(
    1ms, std::bind(&MuJoCoMessageHandler::publish_simulation_clock, this)));
  //timers_.emplace_back(this->create_wall_timer(20ms, std::bind(&MuJoCoMessageHandler::publish_image_from_render, this)));
  //timers_.emplace_back(this->create_wall_timer(20ms, std::bind(&MuJoCoMessageHandler::publish_image, this)));
  //subcriber_ = this->create_subscription<example_interfaces::msg::String>("robot_news", 10, std::bind(&ListenerStationNode::callbacklistener, this, std::placeholders::_1));
  actuator_cmd_subscription_ = this->create_subscription<mujoco_msgs::msg::Control>("cmd", qos, std::bind(&MuJoCoMessageHandler::actuator_cmd_callback, this, std::placeholders::_1));

  actuator_cmds_ptr_ = std::make_shared<Control>();
  RCLCPP_INFO(this->get_logger(), "Start MuJoCoMessageHandler ...");

  sim_->uiloadrequest.fetch_add(1);
}

MuJoCoMessageHandler::~MuJoCoMessageHandler() {
  RCLCPP_INFO(this->get_logger(), "close node ...");
}

void MuJoCoMessageHandler::odom_callback() {
  const std::lock_guard<std::mutex> lock(sim_->mtx);
  if (sim_->d != nullptr) {
    auto message = nav_msgs::msg::Odometry();
    message.header.stamp = this->now();
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
void MuJoCoMessageHandler::publish_image() {
  if (!sim_ || !sim_->m) return;

  const mjvScene* scn = &sim_->scn;
  const mjrContext* con = &sim_->platform_ui->mjr_context();
  const mjrRect& viewport = sim_->uistate.rect[3];  // or any suitable rect

  publish_image_from_render(scn, con, viewport);
}
void MuJoCoMessageHandler::publish_image_from_render(const mjvScene* scn, const mjrContext* con, const mjrRect& viewport) {
    int W = viewport.width;
    int H = viewport.height;

    unsigned char* rgb_data = (unsigned char*)std::malloc(3 * W * H);
    if (!rgb_data) {
        RCLCPP_ERROR(this->get_logger(), "Failed to allocate memory for render image.");
        return;
    }

    mjr_readPixels(rgb_data, nullptr, viewport, con);

    // Optional: skip black frames
    bool is_black = std::all_of(rgb_data, rgb_data + 3 * W * H,
                                 [](unsigned char v) { return v == 0; });
    if (is_black) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Skipped black frame");
        std::free(rgb_data);
        return;
    }

    // Flip vertically
    std::vector<unsigned char> flipped_data(3 * W * H);
    for (int row = 0; row < H; ++row) {
        std::memcpy(
            &flipped_data[3 * W * row],
            &rgb_data[3 * W * (H - 1 - row)],
            3 * W
        );
    }

    sensor_msgs::msg::Image image_msg;
    image_msg.header.stamp = this->now();
    image_msg.header.frame_id = "drone";
    image_msg.height = H;
    image_msg.width = W;
    image_msg.encoding = "rgb8";
    image_msg.step = W * 3;
    image_msg.data = std::move(flipped_data);

    rgb_img_publisher_ptr_->publish(image_msg);
    std::free(rgb_data);
}
//void MuJoCoMessageHandler::img_callback() {
//  const std::lock_guard<std::mutex> lock(sim_->mtx);
//    if (sim_->d != nullptr && sim_->m != nullptr) {
//        // Perform a simulation step
//        static mjrContext con;
//        static mjvOption opt;
//        static mjvScene scn;
//
//        // Initialize if not already done
//        static bool initialized = false;
//        if (!initialized) {
//            mjv_defaultOption(&opt);
//            mjv_makeScene(sim_->m, &scn, 1000);  // Adjust the maxgeom if necessary
//            mjr_defaultContext(&con);
//            initialized = true;
//
//            // Find the camera defined in the XML file
//            int camera_id = mj_name2id(sim_->m, mjOBJ_CAMERA, "Upper_view");
//            if (camera_id == -1) {
//                RCLCPP_ERROR(this->get_logger(), "Camera 'Upper_view' not found in the model!");
//            return;
//            }
//            sim_->cam.type = mjCAMERA_FIXED;
//            sim_->cam.fixedcamid = camera_id;
//        }
//        sim_->mtx.unlock();
//        // Render scene
//        mjv_updateScene(sim_->m, sim_->d, &opt, nullptr, &sim_->cam, mjCAT_ALL, &scn);
//        mjrRect viewport = {0, 0, 640, 480};
//        mjr_render(viewport, &scn, &con);
//
//        int W = viewport.width; 
//        int H = viewport.height;
//
//       unsigned char* front_rgb = (unsigned char*)std::malloc(3 * W * H);
//        if (!front_rgb) {
//            RCLCPP_ERROR(this->get_logger(), "Failed to allocate memory for image capture!");
//            return;
//        }
//
//        // Capture image
//        mjr_readPixels(front_rgb, NULL, viewport, &con);
//
//        auto front_image = sensor_msgs::msg::Image();
//        front_image.header.frame_id = "drone";
//        front_image.header.stamp = rclcpp::Clock().now();
//        front_image.height = 480;
//        front_image.width = 640;
//        front_image.encoding = "rgb8";
//        front_image.is_bigendian = 0;
//        front_image.step = front_image.width * 3;
//        size_t front_data_size = front_image.width * front_image.height * 3;
//        front_image.data.resize(front_data_size);
//        std::memcpy(&front_image.data[0], front_rgb, front_data_size);
//        rgb_img_publisher_ptr_->publish(front_image);
//        RCLCPP_INFO(this->get_logger(), "Sending image ...");
//    }
//}

void MuJoCoMessageHandler::odom_load_callback() {

  const std::lock_guard<std::mutex> lock(sim_->mtx);
  if (sim_->d != nullptr) {
    auto message = nav_msgs::msg::Odometry();
    message.header.stamp = this->now();
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
    //actuator_cmds_ptr_->time = this->now();
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
    message.header.stamp = this->now();
    const std::lock_guard<std::mutex> lock(sim_->mtx);

    for (int i = 0; i < sim_->m->nsensor; i++) {
      if (sim_->m->sensor_type[i] == mjtSensor::mjSENS_ACCELEROMETER) {
        message.linear_acceleration.x =
            sim_->d->sensordata[sim_->m->sensor_adr[i]];
        message.linear_acceleration.y =
            sim_->d->sensordata[sim_->m->sensor_adr[i] + 1];
        message.linear_acceleration.z =
            sim_->d->sensordata[sim_->m->sensor_adr[i] + 2] - 9.81;
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

void MuJoCoMessageHandler::publish_simulation_clock() {
    if (sim_ && sim_->d) {
        rosgraph_msgs::msg::Clock clock_msg;
        clock_msg.clock = rclcpp::Time(sim_->d->time * 1e9);  // seconds → nanoseconds
        clock_pub_->publish(clock_msg);
    }
}

std::shared_ptr<MuJoCoMessageHandler::Control>
MuJoCoMessageHandler::get_actuator_cmds_ptr() {
  return actuator_cmds_ptr_;
}
} // namespace deepbreak
