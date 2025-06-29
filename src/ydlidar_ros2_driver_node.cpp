/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS 2 Node (PointCloud2 version)
 *
 *  Modified to publish sensor_msgs::msg::PointCloud2
 *  Default parameters adjusted per gs2_fast.yaml
 *  Fixed PointCloud2 iterator assignments (now using *iterator = value)
 *
 *  Copyright 2017 - 2020 EAI TEAM
 *  http://www.eaibot.com
 *
 */

#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif

#include "src/CYdLidar.h"
#include <math.h>
#include <chrono>
#include <iostream>
#include <memory>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_srvs/srv/empty.hpp"
#include "rclcpp/rclcpp.hpp"

#define ROS2_VERSION "1.0.1"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ydlidar_ros2_driver_node");

  RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Current ROS Driver Version: %s", ROS2_VERSION);

  CYdLidar laser;
  // --- Parameter declarations and retrieval (defaults per gs2_fast.yaml) ---
  std::string str_optvalue;
  int int_optvalue;
  bool bool_optvalue;
  float float_optvalue;
  std::string frame_id;

  // ─────────── transport ───────────
  str_optvalue = "/dev/ttyUSB0";
  node->declare_parameter("port", str_optvalue);
  node->get_parameter("port", str_optvalue);
  laser.setlidaropt(LidarPropSerialPort, str_optvalue.c_str(), str_optvalue.size());

  // ignore array
  str_optvalue = "";
  node->declare_parameter("ignore_array", str_optvalue);
  node->get_parameter("ignore_array", str_optvalue);
  laser.setlidaropt(LidarPropIgnoreArray, str_optvalue.c_str(), str_optvalue.size());

  // frame id
  frame_id = "laser_frame";
  node->declare_parameter("frame_id", frame_id);
  node->get_parameter("frame_id", frame_id);

  // baudrate
  int_optvalue = 921600;
  node->declare_parameter("baudrate", int_optvalue);
  node->get_parameter("baudrate", int_optvalue);
  laser.setlidaropt(LidarPropSerialBaudrate, &int_optvalue, sizeof(int));

  // ─────────── model & link ───────
  int_optvalue = 3;  // GS‑series solid‑state
  node->declare_parameter("lidar_type", int_optvalue);
  node->get_parameter("lidar_type", int_optvalue);
  laser.setlidaropt(LidarPropLidarType, &int_optvalue, sizeof(int));

  int_optvalue = YDLIDAR_TYPE_SERIAL;  // serial
  node->declare_parameter("device_type", int_optvalue);
  node->get_parameter("device_type", int_optvalue);
  laser.setlidaropt(LidarPropDeviceType, &int_optvalue, sizeof(int));

  int_optvalue = 4;  // dual‑channel
  node->declare_parameter("sample_rate", int_optvalue);
  node->get_parameter("sample_rate", int_optvalue);
  laser.setlidaropt(LidarPropSampleRate, &int_optvalue, sizeof(int));

  int_optvalue = 4;  // retry limit
  node->declare_parameter("abnormal_check_count", int_optvalue);
  node->get_parameter("abnormal_check_count", int_optvalue);
  laser.setlidaropt(LidarPropAbnormalCheckCount, &int_optvalue, sizeof(int));

  // ─────────── data content ───────
  int_optvalue = 8;  // 8‑bit intensity
  node->declare_parameter("intensity_bit", int_optvalue);
  node->get_parameter("intensity_bit", int_optvalue);
  laser.setlidaropt(LidarPropIntenstiyBit, &int_optvalue, sizeof(int));

  bool_optvalue = true;  // fixed resolution
  node->declare_parameter("fixed_resolution", bool_optvalue);
  node->get_parameter("fixed_resolution", bool_optvalue);
  laser.setlidaropt(LidarPropFixedResolution, &bool_optvalue, sizeof(bool));

  bool_optvalue = false; // reversion
  node->declare_parameter("reversion", bool_optvalue);
  node->get_parameter("reversion", bool_optvalue);
  laser.setlidaropt(LidarPropReversion, &bool_optvalue, sizeof(bool));

  bool_optvalue = false; // inverted
  node->declare_parameter("inverted", bool_optvalue);
  node->get_parameter("inverted", bool_optvalue);
  laser.setlidaropt(LidarPropInverted, &bool_optvalue, sizeof(bool));

  bool_optvalue = true; // auto reconnect
  node->declare_parameter("auto_reconnect", bool_optvalue);
  node->get_parameter("auto_reconnect", bool_optvalue);
  laser.setlidaropt(LidarPropAutoReconnect, &bool_optvalue, sizeof(bool));

  bool_optvalue = false; // single channel
  node->declare_parameter("isSingleChannel", bool_optvalue);
  node->get_parameter("isSingleChannel", bool_optvalue);
  laser.setlidaropt(LidarPropSingleChannel, &bool_optvalue, sizeof(bool));

  bool_optvalue = true; // intensity enabled
  node->declare_parameter("intensity", bool_optvalue);
  node->get_parameter("intensity", bool_optvalue);
  laser.setlidaropt(LidarPropIntenstiy, &bool_optvalue, sizeof(bool));

  bool_optvalue = true; // motor DTR
  node->declare_parameter("support_motor_dtr", bool_optvalue);
  node->get_parameter("support_motor_dtr", bool_optvalue);
  laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &bool_optvalue, sizeof(bool));

  bool_optvalue = false; // debug
  node->declare_parameter("debug", bool_optvalue);
  node->get_parameter("debug", bool_optvalue);
  laser.setEnableDebug(bool_optvalue);

  // ─────────── spatial window ──────
  float_optvalue = 180.0f;
  node->declare_parameter("angle_max", float_optvalue);
  node->get_parameter("angle_max", float_optvalue);
  laser.setlidaropt(LidarPropMaxAngle, &float_optvalue, sizeof(float));

  float_optvalue = -180.0f;
  node->declare_parameter("angle_min", float_optvalue);
  node->get_parameter("angle_min", float_optvalue);
  laser.setlidaropt(LidarPropMinAngle, &float_optvalue, sizeof(float));

  float_optvalue = 1.0f;
  node->declare_parameter("range_max", float_optvalue);
  node->get_parameter("range_max", float_optvalue);
  laser.setlidaropt(LidarPropMaxRange, &float_optvalue, sizeof(float));

  float_optvalue = 0.025f;
  node->declare_parameter("range_min", float_optvalue);
  node->get_parameter("range_min", float_optvalue);
  laser.setlidaropt(LidarPropMinRange, &float_optvalue, sizeof(float));

  // scan frequency
  float_optvalue = 10.0f;
  node->declare_parameter("frequency", float_optvalue);
  node->get_parameter("frequency", float_optvalue);
  laser.setlidaropt(LidarPropScanFrequency, &float_optvalue, sizeof(float));

  // invalid range is inf
  bool_optvalue = false;
  node->declare_parameter("invalid_range_is_inf", bool_optvalue);
  node->get_parameter("invalid_range_is_inf", bool_optvalue);

  // ─────────── work‑modes ─────────
  bool ret = laser.initialize();
  if (ret) {
    int mode = 0;
    node->declare_parameter("m1_mode", mode);
    node->get_parameter("m1_mode", mode);
    laser.setWorkMode(mode, 0x01);

    mode = 0;
    node->declare_parameter("m2_mode", mode);
    node->get_parameter("m2_mode", mode);
    laser.setWorkMode(mode, 0x02);

    mode = 1;
    node->declare_parameter("m3_mode", mode);
    node->get_parameter("m3_mode", mode);
    laser.setWorkMode(mode, 0x04);

    ret = laser.turnOn();
  } else {
    RCLCPP_ERROR(node->get_logger(), "%s", laser.DescribeError());
  }

  // publishers
  auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());
  auto pc2_pub   = node->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud2", rclcpp::SensorDataQoS());

  // services
  auto stop_scan_callback = [&laser](const std::shared_ptr<rmw
