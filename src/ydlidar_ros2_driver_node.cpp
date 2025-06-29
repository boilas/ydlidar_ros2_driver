/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS 2 Node (PointCloud2 version)
 *
 *  Modified to publish sensor_msgs::msg::PointCloud2
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
  // --- Parameter declarations and retrieval ---
  std::string str_optvalue;
  int int_optvalue;
  bool bool_optvalue;
  float float_optvalue;
  std::string frame_id;

  // serial port
  str_optvalue = "/dev/ydlidar";
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
  int_optvalue = 230400;
  node->declare_parameter("baudrate", int_optvalue);
  node->get_parameter("baudrate", int_optvalue);
  laser.setlidaropt(LidarPropSerialBaudrate, &int_optvalue, sizeof(int));

  // lidar type
  int_optvalue = TYPE_TRIANGLE;
  node->declare_parameter("lidar_type", int_optvalue);
  node->get_parameter("lidar_type", int_optvalue);
  laser.setlidaropt(LidarPropLidarType, &int_optvalue, sizeof(int));

  // device type
  int_optvalue = YDLIDAR_TYPE_SERIAL;
  node->declare_parameter("device_type", int_optvalue);
  node->get_parameter("device_type", int_optvalue);
  laser.setlidaropt(LidarPropDeviceType, &int_optvalue, sizeof(int));

  // sample rate
  int_optvalue = 9;
  node->declare_parameter("sample_rate", int_optvalue);
  node->get_parameter("sample_rate", int_optvalue);
  laser.setlidaropt(LidarPropSampleRate, &int_optvalue, sizeof(int));

  // abnormal check count
  int_optvalue = 4;
  node->declare_parameter("abnormal_check_count", int_optvalue);
  node->get_parameter("abnormal_check_count", int_optvalue);
  laser.setlidaropt(LidarPropAbnormalCheckCount, &int_optvalue, sizeof(int));

  // intensity bit
  int_optvalue = 0;
  node->declare_parameter("intensity_bit", int_optvalue);
  node->get_parameter("intensity_bit", int_optvalue);
  laser.setlidaropt(LidarPropIntenstiyBit, &int_optvalue, sizeof(int));

  // fixed resolution
  bool_optvalue = false;
  node->declare_parameter("fixed_resolution", bool_optvalue);
  node->get_parameter("fixed_resolution", bool_optvalue);
  laser.setlidaropt(LidarPropFixedResolution, &bool_optvalue, sizeof(bool));

  // reversion
  bool_optvalue = true;
  node->declare_parameter("reversion", bool_optvalue);
  node->get_parameter("reversion", bool_optvalue);
  laser.setlidaropt(LidarPropReversion, &bool_optvalue, sizeof(bool));

  // inverted
  bool_optvalue = true;
  node->declare_parameter("inverted", bool_optvalue);
  node->get_parameter("inverted", bool_optvalue);
  laser.setlidaropt(LidarPropInverted, &bool_optvalue, sizeof(bool));

  // auto reconnect
  bool_optvalue = true;
  node->declare_parameter("auto_reconnect", bool_optvalue);
  node->get_parameter("auto_reconnect", bool_optvalue);
  laser.setlidaropt(LidarPropAutoReconnect, &bool_optvalue, sizeof(bool));

  // single channel
  bool_optvalue = false;
  node->declare_parameter("isSingleChannel", bool_optvalue);
  node->get_parameter("isSingleChannel", bool_optvalue);
  laser.setlidaropt(LidarPropSingleChannel, &bool_optvalue, sizeof(bool));

  // intensity
  bool_optvalue = false;
  node->declare_parameter("intensity", bool_optvalue);
  node->get_parameter("intensity", bool_optvalue);
  laser.setlidaropt(LidarPropIntenstiy, &bool_optvalue, sizeof(bool));

  // support motor DTR
  bool_optvalue = false;
  node->declare_parameter("support_motor_dtr", bool_optvalue);
  node->get_parameter("support_motor_dtr", bool_optvalue);
  laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &bool_optvalue, sizeof(bool));

  // debug
  bool_optvalue = false;
  node->declare_parameter("debug", bool_optvalue);
  node->get_parameter("debug", bool_optvalue);
  laser.setEnableDebug(bool_optvalue);

  // max & min angle
  float_optvalue = 180.0f;
  node->declare_parameter("angle_max", float_optvalue);
  node->get_parameter("angle_max", float_optvalue);
  laser.setlidaropt(LidarPropMaxAngle, &float_optvalue, sizeof(float));

  float_optvalue = -180.0f;
  node->declare_parameter("angle_min", float_optvalue);
  node->get_parameter("angle_min", float_optvalue);
  laser.setlidaropt(LidarPropMinAngle, &float_optvalue, sizeof(float));

  // max & min range
  float_optvalue = 64.0f;
  node->declare_parameter("range_max", float_optvalue);
  node->get_parameter("range_max", float_optvalue);
  laser.setlidaropt(LidarPropMaxRange, &float_optvalue, sizeof(float));

  float_optvalue = 0.1f;
  node->declare_parameter("range_min", float_optvalue);
  node->get_parameter("range_min", float_optvalue);
  laser.setlidaropt(LidarPropMinRange, &float_optvalue, sizeof(float));

  // scan frequency
  float_optvalue = 10.0f;
  node->declare_parameter("frequency", float_optvalue);
  node->get_parameter("frequency", float_optvalue);
  laser.setlidaropt(LidarPropScanFrequency, &float_optvalue, sizeof(float));

  // invalid range handling
  bool_optvalue = false;
  node->declare_parameter("invalid_range_is_inf", bool_optvalue);
  node->get_parameter("invalid_range_is_inf", bool_optvalue);

  // initialize and start
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
  auto stop_scan_callback = [&laser](
    const std::shared_ptr<rmw_request_id_t>, 
    const std::shared_ptr<std_srvs::srv::Empty::Request>, 
    std::shared_ptr<std_srvs::srv::Empty::Response>) -> bool
  { return laser.turnOff(); };
  node->create_service<std_srvs::srv::Empty>("stop_scan", stop_scan_callback);

  auto start_scan_callback = [&laser](
    const std::shared_ptr<rmw_request_id_t>, 
    const std::shared_ptr<std_srvs::srv::Empty::Request>, 
    std::shared_ptr<std_srvs::srv::Empty::Response>) -> bool
  { return laser.turnOn(); };
  node->create_service<std_srvs::srv::Empty>("start_scan", start_scan_callback);

  rclcpp::WallRate loop_rate(20);

  while (ret && rclcpp::ok()) {
    LaserScan scan;
    if (!laser.doProcessSimple(scan)) {
      RCLCPP_ERROR(node->get_logger(), "Failed to get scan");
      continue;
    }

    // --- Publish LaserScan ---
    auto scan_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
    scan_msg->header.stamp.sec = RCL_NS_TO_S(scan.stamp);
    scan_msg->header.stamp.nanosec = scan.stamp - RCL_S_TO_NS(scan_msg->header.stamp.sec);
    scan_msg->header.frame_id = frame_id;
    scan_msg->angle_min = scan.config.min_angle;
    scan_msg->angle_max = scan.config.max_angle;
    scan_msg->angle_increment = scan.config.angle_increment;
    scan_msg->scan_time = scan.config.scan_time;
    scan_msg->time_increment = scan.config.time_increment;
    scan_msg->range_min = scan.config.min_range;
    scan_msg->range_max = scan.config.max_range;
    int count = static_cast<int>((scan.config.max_angle - scan.config.min_angle) / scan.config.angle_increment) + 1;
    scan_msg->ranges.resize(count);
    scan_msg->intensities.resize(count);

    for (auto &pt : scan.points) {
      int idx = std::lround((pt.angle - scan.config.min_angle) / scan.config.angle_increment);
      if (idx >= 0 && idx < count && pt.range >= scan.config.min_range && pt.range <= scan.config.max_range) {
        scan_msg->ranges[idx]     = pt.range;
        scan_msg->intensities[idx] = pt.intensity;
      }
    }
    laser_pub->publish(*scan_msg);

    // --- Build & Publish PointCloud2 ---
    std::vector<LaserPoint> valid_pts;
    valid_pts.reserve(scan.points.size());
    for (auto &pt : scan.points) {
      if (pt.range >= scan.config.min_range && pt.range <= scan.config.max_range) {
        valid_pts.push_back(pt);
      }
    }

    sensor_msgs::msg::PointCloud2 pc2_msg;
    pc2_msg.header = scan_msg->header;
    pc2_msg.height = 1;
    pc2_msg.width = valid_pts.size();
    pc2_msg.is_bigendian = false;
    pc2_msg.is_dense = false;

    // define fields
    pc2_msg.fields = {
      sensor_msgs::msg::PointField{"x", 0,  sensor_msgs::msg::PointField::FLOAT32, 1},
      sensor_msgs::msg::PointField{"y", 4,  sensor_msgs::msg::PointField::FLOAT32, 1},
      sensor_msgs::msg::PointField{"z", 8,  sensor_msgs::msg::PointField::FLOAT32, 1},
      sensor_msgs::msg::PointField{"intensity", 12, sensor_msgs::msg::PointField::FLOAT32, 1},
      sensor_msgs::msg::PointField{"stamp_offset", 16, sensor_msgs::msg::PointField::FLOAT32, 1}
    };
    pc2_msg.point_step = sizeof(float) * 5;
    pc2_msg.row_step = pc2_msg.point_step * pc2_msg.width;
    pc2_msg.data.resize(pc2_msg.row_step);

    sensor_msgs::PointCloud2Iterator<float> it_x(pc2_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> it_y(pc2_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> it_z(pc2_msg, "z");
    sensor_msgs::PointCloud2Iterator<float> it_i(pc2_msg, "intensity");
    sensor_msgs::PointCloud2Iterator<float> it_s(pc2_msg, "stamp_offset");

    for (size_t i = 0; i < valid_pts.size(); ++i, ++it_x, ++it_y, ++it_z, ++it_i, ++it_s) {
      it_x = valid_pts[i].range * std::cos(valid_pts[i].angle);
      it_y = valid_pts[i].range * std::sin(valid_pts[i].angle);
      it_z = 0.0f;
      it_i = valid_pts[i].intensity;
      it_s = i * scan.config.time_increment;
    }

    pc2_pub->publish(pc2_msg);

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Now YDLIDAR is stopping .......");
  laser.turnOff();
  laser.disconnecting();
  rclcpp::shutdown();
  return 0;
}
