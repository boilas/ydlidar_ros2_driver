/*
 *  YDLIDAR SYSTEM – ROS 2 Driver (full‑parameters, PointCloud2, un‑throttled)
 *
 *  Fork of the official YDLIDAR driver (© EAI TEAM 2017‑2020).
 *  Changes on 2025‑07‑31 by ChatGPT:
 *     • Replaced deprecated sensor_msgs/PointCloud with sensor_msgs/PointCloud2
 *     • Removed the hard‑coded 20 Hz rate – loop now runs at native scan rate
 *     • Preserved every original parameter & service interface
 */

#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif

#include "src/CYdLidar.h" // YDLIDAR SDK

#include <array>
#include <cmath>
#include <chrono>
#include <csignal>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_srvs/srv/empty.hpp"

#define ROS2Verision "1.0.1"

using LaserScanMsg   = sensor_msgs::msg::LaserScan;
using PointCloud2Msg = sensor_msgs::msg::PointCloud2;
using PointField     = sensor_msgs::msg::PointField;

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ydlidar_ros2_driver_node");
  RCLCPP_INFO(node->get_logger(), "[YDLIDAR] Driver version: %s", ROS2Verision);

  /* ====================================================================== */
  /*                         ===== PARAMETERS =====                         */
  /* ====================================================================== */
  CYdLidar laser;

  /* ---------- String params ---------- */
  std::string port = "/dev/ydlidar";
  node->declare_parameter("port", port);
  node->get_parameter("port", port);
  laser.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());

  std::string ignore_array = "";
  node->declare_parameter("ignore_array", ignore_array);
  node->get_parameter("ignore_array", ignore_array);
  laser.setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(), ignore_array.size());

  std::string frame_id = "laser_frame";
  node->declare_parameter("frame_id", frame_id);
  node->get_parameter("frame_id", frame_id);

  /* ---------- Int params ---------- */
  int baudrate = 230400;
  node->declare_parameter("baudrate", baudrate);
  node->get_parameter("baudrate", baudrate);
  laser.setlidaropt(LidarPropSerialBaudrate, &baudrate, sizeof(int));

  int lidar_type = TYPE_TRIANGLE;
  node->declare_parameter("lidar_type", lidar_type);
  node->get_parameter("lidar_type", lidar_type);
  laser.setlidaropt(LidarPropLidarType, &lidar_type, sizeof(int));

  int device_type = YDLIDAR_TYPE_SERIAL;
  node->declare_parameter("device_type", device_type);
  node->get_parameter("device_type", device_type);
  laser.setlidaropt(LidarPropDeviceType, &device_type, sizeof(int));

  int sample_rate = 9;
  node->declare_parameter("sample_rate", sample_rate);
  node->get_parameter("sample_rate", sample_rate);
  laser.setlidaropt(LidarPropSampleRate, &sample_rate, sizeof(int));

  int abnormal_check_count = 4;
  node->declare_parameter("abnormal_check_count", abnormal_check_count);
  node->get_parameter("abnormal_check_count", abnormal_check_count);
  laser.setlidaropt(LidarPropAbnormalCheckCount, &abnormal_check_count, sizeof(int));

  int intensity_bit = 0;
  node->declare_parameter("intensity_bit", intensity_bit);
  node->get_parameter("intensity_bit", intensity_bit);
  laser.setlidaropt(LidarPropIntenstiyBit, &intensity_bit, sizeof(int));

  /* ---------- Bool params ---------- */
  bool fixed_resolution = false;
  node->declare_parameter("fixed_resolution", fixed_resolution);
  node->get_parameter("fixed_resolution", fixed_resolution);
  laser.setlidaropt(LidarPropFixedResolution, &fixed_resolution, sizeof(bool));

  bool reversion = true;
  node->declare_parameter("reversion", reversion);
  node->get_parameter("reversion", reversion);
  laser.setlidaropt(LidarPropReversion, &reversion, sizeof(bool));

  bool inverted = true;
  node->declare_parameter("inverted", inverted);
  node->get_parameter("inverted", inverted);
  laser.setlidaropt(LidarPropInverted, &inverted, sizeof(bool));

  bool auto_reconnect = true;
  node->declare_parameter("auto_reconnect", auto_reconnect);
  node->get_parameter("auto_reconnect", auto_reconnect);
  laser.setlidaropt(LidarPropAutoReconnect, &auto_reconnect, sizeof(bool));

  bool single_channel = false;
  node->declare_parameter("isSingleChannel", single_channel);
  node->get_parameter("isSingleChannel", single_channel);
  laser.setlidaropt(LidarPropSingleChannel, &single_channel, sizeof(bool));

  bool intensity = false;
  node->declare_parameter("intensity", intensity);
  node->get_parameter("intensity", intensity);
  laser.setlidaropt(LidarPropIntenstiy, &intensity, sizeof(bool));

  bool support_motor_dtr = false;
  node->declare_parameter("support_motor_dtr", support_motor_dtr);
  node->get_parameter("support_motor_dtr", support_motor_dtr);
  laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &support_motor_dtr, sizeof(bool));

  bool debug_en = false;
  node->declare_parameter("debug", debug_en);
  node->get_parameter("debug", debug_en);
  laser.setEnableDebug(debug_en);

  /* ---------- Float params ---------- */
  float angle_max = 180.f;
  node->declare_parameter("angle_max", angle_max);
  node->get_parameter("angle_max", angle_max);
  laser.setlidaropt(LidarPropMaxAngle, &angle_max, sizeof(float));

  float angle_min = -180.f;
  node->declare_parameter("angle_min", angle_min);
  node->get_parameter("angle_min", angle_min);
  laser.setlidaropt(LidarPropMinAngle, &angle_min, sizeof(float));

  float range_max = 64.f;
  node->declare_parameter("range_max", range_max);
  node->get_parameter("range_max", range_max);
  laser.setlidaropt(LidarPropMaxRange, &range_max, sizeof(float));

  float range_min = 0.1f;
  node->declare_parameter("range_min", range_min);
  node->get_parameter("range_min", range_min);
  laser.setlidaropt(LidarPropMinRange, &range_min, sizeof(float));

  float frequency = 10.f;
  node->declare_parameter("frequency", frequency);
  node->get_parameter("frequency", frequency);
  laser.setlidaropt(LidarPropScanFrequency, &frequency, sizeof(float));

  bool invalid_range_is_inf = false;
  node->declare_parameter("invalid_range_is_inf", invalid_range_is_inf);
  node->get_parameter("invalid_range_is_inf", invalid_range_is_inf);

  /* ---------- Work‑mode params ---------- */
  int m1_mode = 0; node->declare_parameter("m1_mode", m1_mode); node->get_parameter("m1_mode", m1_mode); laser.setWorkMode(m1_mode, 0x01);
  int m2_mode = 0; node->declare_parameter("m2_mode", m2_mode); node->get_parameter("m2_mode", m2_mode); laser.setWorkMode(m2_mode, 0x02);
  int m3_mode = 1; node->declare_parameter("m3_mode", m3_mode); node->get_parameter("m3_mode", m3_mode); laser.setWorkMode(m3_mode, 0x04);

  RCLCPP_INFO(node->get_logger(), "[YDLIDAR] Port=%s Baud=%d SampleRate=%d Freq=%.1f", port.c_str(), baudrate, sample_rate, frequency);

  /* ====================================================================== */
  /*                       ===== DRIVER INITIALISATION =====                */
  /* ====================================================================== */
  bool ret = laser.initialize();
  if (ret)
  {
    ret = laser.turnOn();
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "%s", laser.DescribeError());
  }

  /* ====================================================================== */
  /*                       ===== ROS 2 INTERFACES =====                      */
  /* ====================================================================== */
  auto scan_pub = node->create_publisher<LaserScanMsg>("scan", rclcpp::SensorDataQoS());
  auto pc2_pub  = node->create_publisher<PointCloud2Msg>("point_cloud", rclcpp::SensorDataQoS());

  node->create_service<std_srvs::srv::Empty>(
      "stop_scan", [&laser](const std::shared_ptr<rmw_request_id_t>,
                             const std::shared_ptr<std_srvs::srv::Empty::Request>,
                             std::shared_ptr<std_srvs::srv::Empty::Response>) { return laser.turnOff(); });

  node->create_service<std_srvs::srv::Empty>(
      "start_scan", [&laser](const std::shared_ptr<rmw_request_id_t>,
                              const std::shared_ptr<std_srvs::srv::Empty::Request>,
                              std::shared_ptr<std_srvs::srv::Empty::Response>) { return laser.turnOn(); });

  /* ====================================================================== */
  /*                        ===== MAIN ACQUISITION LOOP =====               */
  /* ====================================================================== */
  while (ret && rclcpp::ok())
  {
    LaserScan scan_raw; // struct from CYdLidar.h
    if (laser.doProcessSimple(scan_raw))
    {
      /* --------------------- LaserScan message --------------------- */
      auto scan_msg = std::make_shared<LaserScanMsg>();
      scan_msg->header.stamp    = rclcpp::Time(scan_raw.stamp);
      scan_msg->header.frame_id = frame_id;
      scan_msg->angle_min       = scan_raw.config.min_angle;
      scan_msg->angle_max       = scan_raw.config.max_angle;
      scan_msg->angle_increment = scan_raw.config.angle_increment;
      scan_msg->scan_time       = scan_raw.config.scan_time;
      scan_msg->time_increment  = scan_raw.config.time_increment;
      scan_msg->range_min       = scan_raw.config.min_range;
      scan_msg->range_max       = scan_raw.config.max_range;

      const int beam_count = static_cast<int>((scan_raw.config.max_angle - scan_raw.config.min_angle) /
                                              scan_raw.config.angle_increment) + 1;
      scan_msg->ranges.assign(beam_count, std::numeric_limits<float>::quiet_NaN());
      scan_msg->intensities.assign(beam_count, 0.0f);

      /* Count valid points for PointCloud2 sizing */
      std::size_t valid_pts = 0;
      for (const auto &p : scan_raw.points)
        if (p.range >= scan_raw.config.min_range && p.range <= scan_raw.config.max_range)
          ++valid_pts;

      /* --------------------- PointCloud2 message --------------------- */
      auto pc2_msg = std::make_shared<PointCloud2Msg>();
      pc2_msg->header        = scan_msg->header;
      pc2_msg->height        = 1;
      pc2_msg->width         = static_cast<uint32_t>(valid_pts);
      pc2_msg->is_bigendian  = false;
      pc2_msg->is_dense      = false;
      pc2_msg->point_step    = 20; // 5 × float32
      pc2_msg->row_step      = pc2_msg->width * pc2_msg->point_step;

      pc2_msg->fields.resize(5);
      const std::array<std::string,5> field_names = {"x","y","z","intensity","time"};
      for (std::size_t i=0;i<field_names.size();++i)
      {
        pc2_msg->fields[i].name     = field_names[i];
        pc2_msg->fields[i].offset   = static_cast<uint32_t>(i*4);
        pc2_msg->fields[i].datatype = PointField::FLOAT32;
        pc2_msg->fields[i].count    = 1;
      }

      pc2_msg->data.resize(pc2_msg->row_step);
      sensor_msgs::PointCloud2Iterator<float> iter_x(*pc2_msg,"x"),
                                              iter_y(*pc2_msg,"y"),
                                              iter_z(*pc2_msg,"z"),
                                              iter_i(*pc2_msg,"intensity"),
                                              iter_t(*pc2_msg,"time");

      /* Populate both messages */
      for (std::size_t i = 0; i < scan_raw.points.size(); ++i)
      {
        const auto &pt = scan_raw.points[i];

        /* LaserScan arrays */
        const int idx = static_cast<int>((pt.angle - scan_raw.config.min_angle) / scan_raw.config.angle_increment + 0.5f);
        if (idx >= 0 && idx < beam_count && pt.range >= scan_raw.config.min_range)
        {
          scan_msg->ranges[idx]      = pt.range;
          scan_msg->intensities[idx] = pt.intensity;
        }

        /* PointCloud2 data */
        if (pt.range >= scan_raw.config.min_range && pt.range <= scan_raw.config.max_range)
        {
          *iter_x = pt.range * std::cos(pt.angle); ++iter_x;
          *iter_y = pt.range * std::sin(pt.angle); ++iter_y;
          *iter_z = 0.0f;                           ++iter_z;
          *iter_i = pt.intensity;                   ++iter_i;
          *iter_t = static_cast<float>(i) * scan_raw.config.time_increment; ++iter_t;
        }
      }

      scan_pub->publish(*scan_msg);
      pc2_pub->publish(*pc2_msg);
    }
    else
    {
      RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000, "Lidar scan lost");
    }

    rclcpp::spin_some(node);
  }

  /* ====================================================================== */
  /*                             SHUTDOWN                                   */
  /* ====================================================================== */
  RCLCPP_INFO(node->get_logger(), "[YDLIDAR] Stopping …");
  laser.turnOff();
  laser.disconnecting();
  rclcpp::shutdown();
  return 0;
}
