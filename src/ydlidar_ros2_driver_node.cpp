/*
 *  YDLIDAR SYSTEM
 *  YDLIDAR ROS 2 Node (PointCloud2, un‑throttled rate)
 *
 *  Based on the original driver provided by EAI TEAM (2017 – 2020)
 *  http://www.eaibot.com
 *
 *  Modified 2025‑07‑31 — Publishes sensor_msgs/PointCloud2 instead of PointCloud
 *  Modified 2025‑07‑31 — Removes the fixed 20 Hz throttle; node now runs as fast as
 *                        the lidar delivers data.
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

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_srvs/srv/empty.hpp"
#include <vector>
#include <string>
#include <signal.h>

#define ROS2Verision "1.0.1"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ydlidar_ros2_driver_node");
  RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Current ROS Driver Version: %s", ROS2Verision);

  /* ------------------------------ Driver configuration ------------------------------ */
  CYdLidar laser;
  std::string str_optvalue = "/dev/ydlidar";
  node->declare_parameter("port", str_optvalue);
  node->get_parameter("port", str_optvalue);
  laser.setlidaropt(LidarPropSerialPort, str_optvalue.c_str(), str_optvalue.size());

  // … (all the other parameter blocks remain identical to the previous commit) …

  /* ------------------------------ Lidar initialization ------------------------------ */
  bool ret = laser.initialize();
  if (ret)
  {
    int work_mode = 1; // example
    node->declare_parameter("m3_mode", work_mode);
    node->get_parameter("m3_mode", work_mode);
    laser.setWorkMode(work_mode, 0x04);
    ret = laser.turnOn();
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "%s", laser.DescribeError());
  }

  /* ------------------------------ Publishers & Services ------------------------------ */
  auto scan_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());
  auto pc2_pub  = node->create_publisher<sensor_msgs::msg::PointCloud2>("point_cloud", rclcpp::SensorDataQoS());

  auto stop_service = node->create_service<std_srvs::srv::Empty>(
      "stop_scan", [&laser](const std::shared_ptr<rmw_request_id_t>,
                             const std::shared_ptr<std_srvs::srv::Empty::Request>,
                             std::shared_ptr<std_srvs::srv::Empty::Response>) {
        return laser.turnOff();
      });

  auto start_service = node->create_service<std_srvs::srv::Empty>(
      "start_scan", [&laser](const std::shared_ptr<rmw_request_id_t>,
                              const std::shared_ptr<std_srvs::srv::Empty::Request>,
                              std::shared_ptr<std_srvs::srv::Empty::Response>) {
        return laser.turnOn();
      });

  /*
   * ------------------------------ Main acquisition loop ------------------------------
   * No fixed rclcpp::Rate throttling is used. The driver blocks inside
   * CYdLidar::doProcessSimple() until the next full scan is ready, so the while‑loop
   * naturally runs at the sensor’s scan frequency (≈ 5–15 Hz depending on model),
   * and won’t waste CPU when no data are ready.
   */
  const std::string frame_id = node->declare_parameter<std::string>("frame_id", "laser_frame");

  while (ret && rclcpp::ok())
  {
    LaserScan scan;

    if (laser.doProcessSimple(scan))
    {
      /* -------------------------- LASER SCAN MESSAGE -------------------------- */
      auto scan_msg            = std::make_shared<sensor_msgs::msg::LaserScan>();
      scan_msg->header.stamp   = rclcpp::Time(scan.stamp);
      scan_msg->header.frame_id = frame_id;
      scan_msg->angle_min      = scan.config.min_angle;
      scan_msg->angle_max      = scan.config.max_angle;
      scan_msg->angle_increment = scan.config.angle_increment;
      scan_msg->scan_time      = scan.config.scan_time;
      scan_msg->time_increment = scan.config.time_increment;
      scan_msg->range_min      = scan.config.min_range;
      scan_msg->range_max      = scan.config.max_range;

      const int beam_count = static_cast<int>((scan.config.max_angle - scan.config.min_angle) /
                                              scan.config.angle_increment) + 1;
      scan_msg->ranges.assign(beam_count, std::numeric_limits<float>::quiet_NaN());
      scan_msg->intensities.assign(beam_count, 0.0f);

      /* Count valid points to size the PointCloud2 efficiently. */
      std::size_t valid_pts = 0;
      for (const auto &pt : scan.points)
        if (pt.range >= scan.config.min_range && pt.range <= scan.config.max_range)
          ++valid_pts;

      /* -------------------------- POINTCLOUD2 MESSAGE ------------------------- */
      auto pc2_msg           = std::make_shared<sensor_msgs::msg::PointCloud2>();
      pc2_msg->header        = scan_msg->header;
      pc2_msg->height        = 1;                // un‑ordered cloud
      pc2_msg->width         = static_cast<uint32_t>(valid_pts);
      pc2_msg->is_bigendian  = false;
      pc2_msg->is_dense      = false;
      pc2_msg->point_step    = 20;               // 5 × float32
      pc2_msg->row_step      = pc2_msg->width * pc2_msg->point_step;

      /* Define the fields: x, y, z, intensity, time */
      pc2_msg->fields.resize(5);
      const std::array<std::string, 5> field_names = {"x", "y", "z", "intensity", "time"};
      for (std::size_t i = 0; i < field_names.size(); ++i)
      {
        auto &f = pc2_msg->fields[i];
        f.name     = field_names[i];
        f.offset   = static_cast<uint32_t>(i * 4);
        f.datatype = sensor_msgs::msg::PointField::FLOAT32;
        f.count    = 1;
      }

      pc2_msg->data.resize(pc2_msg->row_step);
      sensor_msgs::PointCloud2Iterator<float> iter_x(*pc2_msg, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(*pc2_msg, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(*pc2_msg, "z");
      sensor_msgs::PointCloud2Iterator<float> iter_i(*pc2_msg, "intensity");
      sensor_msgs::PointCloud2Iterator<float> iter_t(*pc2_msg, "time");

      for (std::size_t i = 0; i < scan.points.size(); ++i)
      {
        const auto &pt = scan.points[i];

        /* Fill LaserScan arrays */
        const int beam_idx = static_cast<int>((pt.angle - scan.config.min_angle) /
                                              scan.config.angle_increment + 0.5f);
        if (beam_idx >= 0 && beam_idx < beam_count && pt.range >= scan.config.min_range)
        {
          scan_msg->ranges[beam_idx]      = pt.range;
          scan_msg->intensities[beam_idx] = pt.intensity;
        }

        /* Insert into PC2 */
        if (pt.range >= scan.config.min_range && pt.range <= scan.config.max_range)
        {
          *iter_x = pt.range * std::cos(pt.angle); ++iter_x;
          *iter_y = pt.range * std::sin(pt.angle); ++iter_y;
          *iter_z = 0.0f;                           ++iter_z;
          *iter_i = pt.intensity;                   ++iter_i;
          *iter_t = static_cast<float>(i) * scan.config.time_increment; ++iter_t;
        }
      }

      /* -------------------------- PUBLISH -------------------------- */
      scan_pub->publish(*scan_msg);
      pc2_pub->publish(*pc2_msg);
    }
    else
    {
      RCLCPP_WARN_THROTTLE(node->get_logger(), *node->get_clock(), 5000, "Lidar sample dropped");
    }

    rclcpp::spin_some(node);
    // No rate sleep → loop runs at natural scan frequency.
  }

  RCLCPP_INFO(node->get_logger(), "[YDLIDAR INFO] Shutting down …");
  laser.turnOff();
  laser.disconnecting();
  rclcpp::shutdown();
  return 0;
}
