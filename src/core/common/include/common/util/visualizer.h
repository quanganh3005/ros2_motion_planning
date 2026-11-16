/**
 * *********************************************************
 *
 * @file: visualizer.h
 * @brief: Contains visualization functions
 * @author: Yang Haodong
 * @date: 2024-09-24
 * @version: 2.0 (ROS 2 Humble adapted)
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_COMMON_UTIL_VISUALIZER_H_
#define RMP_COMMON_UTIL_VISUALIZER_H_

#include <rclcpp/rclcpp.hpp>
#include <costmap_2d/costmap_2d.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "common/geometry/point.h"
#include "common/structure/singleton.h"

namespace rmp::common::util {
class Visualizer {
public:
  using Line2d = std::pair<common::geometry::Point2d, common::geometry::Point2d>;
  using Lines2d = std::vector<Line2d>;

public:
  /**
   * @brief Publish path
   */
  template <typename Point, typename PublisherT>
  static void publishPlan(const std::vector<Point>& plan, PublisherT& publisher,
                          const std::string& frame_id, const rclcpp::Time& stamp = rclcpp::Clock().now()) {
    nav_msgs::msg::Path gui_plan;
    gui_plan.poses.resize(plan.size());
    gui_plan.header.frame_id = frame_id;
    gui_plan.header.stamp = stamp;
    for (size_t i = 0; i < plan.size(); i++) {
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, plan[i].theta());
      gui_plan.poses[i].header.stamp = stamp;
      gui_plan.poses[i].header.frame_id = frame_id;
      gui_plan.poses[i].pose.position.x = plan[i].x();
      gui_plan.poses[i].pose.position.y = plan[i].y();
      gui_plan.poses[i].pose.position.z = 0.0;
      gui_plan.poses[i].pose.orientation.x = q.getX();
      gui_plan.poses[i].pose.orientation.y = q.getY();
      gui_plan.poses[i].pose.orientation.z = q.getZ();
      gui_plan.poses[i].pose.orientation.w = q.getW();
    }
    publisher->publish(gui_plan);
  }

  template <typename Point, typename PublisherT>
  static void publishExpandZone(const std::vector<Point>& expand,
                                const costmap_2d::Costmap2D* costmap,
                                PublisherT& publisher,
                                const std::string& frame_id,
                                const rclcpp::Time& stamp = rclcpp::Clock().now()) {
    nav_msgs::msg::OccupancyGrid grid;
    float resolution = costmap->getResolution();

    grid.header.frame_id = frame_id;
    grid.header.stamp = stamp;
    grid.info.resolution = resolution;
    grid.info.width = costmap->getSizeInCellsX();
    grid.info.height = costmap->getSizeInCellsY();

    double wx, wy;
    costmap->mapToWorld(0, 0, wx, wy);
    grid.info.origin.position.x = wx - resolution / 2;
    grid.info.origin.position.y = wy - resolution / 2;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;
    grid.data.resize(grid.info.width * grid.info.height, 0);

    auto grid2index = [&](const Point& pt) {
      return static_cast<int>(pt.x()) +
             static_cast<int>(grid.info.width) * static_cast<int>(pt.y());
    };

    for (const auto& pt : expand) {
      int idx = grid2index(pt);
      if (idx >= 0 && idx < static_cast<int>(grid.data.size())) {
        grid.data[idx] = 50;
      }
    }

    publisher->publish(grid);
  }

  /**
   * @brief publish points
   */
  template <typename Point, typename PublisherT>
  static void publishPoints(const std::vector<Point>& points,
                            PublisherT& publisher, const std::string& frame_id,
                            const std::string& ns,
                            const rclcpp::Time& stamp = rclcpp::Clock().now(),
                            std_msgs::msg::ColorRGBA color = RED,
                            double scale = 0.2, int type = SPHERE) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = ns;
    marker.id = 0;

    switch (type) {
      case CUBE:
        marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        break;
      case SPHERE:
      default:
        marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    }

    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.color = color;

    marker.points.reserve(points.size());
    for (const auto& pt : points) {
      geometry_msgs::msg::Point p;
      p.x = pt.x();
      p.y = pt.y();
      p.z = 0.0;
      marker.points.push_back(p);
    }

    if (!marker.points.empty()) {
      publisher->publish(marker);
    }
  }

  /**
   * @brief publish lines
   */
  template <typename PublisherT>
  static void publishLines2d(const Lines2d& lines, PublisherT& publisher,
                             const std::string& frame_id, const std::string& ns,
                             const rclcpp::Time& stamp = rclcpp::Clock().now(),
                             std_msgs::msg::ColorRGBA color = RED, double scale = 0.2) {
    visualization_msgs::msg::Marker line_marker;
    line_marker.header.frame_id = frame_id;
    line_marker.header.stamp = stamp;
    line_marker.ns = ns;
    line_marker.id = 0;
    line_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    line_marker.lifetime.sec = 0;
    line_marker.lifetime.nanosec = 500000000;
    line_marker.pose.orientation.w = 1.0;
    line_marker.scale.x = scale;
    line_marker.scale.y = scale;
    line_marker.scale.z = scale;
    line_marker.color = color;

    for (const auto& line : lines) {
      geometry_msgs::msg::Point start, end;
      start.x = line.first.x();
      start.y = line.first.y();
      start.z = 0.0;

      end.x = line.second.x();
      end.y = line.second.y();
      end.z = 0.0;

      line_marker.points.push_back(start);
      line_marker.points.push_back(end);
    }

    if (!line_marker.points.empty()) {
      publisher->publish(line_marker);
    }
  }

  static std_msgs::msg::ColorRGBA _colorInit(double r, double g, double b, double a);

public:
  static std_msgs::msg::ColorRGBA RED;
  static std_msgs::msg::ColorRGBA DARK_GREEN;
  static std_msgs::msg::ColorRGBA PURPLE;
  static std_msgs::msg::ColorRGBA LIGHT_PURPLE;
  static std_msgs::msg::ColorRGBA LIGHT_ORANGE;
  enum MARKER_TYPE {
    CUBE = 0,
    SPHERE = 1,
  };
};

using VisualizerPtr = rmp::common::structure::Singleton<Visualizer>;

}  // namespace rmp::common::util

#endif
