/**
 * *********************************************************
 *
 * @file: path_planner_node.h
 * @brief: Contains the path planner Nav2 GlobalPlanner plugin
 * @author: Yang Haodong
 * @date: 2024-9-24
 * @version: 2.0 (ROS 2 Humble adapted)
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_PATH_PLANNER_PATH_PLANNER_NODE_H_
#define RMP_PATH_PLANNER_PATH_PLANNER_NODE_H_

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <nav2_core/global_planner.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "common/geometry/point.h"
#include "path_planner/path_planner.h"
#include "path_planner/utils/path_planner_factory.h"

namespace rmp::path_planner {

class PathPlannerNode : public nav2_core::GlobalPlanner {
public:
  PathPlannerNode();
  ~PathPlannerNode() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

protected:
  bool _getPlanFromPath(const common::geometry::Points3d& path,
                        nav_msgs::msg::Path& global_path);

protected:
  bool initialized_{ false };
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string name_;
  std::string frame_id_;
  std::string planner_name_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_{ rclcpp::get_logger("PathPlannerNode") };

  std::shared_ptr<PathPlanner> g_planner_;
  PLANNER_TYPE planner_type_;

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr plan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr expand_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr points_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr lines_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr tree_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::Marker>::SharedPtr particles_pub_;
};

}  // namespace rmp::path_planner

#endif
