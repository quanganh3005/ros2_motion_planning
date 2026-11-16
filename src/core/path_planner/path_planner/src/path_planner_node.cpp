/**
 * *********************************************************
 *
 * @file: path_planner_node.cpp
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
#include <tf2/utils.h>
#include <pluginlib/class_list_macros.hpp>
#include <nav2_util/node_utils.hpp>

#include "path_planner/path_planner_node.h"
#include "common/util/log.h"
#include "common/util/visualizer.h"

PLUGINLIB_EXPORT_CLASS(rmp::path_planner::PathPlannerNode, nav2_core::GlobalPlanner)

namespace rmp::path_planner {
using Visualizer = rmp::common::util::Visualizer;

PathPlannerNode::PathPlannerNode() : initialized_(false), g_planner_(nullptr) {}

void PathPlannerNode::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("Unable to lock parent node in PathPlannerNode::configure");
  }

  node_ = parent;
  name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  clock_ = node->get_clock();
  logger_ = node->get_logger();
  frame_id_ = costmap_ros_->getGlobalFrameID();

  nav2_util::declare_parameter_if_not_declared(
    node, name_ + ".planner_name", rclcpp::ParameterValue("astar"));
  node->get_parameter(name_ + ".planner_name", planner_name_);

  RCLCPP_INFO(logger_, "Configuring PathPlannerNode [%s] with algorithm: %s",
              name_.c_str(), planner_name_.c_str());

  int nx = static_cast<int>(costmap_ros_->getCostmap()->getSizeInCellsX());
  int ny = static_cast<int>(costmap_ros_->getCostmap()->getSizeInCellsY());
  if (nx > 10 && ny > 10) {
    PathPlannerFactory::PlannerProps path_planner_props;
    if (PathPlannerFactory::createPlanner(planner_name_, costmap_ros_.get(), path_planner_props)) {
      g_planner_ = path_planner_props.planner_ptr;
      planner_type_ = path_planner_props.planner_type;
    }
  } else {
    RCLCPP_INFO(logger_, "Costmap not yet loaded (size: %d x %d). Planner [%s] will be initialized upon first plan request.",
                nx, ny, planner_name_.c_str());
  }

  plan_pub_ = node->create_publisher<nav_msgs::msg::Path>(name_ + "/plan", 1);
  expand_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>(name_ + "/expand", 1);
  points_pub_ = node->create_publisher<visualization_msgs::msg::Marker>(name_ + "/key_points", 1);
  lines_pub_ = node->create_publisher<visualization_msgs::msg::Marker>(name_ + "/safety_corridor", 1);
  tree_pub_ = node->create_publisher<visualization_msgs::msg::Marker>(name_ + "/random_tree", 1);
  particles_pub_ = node->create_publisher<visualization_msgs::msg::Marker>(name_ + "/particles", 1);

  initialized_ = true;
}

void PathPlannerNode::cleanup() {
  RCLCPP_INFO(logger_, "Cleaning up PathPlannerNode [%s]", name_.c_str());
  plan_pub_.reset();
  expand_pub_.reset();
  points_pub_.reset();
  lines_pub_.reset();
  tree_pub_.reset();
  particles_pub_.reset();
  g_planner_.reset();
  initialized_ = false;
}

void PathPlannerNode::activate() {
  RCLCPP_INFO(logger_, "Activating PathPlannerNode [%s]", name_.c_str());
  if (plan_pub_) plan_pub_->on_activate();
  if (expand_pub_) expand_pub_->on_activate();
  if (points_pub_) points_pub_->on_activate();
  if (lines_pub_) lines_pub_->on_activate();
  if (tree_pub_) tree_pub_->on_activate();
  if (particles_pub_) particles_pub_->on_activate();
}

void PathPlannerNode::deactivate() {
  RCLCPP_INFO(logger_, "Deactivating PathPlannerNode [%s]", name_.c_str());
  if (plan_pub_) plan_pub_->on_deactivate();
  if (expand_pub_) expand_pub_->on_deactivate();
  if (points_pub_) points_pub_->on_deactivate();
  if (lines_pub_) lines_pub_->on_deactivate();
  if (tree_pub_) tree_pub_->on_deactivate();
  if (particles_pub_) particles_pub_->on_deactivate();
}

nav_msgs::msg::Path PathPlannerNode::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;
  global_path.header.stamp = clock_->now();
  global_path.header.frame_id = frame_id_;

  if (!initialized_) {
    RCLCPP_ERROR(logger_, "PathPlannerNode has not been initialized yet!");
    return global_path;
  }

  auto costmap = costmap_ros_->getCostmap();
  if (!costmap) {
    RCLCPP_ERROR(logger_, "Costmap pointer is null!");
    return global_path;
  }

  // Mutex lock on costmap
  std::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  int current_nx = static_cast<int>(costmap->getSizeInCellsX());
  int current_ny = static_cast<int>(costmap->getSizeInCellsY());

  if (current_nx <= 0 || current_ny <= 0) {
    RCLCPP_WARN(logger_, "Global costmap has invalid dimensions (%d x %d). Waiting for map...", current_nx, current_ny);
    return global_path;
  }

  std::string configured_planner_name = planner_name_;
  if (auto node = node_.lock()) {
    node->get_parameter(name_ + ".planner_name", configured_planner_name);
  }

  if (!g_planner_ ||
      g_planner_->getNx() != current_nx ||
      g_planner_->getNy() != current_ny ||
      configured_planner_name != planner_name_) {
    planner_name_ = configured_planner_name;
    RCLCPP_INFO(logger_, "Initializing planner [%s] for active costmap: %d x %d (origin: %.2f, %.2f, res: %.3f)",
                planner_name_.c_str(), current_nx, current_ny,
                costmap->getOriginX(), costmap->getOriginY(), costmap->getResolution());
    PathPlannerFactory::PlannerProps path_planner_props;
    if (!PathPlannerFactory::createPlanner(planner_name_, costmap_ros_.get(), path_planner_props)) {
      RCLCPP_ERROR(logger_, "Create path planner failed: %s", planner_name_.c_str());
      return global_path;
    }
    g_planner_ = path_planner_props.planner_ptr;
    planner_type_ = path_planner_props.planner_type;
  }

  // Judge whether goal and start node in costmap frame or not
  if (goal.header.frame_id != frame_id_) {
    RCLCPP_ERROR(logger_, "The goal pose frame_id (%s) must match global frame_id (%s)",
                 goal.header.frame_id.c_str(), frame_id_.c_str());
    return global_path;
  }

  if (start.header.frame_id != frame_id_) {
    RCLCPP_ERROR(logger_, "The start pose frame_id (%s) must match global frame_id (%s)",
                 start.header.frame_id.c_str(), frame_id_.c_str());
    return global_path;
  }

  const auto& visualizer = rmp::common::util::VisualizerPtr::Instance();

  if (g_planner_->config().is_outline_map()) {
    g_planner_->outlineMap();
  }

  common::geometry::Points3d origin_plan;
  common::geometry::Points3d expand;
  bool path_found = false;

  path_found = g_planner_->plan(
    { start.pose.position.x, start.pose.position.y, tf2::getYaw(start.pose.orientation) },
    { goal.pose.position.x, goal.pose.position.y, tf2::getYaw(goal.pose.orientation) },
    &origin_plan, &expand);

  if (path_found) {
    if (_getPlanFromPath(origin_plan, global_path)) {
      if (!global_path.poses.empty()) {
        global_path.poses.back().pose = goal.pose;
        global_path.poses.back().header = global_path.header;
        global_path.poses.front().pose.orientation = start.pose.orientation;
        global_path.poses.front().header = global_path.header;
      }

      // Visualization
      if (g_planner_->config().expand_zone()) {
        if (planner_type_ == GRAPH_PLANNER && expand_pub_) {
          visualizer->publishExpandZone(expand, costmap_ros_->getCostmap(), expand_pub_,
                                        frame_id_, clock_->now());
        } else if (planner_type_ == SAMPLE_PLANNER && tree_pub_) {
          Visualizer::Lines2d tree_lines;
          for (const auto& node : expand) {
            if (node.theta() != 0) {
              int px_i, py_i;
              double px_d, py_d, x_d, y_d;
              g_planner_->index2Grid(node.theta(), px_i, py_i);
              g_planner_->map2World(px_i, py_i, px_d, py_d);
              g_planner_->map2World(node.x(), node.y(), x_d, y_d);
              tree_lines.emplace_back(
                std::make_pair<common::geometry::Point2d, common::geometry::Point2d>(
                  { x_d, y_d }, { px_d, py_d }));
            }
          }
          visualizer->publishLines2d(tree_lines, tree_pub_, frame_id_, "tree",
                                     clock_->now(), Visualizer::DARK_GREEN, 0.05);
        } else if (planner_type_ == EVOLUTION_PLANNER && particles_pub_) {
          common::geometry::Points2d markers;
          for (const auto& node : expand) {
            double wx, wy;
            g_planner_->map2World(node.x(), node.y(), wx, wy);
            markers.emplace_back(wx, wy);
          }
          visualizer->publishPoints(markers, particles_pub_, frame_id_, "particles",
                                    clock_->now(), Visualizer::DARK_GREEN, 0.1, Visualizer::CUBE);
        }
      }

      if (plan_pub_) {
        visualizer->publishPlan(origin_plan, plan_pub_, frame_id_, clock_->now());
      }
    } else {
      RCLCPP_ERROR(logger_, "Failed to get plan from path.");
    }
  } else {
    RCLCPP_WARN(logger_, "PathPlanner [%s]: Failed to find a valid path.", planner_name_.c_str());
  }

  return global_path;
}

bool PathPlannerNode::_getPlanFromPath(const common::geometry::Points3d& path,
                                       nav_msgs::msg::Path& global_path) {
  global_path.poses.clear();
  global_path.header.stamp = clock_->now();
  global_path.header.frame_id = frame_id_;

  for (size_t i = 0; i < path.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = global_path.header.stamp;
    pose.header.frame_id = frame_id_;
    pose.pose.position.x = path[i].x();
    pose.pose.position.y = path[i].y();
    pose.pose.position.z = 0.0;

    double yaw = path[i].theta();
    if (yaw == 0.0 && i + 1 < path.size()) {
      double dx = path[i + 1].x() - path[i].x();
      double dy = path[i + 1].y() - path[i].y();
      yaw = std::atan2(dy, dx);
    } else if (yaw == 0.0 && i > 0) {
      double dx = path[i].x() - path[i - 1].x();
      double dy = path[i].y() - path[i - 1].y();
      yaw = std::atan2(dy, dx);
    }

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    pose.pose.orientation.x = q.getX();
    pose.pose.orientation.y = q.getY();
    pose.pose.orientation.z = q.getZ();
    pose.pose.orientation.w = q.getW();
    global_path.poses.push_back(pose);
  }

  return !global_path.poses.empty();
}

}  // namespace rmp::path_planner
