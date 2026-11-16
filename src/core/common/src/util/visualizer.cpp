/**
 * *********************************************************
 *
 * @file: visualizer.cpp
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
#include "common/util/visualizer.h"

namespace rmp::common::util {
std_msgs::msg::ColorRGBA Visualizer::_colorInit(double r, double g, double b, double a) {
  std_msgs::msg::ColorRGBA color;
  color.r = static_cast<float>(r);
  color.g = static_cast<float>(g);
  color.b = static_cast<float>(b);
  color.a = static_cast<float>(a);
  return color;
}

std_msgs::msg::ColorRGBA Visualizer::RED = Visualizer::_colorInit(1.0, 0.0, 0.0, 1.0);
std_msgs::msg::ColorRGBA Visualizer::DARK_GREEN = Visualizer::_colorInit(0.43, 0.54, 0.24, 0.5);
std_msgs::msg::ColorRGBA Visualizer::PURPLE = Visualizer::_colorInit(1.0, 0.0, 1.0, 1.0);
std_msgs::msg::ColorRGBA Visualizer::LIGHT_PURPLE = Visualizer::_colorInit(0.7, 0.4, 0.9, 0.8);
std_msgs::msg::ColorRGBA Visualizer::LIGHT_ORANGE = Visualizer::_colorInit(1.0, 0.65, 0.0, 0.8);

}  // namespace rmp::common::util
