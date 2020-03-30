/*
 * Copyright (C) 2020 Square Robot, Inc - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential
 */

#include <sensor_model_plugins/gemini_model_plugin.h>

PLUGINLIB_EXPORT_CLASS(square_robot::GeminiModelPlugin, square_robot::SensorModelBase)

namespace square_robot {
GeminiModelPlugin::GeminiModelPlugin() : SensorModelBase(0.65, 0.3, 20) {
}

bool GeminiModelPlugin::initialize(std::string name) {
  ros::NodeHandle nh("~/" + name);
  nh.param("prob_hit", prob_hit_, prob_hit_);
  nh.param("prob_miss", prob_miss_, prob_miss_);
  nh.param("max_range", max_range_, max_range_);
  std::string tank_info_topic = "tank_info";
  nh.param("tank_info_topic", tank_info_topic, tank_info_topic);

  ros::NodeHandle base_nh;
  tank_info_sub_ = base_nh.subscribe(tank_info_topic, 1, &GeminiModelPlugin::onTankInfo, this);
  return true;
}

void GeminiModelPlugin::onTankInfo(const utbot_msgs::TankInfo::ConstPtr& tank_info) {
  tank_info_ = *tank_info;
}

bool GeminiModelPlugin::shouldIncludeRay(double x, double y, double z, double intensity) const {
  return z < tank_info_.product_depth;
}

std::pair<double, double> GeminiModelPlugin::getRayProbs(double x, double y, double z, double intensity) const {
  return std::pair<double, double>(prob_hit_, prob_miss_);
}

}  // namespace square_robot
