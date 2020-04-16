/*
 * Copyright (C) 2020 Square Robot, Inc - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential
 */

#include <sensor_model_plugins/me1512_model_plugin.h>

PLUGINLIB_EXPORT_CLASS(square_robot::Me1512ModelPlugin, square_robot::SensorModelBase)

namespace square_robot {
Me1512ModelPlugin::Me1512ModelPlugin() : SensorModelBase(0.65, 0.4, 12) {
}

bool Me1512ModelPlugin::initialize(std::string name) {
  ros::NodeHandle nh("~/" + name);
  nh.param("prob_hit", prob_hit_, prob_hit_);
  nh.param("prob_miss", prob_miss_, prob_miss_);
  nh.param("max_range", max_range_, max_range_);
  return true;
}

bool Me1512ModelPlugin::shouldIncludeRay(double x, double y, double z, double intensity) const {
    return true;
}

std::pair<double, double> Me1512ModelPlugin::getRayProbs(double x, double y, double z, double intensity) const {
    return std::pair<double, double>(prob_hit_, prob_miss_);
}

}  // namespace square_robot
