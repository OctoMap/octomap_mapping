/*
 * Copyright (C) 2020 Square Robot, Inc - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential
 */

#ifndef SRC_EXTERNAL_OCTOMAP_MAPPING_SENSOR_MODEL_PLUGINS_INCLUDE_SENSOR_MODEL_PLUGINS_GEMINI_MODEL_PLUGIN_H_
#define SRC_EXTERNAL_OCTOMAP_MAPPING_SENSOR_MODEL_PLUGINS_INCLUDE_SENSOR_MODEL_PLUGINS_GEMINI_MODEL_PLUGIN_H_

#include <sensor_model_plugins/sensor_model_base.h>
#include <utbot_msgs/TankInfo.h>

namespace square_robot {

class GeminiModelPlugin : public SensorModelBase {
private:
  ros::Subscriber tank_info_sub_;
  utbot_msgs::TankInfo tank_info_;

public:
  GeminiModelPlugin();
  bool initialize(std::string name) override;
  bool shouldIncludeRay(double x, double y, double z, double intensity = 0) const override;
  std::pair<double, double> getRayProbs(double x, double y, double z, double intensity) const override;

  void onTankInfo(const utbot_msgs::TankInfo::ConstPtr& tank_info);
};

}  // namespace square_robot

#endif  // SRC_EXTERNAL_OCTOMAP_MAPPING_SENSOR_MODEL_PLUGINS_INCLUDE_SENSOR_MODEL_PLUGINS_GEMINI_MODEL_PLUGIN_H_
