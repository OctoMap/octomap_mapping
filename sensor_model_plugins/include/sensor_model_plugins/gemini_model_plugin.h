/*
 * Copyright (C) 2020 Square Robot, Inc - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential
 */

#ifndef SENSOR_MODEL_PLUGINS_INCLUDE_SENSOR_MODEL_PLUGINS_GEMINI_MODEL_PLUGIN_H_
#define SENSOR_MODEL_PLUGINS_INCLUDE_SENSOR_MODEL_PLUGINS_GEMINI_MODEL_PLUGIN_H_

#include <sensor_model_plugins/sensor_model_base.h>
#include <std_msgs/Float32.h>

namespace square_robot {

class GeminiModelPlugin : public SensorModelBase {
private:
  ros::Subscriber fluid_level_sub_;
  std_msgs::Float32 fluid_level_;

public:
  GeminiModelPlugin();
  bool initialize(std::string name) override;
  bool shouldIncludeRay(double x, double y, double z, double intensity = 0) const override;
  std::pair<double, double> getRayProbs(double x, double y, double z, double intensity) const override;

  void onFluidLevel(const std_msgs::Float32::ConstPtr& fluid_level);
};

}  // namespace square_robot

#endif  // SENSOR_MODEL_PLUGINS_INCLUDE_SENSOR_MODEL_PLUGINS_GEMINI_MODEL_PLUGIN_H_
