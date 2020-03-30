/*
 * Copyright (C) 2020 Square Robot, Inc - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential
 */

#ifndef SRC_EXTERNAL_OCTOMAP_MAPPING_SENSOR_MODEL_PLUGINS_INCLUDE_SENSOR_MODEL_PLUGINS_ISA500_MODEL_PLUGIN_H_
#define SRC_EXTERNAL_OCTOMAP_MAPPING_SENSOR_MODEL_PLUGINS_INCLUDE_SENSOR_MODEL_PLUGINS_ISA500_MODEL_PLUGIN_H_

#include <sensor_model_plugins/sensor_model_base.h>

namespace square_robot {

class Isa500ModelPlugin : public SensorModelBase {
public:
  Isa500ModelPlugin();
  bool initialize(std::string name) override;
  bool shouldIncludeRay(double x, double y, double z, double intensity = 0) const override;
  std::pair<double, double> getRayProbs(double x, double y, double z, double intensity) const override;
};

}  // namespace square_robot

#endif  // SRC_EXTERNAL_OCTOMAP_MAPPING_SENSOR_MODEL_PLUGINS_INCLUDE_SENSOR_MODEL_PLUGINS_ISA500_MODEL_PLUGIN_H_
