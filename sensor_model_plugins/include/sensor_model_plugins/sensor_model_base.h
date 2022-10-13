/*
 * Copyright (C) 2020 Square Robot, Inc - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited.
 * Proprietary and confidential
 */

#ifndef SENSOR_MODEL_PLUGINS_INCLUDE_SENSOR_MODEL_PLUGINS_SENSOR_MODEL_BASE_H_
#define SENSOR_MODEL_PLUGINS_INCLUDE_SENSOR_MODEL_PLUGINS_SENSOR_MODEL_BASE_H_

#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <utility>

namespace square_robot {

class SensorModelBase {
protected:
  double prob_hit_;
  double prob_miss_;
  
public:
  double max_range_;
  double min_range_{0};

  SensorModelBase();

  SensorModelBase(double prob_hit, double prob_miss, double max_range) {
    prob_hit_ = prob_hit;
    prob_miss_ = prob_miss;
    max_range_ = max_range;
  }

  /**
   *  Initializes the plugin.
   *  - nh: the node handle of the parent node.
   */
  virtual bool initialize(std::string name) = 0;

  /**
   *  returns true if the ray represented by the given endpoint should be included in
   *  the map update.
   *  - x, y, z: the position of the ray endpoint
   *  - intensity (optional): the intensity of the return reported by the sensor
   */
  virtual bool shouldIncludeRay(double x, double y, double z, double intensity = 0) const = 0;

  /**
   *  Returns the probabilities of the ray associated with the given endpoint.
   *  - x, y, z: the position of the ray endpoint
   *  - intensity (optional): the intensity of the return reported by the sensor
   *
   *  result pair<prob_hit, prob_miss> (probabilities in range [0 - 1.0]) where
   *      - prob_hit is the probability that the endpoint is occupied space given the measurement
   *      - prob_miss is the probability that the cells along the ray to the endpoint are freespace given the
   *        measurement
   */
  virtual std::pair<double, double> getRayProbs(double x, double y, double z, double intensity = 0) const = 0;
};

}  // namespace square_robot
#endif  // SENSOR_MODEL_PLUGINS_INCLUDE_SENSOR_MODEL_PLUGINS_SENSOR_MODEL_BASE_H_
