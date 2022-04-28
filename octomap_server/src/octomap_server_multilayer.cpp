// Copyright 2011-2013, A. Hornung, M. Philips. All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Willow Garage nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"

#include "octomap_server/octomap_server_multilayer.hpp"

namespace octomap_server
{
OctomapServerMultilayer::OctomapServerMultilayer(const rclcpp::NodeOptions & node_options)
: OctomapServer(node_options)
{
  // TODO(someone): callback for arm_navigation attached objects was removed, is
  // there a replacement functionality?

  // TODO(someone): param maps, limits
  // right now 0: base, 1: spine, 2: arms
  ProjectedMap m;
  m.name = "projected_base_map";
  m.min_z = 0.0;
  m.max_z = 0.3;
  m.z = 0.0;
  multi_gridmap_.push_back(m);

  m.name = "projected_spine_map";
  m.min_z = 0.25;
  m.max_z = 1.4;
  m.z = 0.6;
  multi_gridmap_.push_back(m);

  m.name = "projected_arm_map";
  m.min_z = 0.7;
  m.max_z = 0.9;
  m.z = 0.8;
  multi_gridmap_.push_back(m);

  const auto qos = latched_topics_ ? rclcpp::QoS{5}.transient_local() : rclcpp::QoS{5};

  for (size_t i = 0; i < multi_gridmap_.size(); ++i) {
    multi_map_pub_.push_back(create_publisher<OccupancyGrid>(multi_gridmap_.at(i).name, qos));
  }

  // init arm links (could be params as well)
  arm_links_.push_back("l_elbow_flex_link");
  arm_link_offsets_.push_back(0.10);
  arm_links_.push_back("l_gripper_l_finger_tip_link");
  arm_link_offsets_.push_back(0.03);
  arm_links_.push_back("l_gripper_r_finger_tip_link");
  arm_link_offsets_.push_back(0.03);
  arm_links_.push_back("l_upper_arm_roll_link");
  arm_link_offsets_.push_back(0.16);
  arm_links_.push_back("l_wrist_flex_link");
  arm_link_offsets_.push_back(0.05);
  arm_links_.push_back("r_elbow_flex_link");
  arm_link_offsets_.push_back(0.10);
  arm_links_.push_back("r_gripper_l_finger_tip_link");
  arm_link_offsets_.push_back(0.03);
  arm_links_.push_back("r_gripper_r_finger_tip_link");
  arm_link_offsets_.push_back(0.03);
  arm_links_.push_back("r_upper_arm_roll_link");
  arm_link_offsets_.push_back(0.16);
  arm_links_.push_back("r_wrist_flex_link");
  arm_link_offsets_.push_back(0.05);
}

void OctomapServerMultilayer::handlePreNodeTraversal(const rclcpp::Time & rostime)
{
  // multilayer server always publishes 2D maps:
  publish_2d_map_ = true;
  MapMetaData gridmap_info = gridmap_.info;

  OctomapServer::handlePreNodeTraversal(rostime);


  // recalculate height of arm layer (stub, TODO)
  geometry_msgs::msg::PointStamped vin;
  vin.point.x = 0;
  vin.point.y = 0;
  vin.point.z = 0;
  vin.header.stamp = rostime;
  double link_padding = 0.03;

  double min_arm_height = 2.0;
  double max_arm_height = 0.0;

  for (size_t i = 0; i < arm_links_.size(); ++i) {
    vin.header.frame_id = arm_links_[i];
    geometry_msgs::msg::PointStamped vout;
    geometry_msgs::msg::TransformStamped transform_stamped;
    try {
      transform_stamped = tf2_buffer_->lookupTransform(
        "base_footprint", arm_links_.at(i), rclcpp::Time(0),
        rclcpp::Duration::from_seconds(1.0));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "%s", ex.what());
      return;
    }
    tf2::doTransform(vin, vout, transform_stamped);
    max_arm_height = std::max(
      max_arm_height, vout.point.z + (arm_link_offsets_.at(i) + link_padding));
    min_arm_height = std::min(
      min_arm_height, vout.point.z - (arm_link_offsets_.at(i) + link_padding));
  }
  RCLCPP_INFO(
    get_logger(), "Arm layer interval adjusted to (%f,%f)", min_arm_height,
    max_arm_height);
  multi_gridmap_.at(2).min_z = min_arm_height;
  multi_gridmap_.at(2).max_z = max_arm_height;
  multi_gridmap_.at(2).z = (max_arm_height + min_arm_height) / 2.0;


  // TODO(someone): also clear multilevel maps in BBX region (see OctomapServer.cpp)?

  bool map_info_changed = mapChanged(gridmap_info, gridmap_.info);

  for (MultilevelGrid::iterator it = multi_gridmap_.begin(); it != multi_gridmap_.end(); ++it) {
    it->map.header = gridmap_.header;
    it->map.info = gridmap_.info;
    it->map.info.origin.position.z = it->z;
    if (project_complete_map_) {
      RCLCPP_INFO(get_logger(), "Map resolution changed, rebuilding complete 2D maps");
      it->map.data.clear();
      // init to unknown:
      it->map.data.resize(it->map.info.width * it->map.info.height, -1);
    } else if (map_info_changed) {
      adjustMapData(it->map, gridmap_info);
    }
  }
}

void OctomapServerMultilayer::handlePostNodeTraversal(const rclcpp::Time & rostime)
{
// TODO(someone): calc tall / short obs. cells for arm layer, => temp arm layer
//  std::vector<int> shortObsCells;
//  for(size_t int i=0; i<arm_map.data.size(); i++){
//    if(temp_arm_map.data[i] == 0){
//      if(map.data[i] == -1)
//        arm_map.data[i] = -1;
//    }
//    else if(arm_map.data[i] == 0)
//      arm_map.data[i] = 0;
//    else if(double(arm_map.data[i])/temp_arm_map.data[i] > 0.8)
//      arm_map.data[i] = 101;
//    else{
//      arm_map.data[i] = 100;
//      shortObsCells.push_back(i);
//    }
//  }
//
//  std::vector<int> tallObsCells;
//  tallObsCells.reserve(shortObsCells.size());
//  int dxy[8] = { - arm_map.info.width - 1, -arm_map.info.width, - arm_map.info.width + 1, -1,
//                 1, arm_map.info.width - 1, arm_map.info.width, arm_map.info.width + 1 };
//  for(size_t int i=0; i<shortObsCells.size(); i++){
//    for(int j=0; j<8; j++){
//      int temp = shortObsCells[i]+dxy[j];
//      if(temp<0 || temp>=arm_map.data.size())
//        continue;
//      if(arm_map.data[temp]==101){
//        tallObsCells.push_back(shortObsCells[i]);
//        break;
//      }
//    }
//  }
//  for(size_t int i=0; i<tallObsCells.size(); i++)
//    arm_map.data[tallObsCells[i]] = 101;

  OctomapServer::handlePostNodeTraversal(rostime);

  for (size_t i = 0; i < multi_map_pub_.size(); ++i) {
    multi_map_pub_[i]->publish(multi_gridmap_.at(i).map);
  }
}

void OctomapServerMultilayer::update2DMap(const OcTreeT::iterator & it, bool occupied)
{
  double z = it.getZ();
  double s2 = it.getSize() / 2.0;

  // create a mask on which maps to update:
  std::vector<bool> inMapLevel(multi_gridmap_.size(), false);
  for (size_t i = 0; i < multi_gridmap_.size(); ++i) {
    if (z + s2 >= multi_gridmap_[i].min_z && z - s2 <= multi_gridmap_[i].max_z) {
      inMapLevel[i] = true;
    }
  }

  if (it.getDepth() == max_tree_depth_) {
    size_t idx = mapIdx(it.getKey());
    if (occupied) {
      gridmap_.data[idx] = 100;
    } else if (gridmap_.data[idx] == -1) {
      gridmap_.data[idx] = 0;
    }

    for (size_t i = 0; i < inMapLevel.size(); ++i) {
      if (inMapLevel[i]) {
        if (occupied) {
          multi_gridmap_[i].map.data[idx] = 100;
        } else if (multi_gridmap_[i].map.data[idx] == -1) {
          multi_gridmap_[i].map.data[idx] = 0;
        }
      }
    }

  } else {
    int int_size = 1 << (tree_depth_ - it.getDepth());
    octomap::OcTreeKey min_key = it.getIndexKey();
    for (int dx = 0; dx < int_size; dx++) {
      int i = (min_key[0] + dx - padded_min_key_[0]) / multires_2d_scale_;
      for (int dy = 0; dy < int_size; dy++) {
        size_t idx = mapIdx(i, (min_key[1] + dy - padded_min_key_[1]) / multires_2d_scale_);
        if (occupied) {
          gridmap_.data[idx] = 100;
        } else if (gridmap_.data[idx] == -1) {
          gridmap_.data[idx] = 0;
        }

        for (size_t i = 0; i < inMapLevel.size(); ++i) {
          if (inMapLevel[i]) {
            if (occupied) {
              multi_gridmap_[i].map.data[idx] = 100;
            } else if (multi_gridmap_[i].map.data[idx] == -1) {
              multi_gridmap_[i].map.data[idx] = 0;
            }
          }
        }
      }
    }
  }
}

}  // namespace octomap_server

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(octomap_server::OctomapServerMultilayer)
