// Copyright 2012, D. Kuhner, P. Ruchti, University of Freiburg. All rights reserved.
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

#include <octomap_server/tracking_octomap_server.hpp>
#include <string>

namespace octomap_server
{
TrackingOctomapServer::TrackingOctomapServer(const rclcpp::NodeOptions & node_options)
: OctomapServer(node_options)
{
  using std::placeholders::_1;

  const auto change_set_topic = declare_parameter("topic_changes", "changes");
  change_id_frame_ = declare_parameter("change_id_frame", "talker/changes");
  track_changes_ = declare_parameter("track_changes", false);
  listen_changes_ = declare_parameter("listen_changes", false);
  min_change_pub_ = declare_parameter("min_change_pub", 0);

  if (track_changes_ && listen_changes_) {
    RCLCPP_WARN(
      get_logger(),
      "OctoMapServer: It might not be useful to publish changes"
      "and at the same time listen to them."
      "Setting 'track_changes' to false!");
    track_changes_ = false;
  }

  if (track_changes_) {
    RCLCPP_INFO(get_logger(), "starting server");
    pub_change_set_ = create_publisher<PointCloud2>(change_set_topic, 1);
    octree_->enableChangeDetection(true);
  }

  if (listen_changes_) {
    RCLCPP_INFO(get_logger(), "starting client");
    sub_change_set_ = create_subscription<PointCloud2>(
      change_set_topic, 1, std::bind(&TrackingOctomapServer::trackCallback, this, _1));
  }
}

void TrackingOctomapServer::insertScan(
  const tf2::Vector3 & sensor_origin, const PCLPointCloud & ground, const PCLPointCloud & nonground)
{
  OctomapServer::insertScan(sensor_origin, ground, nonground);

  if (track_changes_) {
    trackChanges();
  }
}

void TrackingOctomapServer::trackChanges()
{
  auto start_pnt = octree_->changedKeysBegin();
  auto end_pnt = octree_->changedKeysEnd();

  pcl::PointCloud<pcl::PointXYZI> changed_cells = pcl::PointCloud<pcl::PointXYZI>();

  int c = 0;
  for (auto iter = start_pnt; iter != end_pnt; ++iter) {
    ++c;
    octomap::OcTreeNode * node = octree_->search(iter->first);

    bool occupied = octree_->isNodeOccupied(node);

    octomap::point3d center = octree_->keyToCoord(iter->first);

    pcl::PointXYZI pnt;
    pnt.x = center(0);
    pnt.y = center(1);
    pnt.z = center(2);

    if (occupied) {
      pnt.intensity = 1000;
    } else {
      pnt.intensity = -1000;
    }

    changed_cells.push_back(pnt);
  }

  if (c > min_change_pub_) {
    PointCloud2 changed;
    pcl::toROSMsg(changed_cells, changed);
    changed.header.frame_id = change_id_frame_;
    changed.header.stamp = now();
    pub_change_set_->publish(changed);
    RCLCPP_DEBUG(get_logger(), "[server] sending %zu changed entries", changed_cells.size());

    octree_->resetChangeDetection();
    RCLCPP_DEBUG(
      get_logger(), "[server] octomap size after updating: %zu",
      octree_->calcNumNodes());
  }
}

void TrackingOctomapServer::trackCallback(const PointCloud2::ConstSharedPtr cloud)
{
  pcl::PointCloud<pcl::PointXYZI> cells;
  pcl::fromROSMsg(*cloud, cells);
  RCLCPP_DEBUG(get_logger(), "[client] size of newly occupied cloud: %zu", cells.points.size());

  for (size_t i = 0; i < cells.points.size(); ++i) {
    pcl::PointXYZI & pnt = cells.points[i];
    octree_->updateNode(octree_->coordToKey(pnt.x, pnt.y, pnt.z), pnt.intensity, false);
  }

  octree_->updateInnerOccupancy();
  RCLCPP_DEBUG(get_logger(), "[client] octomap size after updating: %zu", octree_->calcNumNodes());
}

}  // namespace octomap_server

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(octomap_server::TrackingOctomapServer)
