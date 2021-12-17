// Copyright 2013, A. Hornung, University of Freiburg. All rights reserved.
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

#include <octomap_server/octomap_server_static.hpp>

#include <octomap_msgs/conversions.h>

#include <memory>
#include <string>
#include <utility>

namespace octomap_server
{
OctomapServerStatic::OctomapServerStatic(const rclcpp::NodeOptions & node_options)
: Node("octomap_server_static", node_options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  frame_id_ = declare_parameter("frame_id", "map");
  const auto filename = declare_parameter("octomap_path", "");
  if (filename.length() <= 3) {
    RCLCPP_ERROR(get_logger(), "Octree file does not have .ot extension");
    return;
  }
  const auto suffix = filename.substr(filename.length() - 3, 3);

  if (suffix == ".bt") {
    octree_ = std::make_unique<octomap::OcTree>(filename);
  } else if (suffix == ".ot") {
    std::unique_ptr<octomap::AbstractOcTree> tree{octomap::AbstractOcTree::read(filename)};
    if (!tree) {
      RCLCPP_ERROR(get_logger(), "Could not read octree from file");
      return;
    }

    octree_ = std::unique_ptr<octomap::AbstractOccupancyOcTree>(
      dynamic_cast<octomap::AbstractOccupancyOcTree *>(tree.release()));
  } else {
    RCLCPP_ERROR(get_logger(), "Octree file does not have .bt or .ot extension");
    return;
  }
  octomap_binary_srv_ = create_service<GetOctomap>(
    "octomap_binary", std::bind(&OctomapServerStatic::onOctomapBinarySrv, this, _1, _2));
  octomap_full_srv_ = create_service<GetOctomap>(
    "octomap_full", std::bind(&OctomapServerStatic::onOctomapFullSrv, this, _1, _2));
}

bool OctomapServerStatic::onOctomapBinarySrv(
  [[maybe_unused]] const std::shared_ptr<GetOctomap::Request> req,
  const std::shared_ptr<GetOctomap::Response> res)
{
  RCLCPP_INFO(get_logger(), "Sending binary map data on service request");
  res->map.header.frame_id = frame_id_;
  res->map.header.stamp = now();
  if (!octomap_msgs::binaryMapToMsg(*octree_, res->map)) {
    return false;
  }
  return true;
}

bool OctomapServerStatic::onOctomapFullSrv(
  [[maybe_unused]] const std::shared_ptr<GetOctomap::Request> req,
  const std::shared_ptr<GetOctomap::Response> res)
{
  RCLCPP_INFO(get_logger(), "Sending full map data on service request");
  res->map.header.frame_id = frame_id_;
  res->map.header.stamp = now();

  if (!octomap_msgs::fullMapToMsg(*octree_, res->map)) {
    return false;
  }

  return true;
}
}  // namespace octomap_server

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(octomap_server::OctomapServerStatic)
