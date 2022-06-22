// Copyright 2010-2013, A. Hornung, University of Freiburg. All rights reserved.
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

#include <octomap/octomap.h>

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "octomap_msgs/conversions.h"
#include "octomap_msgs/srv/get_octomap.hpp"

namespace octomap_server
{
using octomap::AbstractOcTree;
using octomap::AbstractOccupancyOcTree;
using octomap_msgs::srv::GetOctomap;

class OctomapSaver : public rclcpp::Node
{
public:
  explicit OctomapSaver(const rclcpp::NodeOptions & node_options);
};

OctomapSaver::OctomapSaver(
  const rclcpp::NodeOptions & node_options)
: rclcpp::Node("octomap_saver", node_options)
{
  using std::chrono_literals::operator""s;

  const bool full = declare_parameter("full", false);
  const auto map_name = declare_parameter("octomap_path", "");
  if (map_name.length() < 4) {
    RCLCPP_ERROR_STREAM(get_logger(), "Invalid file name or extension: " << map_name);
    rclcpp::shutdown();
    return;
  }
  const auto srv_name = full ? "octomap_full" : "octomap_binary";
  auto client = create_client<GetOctomap>(srv_name);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for service.");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(get_logger(), "Waiting for service...");
  }

  auto request =
    std::make_shared<GetOctomap::Request>();
  auto response = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(
      get_node_base_interface(),
      response) == rclcpp::FutureReturnCode::SUCCESS)
  {
    std::unique_ptr<AbstractOcTree> tree{octomap_msgs::msgToMap(response.get()->map)};
    std::unique_ptr<AbstractOccupancyOcTree> octree;
    if (tree) {
      octree =
        std::unique_ptr<AbstractOccupancyOcTree>(
        dynamic_cast<AbstractOccupancyOcTree *>(tree.
        release()));
    } else {
      RCLCPP_ERROR(get_logger(), "Error creating octree from received message");
      RCLCPP_WARN_EXPRESSION(
        get_logger(), response.get()->map.id == "ColorOcTree",
        "You requested a binary map for a ColorOcTree - this is currently not supported. "
        "Please add -f to request a full map"
      );
    }

    if (octree) {
      RCLCPP_INFO(
        get_logger(),
        "Map received (%zu nodes, %f m res), saving to %s",
        octree->size(), octree->getResolution(), map_name.c_str()
      );
      std::string suffix = map_name.substr(map_name.length() - 3, 3);
      if (suffix == ".bt") {  // write to binary file:
        if (!octree->writeBinary(map_name)) {
          RCLCPP_ERROR(get_logger(), "Error writing to file %s", map_name.c_str());
        }
      } else if (suffix == ".ot") {  // write to full .ot file:
        if (!octree->write(map_name)) {
          RCLCPP_ERROR(get_logger(), "Error writing to file %s", map_name.c_str());
        }
      } else {
        RCLCPP_ERROR(get_logger(), "Unknown file extension, must be either .bt or .ot");
      }
    } else {
      RCLCPP_ERROR(get_logger(), "Error reading OcTree from stream");
    }
  } else {
    RCLCPP_ERROR(get_logger(), "Problem while waiting for response.");
  }

  rclcpp::shutdown();
}
}  // namespace octomap_server

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(octomap_server::OctomapSaver)
