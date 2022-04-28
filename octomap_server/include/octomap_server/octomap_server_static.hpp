// Copyright 2021, Daisuke Nishimatsu. All rights reserved.
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

#ifndef OCTOMAP_SERVER__OCTOMAP_SERVER_STATIC_HPP_
#define OCTOMAP_SERVER__OCTOMAP_SERVER_STATIC_HPP_

#include <octomap/octomap.h>

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "octomap_msgs/srv/get_octomap.hpp"

namespace octomap_server
{
using octomap_msgs::srv::GetOctomap;

class OctomapServerStatic : public rclcpp::Node
{
public:
  explicit OctomapServerStatic(const rclcpp::NodeOptions & node_options);

private:
  bool onOctomapBinarySrv(
    const std::shared_ptr<GetOctomap::Request> req,
    const std::shared_ptr<GetOctomap::Response> res);
  bool onOctomapFullSrv(
    const std::shared_ptr<GetOctomap::Request> req,
    const std::shared_ptr<GetOctomap::Response> res);

  rclcpp::Service<GetOctomap>::SharedPtr octomap_binary_srv_;
  rclcpp::Service<GetOctomap>::SharedPtr octomap_full_srv_;
  std::string frame_id_;
  std::unique_ptr<octomap::AbstractOccupancyOcTree> octree_;
};
}  // namespace octomap_server

#endif  // OCTOMAP_SERVER__OCTOMAP_SERVER_STATIC_HPP_
