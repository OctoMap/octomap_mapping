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

#ifndef OCTOMAP_SERVER__TRACKING_OCTOMAP_SERVER_HPP_
#define OCTOMAP_SERVER__TRACKING_OCTOMAP_SERVER_HPP_

#include <string>

#include "octomap_server/octomap_server.hpp"

namespace octomap_server
{
class TrackingOctomapServer : public OctomapServer
{
public:
  explicit TrackingOctomapServer(const rclcpp::NodeOptions & node_options);

  void trackCallback(const PointCloud2::ConstSharedPtr cloud);
  void insertScan(
    const tf2::Vector3 & sensor_origin,
    const PCLPointCloud & ground, const PCLPointCloud & nonground);

protected:
  void trackChanges();

  bool listen_changes_;
  bool track_changes_;
  int min_change_pub_;
  std::string change_id_frame_;
  rclcpp::Publisher<PointCloud2>::SharedPtr pub_change_set_;
  rclcpp::Subscription<PointCloud2>::SharedPtr sub_change_set_;
};

}  // namespace octomap_server

#endif  // OCTOMAP_SERVER__TRACKING_OCTOMAP_SERVER_HPP_
