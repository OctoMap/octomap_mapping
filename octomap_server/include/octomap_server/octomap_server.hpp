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

#ifndef OCTOMAP_SERVER__OCTOMAP_SERVER_HPP_
#define OCTOMAP_SERVER__OCTOMAP_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/empty.hpp>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/sample_consensus/method_types.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"  // pcl::SAC_SAMPLE_SIZE is protected since PCL 1.8.0 // NOLINT
#include <pcl/sample_consensus/model_types.h>
#pragma GCC diagnostic pop

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/transform_listener.h>
#include <message_filters/subscriber.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/srv/get_octomap.hpp>
#include <octomap_msgs/srv/bounding_box_query.hpp>
#include <octomap_msgs/conversions.h>

#include <octomap_ros/conversions.hpp>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>

// switch color here - easier maintenance, only maintain OctomapServer.
// Two targets are defined in the cmake, octomap_server_color and octomap_server.
// One has this defined, and the other doesn't
// #define COLOR_OCTOMAP_SERVER

#ifdef COLOR_OCTOMAP_SERVER
#include <octomap/ColorOcTree.h>
#endif

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace octomap_server
{
using nav_msgs::msg::MapMetaData;
using nav_msgs::msg::OccupancyGrid;
using octomap_msgs::msg::Octomap;
using sensor_msgs::msg::PointCloud2;
using std_msgs::msg::ColorRGBA;
using visualization_msgs::msg::MarkerArray;

class OctomapServer : public rclcpp::Node
{
public:
#ifdef COLOR_OCTOMAP_SERVER
  using PCLPoint = pcl::PointXYZRGB;
  using PCLPointCloud = pcl::PointCloud<pcl::PointXYZRGB>;
  using OcTreeT = octomap::ColorOcTree;
#else
  using PCLPoint = pcl::PointXYZ;
  using PCLPointCloud = pcl::PointCloud<pcl::PointXYZ>;
  using OcTreeT = octomap::OcTree;
#endif
  using OctomapSrv = octomap_msgs::srv::GetOctomap;
  using BBoxSrv = octomap_msgs::srv::BoundingBoxQuery;
  using ResetSrv = std_srvs::srv::Empty;

  explicit OctomapServer(const rclcpp::NodeOptions & node_options);
  virtual bool onOctomapBinarySrv(
    const std::shared_ptr<OctomapSrv::Request> req,
    const std::shared_ptr<OctomapSrv::Response> res);
  virtual bool onOctomapFullSrv(
    const std::shared_ptr<OctomapSrv::Request> req,
    const std::shared_ptr<OctomapSrv::Response> res);
  bool clearBBoxSrv(
    const std::shared_ptr<BBoxSrv::Request> req,
    const std::shared_ptr<BBoxSrv::Response> resp);
  bool resetSrv(
    const std::shared_ptr<ResetSrv::Request> req,
    const std::shared_ptr<ResetSrv::Response> resp);

  virtual void insertCloudCallback(const PointCloud2::ConstSharedPtr cloud);
  virtual bool openFile(const std::string & filename);

protected:
  inline static void updateMinKey(const octomap::OcTreeKey & in, octomap::OcTreeKey & min)
  {
    for (size_t i = 0; i < 3; ++i) {
      min[i] = std::min(in[i], min[i]);
    }
  }

  inline static void updateMaxKey(const octomap::OcTreeKey & in, octomap::OcTreeKey & max)
  {
    for (size_t i = 0; i < 3; ++i) {
      max[i] = std::max(in[i], max[i]);
    }
  }

  /// Test if key is within update area of map (2D, ignores height)
  inline bool isInUpdateBBX(const OcTreeT::iterator & it) const
  {
    // 2^(tree_depth-depth) voxels wide:
    unsigned voxelWidth = (1 << (max_tree_depth_ - it.getDepth()));
    octomap::OcTreeKey key = it.getIndexKey();  // lower corner of voxel
    return key[0] + voxelWidth >= update_bbox_min_[0] &&
           key[1] + voxelWidth >= update_bbox_min_[1] &&
           key[0] <= update_bbox_max_[0] &&
           key[1] <= update_bbox_max_[1];
  }

  OnSetParametersCallbackHandle::SharedPtr set_param_res_;
  rcl_interfaces::msg::SetParametersResult onParameter(
    const std::vector<rclcpp::Parameter> & parameters);
  void publishBinaryOctoMap(const rclcpp::Time & rostime) const;
  void publishFullOctoMap(const rclcpp::Time & rostime) const;
  virtual void publishAll(const rclcpp::Time & rostime);

  /**
  * @brief update occupancy map with a scan labeled as ground and nonground.
  * The scans should be in the global map frame.
  *
  * @param sensor_origin origin of the measurements for raycasting
  * @param ground scan endpoints on the ground plane (only clear space)
  * @param nonground all other endpoints (clear up to occupied endpoint)
  */
  virtual void insertScan(
    const tf2::Vector3 & sensor_origin, const PCLPointCloud & ground,
    const PCLPointCloud & nonground);

  /// label the input cloud "pc" into ground and nonground.
  /// Should be in the robot's fixed frame (not world!)
  void filterGroundPlane(
    const PCLPointCloud & pc, PCLPointCloud & ground,
    PCLPointCloud & nonground) const;

  /**
  * @brief Find speckle nodes (single occupied voxels with no neighbors). Only works on lowest resolution!
  * @param key
  * @return
  */
  bool isSpeckleNode(const octomap::OcTreeKey & key) const;

  /// hook that is called before traversing all nodes
  virtual void handlePreNodeTraversal(const rclcpp::Time & rostime);

  /// hook that is called when traversing all nodes of the updated Octree (does nothing here)
  virtual void handleNode([[maybe_unused]] const OcTreeT::iterator & it) {}

  /// hook that is called
  /// when traversing all nodes of the updated Octree in the updated area (does nothing here)
  virtual void handleNodeInBBX([[maybe_unused]] const OcTreeT::iterator & it) {}

  /// hook that is called when traversing occupied nodes of the updated Octree
  virtual void handleOccupiedNode(const OcTreeT::iterator & it);

  /// hook that is called
  /// when traversing occupied nodes in the updated area (updates 2D map projection here)
  virtual void handleOccupiedNodeInBBX(const OcTreeT::iterator & it);

  /// hook that is called when traversing free nodes of the updated Octree
  virtual void handleFreeNode(const OcTreeT::iterator & it);

  /// hook that is called
  /// when traversing free nodes in the updated area (updates 2D map projection here)
  virtual void handleFreeNodeInBBX(const OcTreeT::iterator & it);

  /// hook that is called after traversing all nodes
  virtual void handlePostNodeTraversal(const rclcpp::Time & rostime);

  /// updates the downprojected 2D map as either occupied or free
  virtual void update2DMap(const OcTreeT::iterator & it, bool occupied);

  inline size_t mapIdx(const int i, const int j) const
  {
    return gridmap_.info.width * j + i;
  }

  inline size_t mapIdx(const octomap::OcTreeKey & key) const
  {
    return mapIdx(
      (key[0] - padded_min_key_[0]) / multires_2d_scale_,
      (key[1] - padded_min_key_[1]) / multires_2d_scale_);
  }

  /**
   * Adjust data of map due to a change in its info properties (origin or size,
   * resolution needs to stay fixed). map already contains the new map info,
   * but the data is stored according to old_map_info.
   */

  void adjustMapData(OccupancyGrid & map, const MapMetaData & old_map_info) const;

  inline bool mapChanged(const MapMetaData & old_map_info, const MapMetaData & new_map_info)
  {
    return old_map_info.height != new_map_info.height ||
           old_map_info.width != new_map_info.width ||
           old_map_info.origin.position.x != new_map_info.origin.position.x ||
           old_map_info.origin.position.y != new_map_info.origin.position.y;
  }

  static ColorRGBA heightMapColor(double h);

  rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<Octomap>::SharedPtr binary_map_pub_;
  rclcpp::Publisher<Octomap>::SharedPtr full_map_pub_;
  rclcpp::Publisher<PointCloud2>::SharedPtr point_cloud_pub_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<MarkerArray>::SharedPtr fmarker_pub_;
  message_filters::Subscriber<PointCloud2> point_cloud_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<PointCloud2>> tf_point_cloud_sub_;
  rclcpp::Service<OctomapSrv>::SharedPtr octomap_binary_srv_;
  rclcpp::Service<OctomapSrv>::SharedPtr octomap_full_srv_;
  rclcpp::Service<BBoxSrv>::SharedPtr clear_bbox_srv_;
  rclcpp::Service<ResetSrv>::SharedPtr reset_srv_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  std::unique_ptr<OcTreeT> octree_;
  octomap::KeyRay key_ray_;  // temp storage for ray casting
  octomap::OcTreeKey update_bbox_min_;
  octomap::OcTreeKey update_bbox_max_;

  double max_range_;
  std::string world_frame_id_;  // the map frame
  std::string base_frame_id_;  // base of the robot for ground plane filtering
  bool use_height_map_;
  ColorRGBA color_;
  ColorRGBA color_free_;
  double color_factor_;

  bool latched_topics_;
  bool publish_free_space_;

  double res_;
  size_t tree_depth_;
  size_t max_tree_depth_;

  double point_cloud_min_x_;
  double point_cloud_max_x_;
  double point_cloud_min_y_;
  double point_cloud_max_y_;
  double point_cloud_min_z_;
  double point_cloud_max_z_;
  double occupancy_min_z_;
  double occupancy_max_z_;
  double min_x_size_;
  double min_y_size_;
  bool filter_speckles_;

  bool filter_ground_plane_;
  double ground_filter_distance_;
  double ground_filter_angle_;
  double ground_filter_plane_distance_;

  bool compress_map_;

  bool init_config_;

  // downprojected 2D map:
  bool incremental_2D_projection_;
  OccupancyGrid gridmap_;
  bool publish_2d_map_;
  bool map_origin_changed;
  octomap::OcTreeKey padded_min_key_;
  unsigned multires_2d_scale_;
  bool project_complete_map_;
  bool use_colored_map_;
};
}  // namespace octomap_server

#endif  // OCTOMAP_SERVER__OCTOMAP_SERVER_HPP_
