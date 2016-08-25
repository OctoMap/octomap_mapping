#ifndef OCTOMAP_SERVER_LABELOCTOMAPSERVER_H
#define OCTOMAP_SERVER_LABELOCTOMAPSERVER_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/ColorRGBA.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <dynamic_reconfigure/server.h>
#include <octomap_server/OctomapServerConfig.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>

#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>

#ifdef COLOR_OCTOMAP_SERVER
#include <octomap/ColorOcTree.h>
#endif  // COLOR_OCTOMAP_SERVER

#include <algorithm>
#include <string>

namespace octomap_server
{

class LabelOctomapServer
{
public:
#ifdef COLOR_OCTOMAP_SERVER
  typedef pcl::PointXYZRGB PCLPoint;
  typedef pcl::PointCloud<pcl::PointXYZRGB> PCLPointCloud;
  typedef octomap::ColorOcTree OcTreeT;
#else
  typedef pcl::PointXYZ PCLPoint;
  typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;
  typedef octomap::OcTree OcTreeT;
#endif
  typedef octomap_msgs::GetOctomap OctomapSrv;
  typedef octomap_msgs::BoundingBoxQuery BBXSrv;

  typedef message_filters::sync_policies::ApproximateTime <sensor_msgs::PointCloud2, sensor_msgs::Image> ApproximateSyncPolicy;

  LabelOctomapServer();
  virtual ~LabelOctomapServer();
  virtual bool octomapBinarySrv(OctomapSrv::Request  &req, OctomapSrv::GetOctomap::Response &res);
  virtual bool octomapFullSrv(OctomapSrv::Request  &req, OctomapSrv::GetOctomap::Response &res);
  bool clearBBXSrv(BBXSrv::Request& req, BBXSrv::Response& resp);
  bool resetSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);

  virtual void insertCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud, const sensor_msgs::Image::ConstPtr& imgmsg);
  virtual bool openFile(const std::string& filename);

protected:
  inline static void updateMinKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& min)
  {
    for (unsigned i = 0; i < 3; ++i)
    {
      min[i] = std::min(in[i], min[i]);
    }
  };

  inline static void updateMaxKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& max)
  {
    for (unsigned i = 0; i < 3; ++i)
    {
      max[i] = std::max(in[i], max[i]);
    }
  };

  /// Test if key is within update area of map (2D, ignores height)
  inline bool isInUpdateBBX(const OcTreeT::iterator& it) const
  {
    // 2^(tree_depth-depth) voxels wide:
    unsigned voxel_width = (1 << (max_tree_depth_ - it.getDepth()));
    octomap::OcTreeKey key = it.getIndexKey(); // lower corner of voxel
    return (key[0] + voxel_width >= update_bbx_min_[0] &&
            key[1] + voxel_width >= update_bbx_min_[1] &&
            key[0] <= update_bbx_max_[0] &&
            key[1] <= update_bbx_max_[1]);
  }

  void periodicalPublishCallback(const ros::TimerEvent& event);
  void reconfigureCallback(octomap_server::OctomapServerConfig& config, uint32_t level);
  void publishBinaryOctoMap(const ros::Time& rostime = ros::Time::now()) const;
  void publishFullOctoMap(const ros::Time& rostime = ros::Time::now()) const;
  virtual void publishAll(const ros::Time& rostime = ros::Time::now());

  /**
    * @brief update occupancy map with a scan labeled as ground and nonground.
    * The scans should be in the global map frame.
    *
    * @param ground scan endpoints on the ground plane (only clear space)
    * @param nonground all other endpoints (clear up to occupied endpoint)
    */
  virtual void insertScan(const sensor_msgs::PointCloud2::ConstPtr& cloud,
                          const sensor_msgs::Image::ConstPtr& imgmsg);

  /**
    * @brief Find speckle nodes (single occupied voxels with no neighbors). Only works on lowest resolution!
    * @param key
    * @return
    */
  bool isSpeckleNode(const octomap::OcTreeKey& key) const;

  /// hook that is called before traversing all nodes
  virtual void handlePreNodeTraversal(const ros::Time& rostime);

  /// hook that is called when traversing all nodes of the updated Octree (does nothing here)
  virtual void handleNode(const OcTreeT::iterator& it)
  {
  };

  /// hook that is called when traversing all nodes of the updated Octree in the updated area (does nothing here)
  virtual void handleNodeInBBX(const OcTreeT::iterator& it)
  {
  };

  /// hook that is called when traversing occupied nodes of the updated Octree
  virtual void handleOccupiedNode(const OcTreeT::iterator& it);

  /// hook that is called when traversing occupied nodes in the updated area (updates 2D map projection here)
  virtual void handleOccupiedNodeInBBX(const OcTreeT::iterator& it);

  /// hook that is called when traversing free nodes of the updated Octree
  virtual void handleFreeNode(const OcTreeT::iterator& it);

  /// hook that is called when traversing free nodes in the updated area (updates 2D map projection here)
  virtual void handleFreeNodeInBBX(const OcTreeT::iterator& it);

  /// hook that is called after traversing all nodes
  virtual void handlePostNodeTraversal(const ros::Time& rostime);

  /// updates the downprojected 2D map as either occupied or free
  virtual void update2DMap(const OcTreeT::iterator& it, bool occupied);

  inline unsigned mapIdx(int i, int j) const
  {
    return gridmap_.info.width * j + i;
  }

  inline unsigned mapIdx(const octomap::OcTreeKey& key) const
  {
    return mapIdx((key[0] - padded_min_key_[0]) / multires_2d_scale_,
                  (key[1] - padded_min_key_[1]) / multires_2d_scale_);
  }

  /**
    * Adjust data of map due to a change in its info properties (origin or size,
    * resolution needs to stay fixed). map already contains the new map info,
    * but the data is stored according to old_map_info.
    */
  void adjustMapData(nav_msgs::OccupancyGrid& map, const nav_msgs::MapMetaData& old_map_info) const;

  inline bool mapChanged(const nav_msgs::MapMetaData& old_map_info, const nav_msgs::MapMetaData& new_map_info)
  {
    return (old_map_info.height != new_map_info.height ||
            old_map_info.width != new_map_info.width ||
            old_map_info.origin.position.x != new_map_info.origin.position.x ||
            old_map_info.origin.position.y != new_map_info.origin.position.y);
  }

  static std_msgs::ColorRGBA heightMapColor(double h);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pub_marker_;
  ros::Publisher pub_binary_map_;
  ros::Publisher pub_full_map_;
  ros::Publisher pub_point_cloud_;
  ros::Publisher pub_map_;
  ros::Publisher pub_fmarker_;
  message_filters::Subscriber<sensor_msgs::PointCloud2>* sub_point_cloud_;
  message_filters::Subscriber<sensor_msgs::Image>* sub_obj_proba_img_;
  ros::ServiceServer srv_octomap_binary_;
  ros::ServiceServer srv_octomap_full_;
  ros::ServiceServer srv_clear_bbx_;
  ros::ServiceServer srv_reset_;
  tf::TransformListener tf_listener_;
  boost::recursive_mutex config_mutex_;
  dynamic_reconfigure::Server<OctomapServerConfig> reconfigure_server_;

  OcTreeT* octree_;
  octomap::KeyRay key_ray_;  // temp storage for ray casting
  octomap::OcTreeKey update_bbx_min_;
  octomap::OcTreeKey update_bbx_max_;

  double max_range_;
  std::string world_frame_id_;  // the map frame
  bool use_height_map_;
  std_msgs::ColorRGBA color_;
  std_msgs::ColorRGBA color_free_;
  double color_factor_;

  bool latched_topics_;
  bool publish_free_space_;
  double publish_rate_;

  ros::Timer timer_periodical_publish_;

  double resolution_;
  unsigned tree_depth_;
  unsigned max_tree_depth_;

  double occupancy_min_z_;
  double occupancy_max_z_;
  double min_size_x_;
  double min_size_y_;
  bool filter_speckles_;

  message_filters::Synchronizer<ApproximateSyncPolicy>* async_;

  bool compress_map_;

  bool init_config_;

  // downprojected 2D map:
  bool incremental_update_;
  nav_msgs::OccupancyGrid gridmap_;
  bool publish_2d_map_;
  octomap::OcTreeKey padded_min_key_;
  unsigned multires_2d_scale_;
  bool project_complete_map_;
  bool use_colored_map_;
};

}  // namespace octomap_server

#endif  // OCTOMAP_SERVER_LABELOCTOMAPSERVER_H
