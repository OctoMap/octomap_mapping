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

#include <octomap_server/octomap_server.hpp>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace
{
template<typename T>
bool update_param(const std::vector<rclcpp::Parameter> & p, const std::string & name, T & value)
{
  auto it = std::find_if(
    p.cbegin(), p.cend(), [&name](const rclcpp::Parameter & parameter) {
      return parameter.get_name() == name;
    });
  if (it != p.cend()) {
    value = it->template get_value<T>();
    return true;
  }
  return false;
}
}  // namespace
namespace octomap_server
{
OctomapServer::OctomapServer(const rclcpp::NodeOptions & node_options)
: Node("octomap_server", node_options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  world_frame_id_ = declare_parameter("frame_id", "map");
  base_frame_id_ = declare_parameter("base_frame_id", "base_footprint");
  use_height_map_ = declare_parameter("use_height_map", false);
  use_colored_map_ = declare_parameter("colored_map", false);
  color_factor_ = declare_parameter("color_factor", 0.8);

  point_cloud_min_x_ = declare_parameter("point_cloud_min_x", -std::numeric_limits<double>::max());
  point_cloud_max_x_ = declare_parameter("point_cloud_max_x", std::numeric_limits<double>::max());
  point_cloud_min_y_ = declare_parameter("point_cloud_min_y", -std::numeric_limits<double>::max());
  point_cloud_max_y_ = declare_parameter("point_cloud_max_y", std::numeric_limits<double>::max());
  {
    rcl_interfaces::msg::ParameterDescriptor point_cloud_min_z_desc;
    point_cloud_min_z_desc.description = "Minimum height of points to consider for insertion";
    rcl_interfaces::msg::FloatingPointRange point_cloud_min_z_range;
    point_cloud_min_z_range.from_value = -100.0;
    point_cloud_min_z_range.to_value = 100.0;
    point_cloud_min_z_desc.floating_point_range.push_back(point_cloud_min_z_range);
    point_cloud_min_z_ = declare_parameter("point_cloud_min_z", -100.0, point_cloud_min_z_desc);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor point_cloud_max_z_desc;
    point_cloud_max_z_desc.description = "Maximum height of points to consider for insertion";
    rcl_interfaces::msg::FloatingPointRange point_cloud_max_z_range;
    point_cloud_max_z_range.from_value = -100.0;
    point_cloud_max_z_range.to_value = 100.0;
    point_cloud_max_z_desc.floating_point_range.push_back(point_cloud_max_z_range);
    point_cloud_max_z_ = declare_parameter("point_cloud_max_z", 100.0, point_cloud_max_z_desc);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor occupancy_min_z_desc;
    occupancy_min_z_desc.description =
      "Minimum height of occupied cells to consider in the final map";
    rcl_interfaces::msg::FloatingPointRange occupancy_min_z_range;
    occupancy_min_z_range.from_value = -100.0;
    occupancy_min_z_range.to_value = 100.0;
    occupancy_min_z_desc.floating_point_range.push_back(occupancy_min_z_range);
    occupancy_min_z_ = declare_parameter("occupancy_min_z", -100.0, occupancy_min_z_desc);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor occupancy_max_z_desc;
    occupancy_max_z_desc.description =
      "Maximum height of occupied cells to consider in the final map";
    rcl_interfaces::msg::FloatingPointRange occupancy_max_z_range;
    occupancy_max_z_range.from_value = -100.0;
    occupancy_max_z_range.to_value = 100.0;
    occupancy_max_z_desc.floating_point_range.push_back(occupancy_max_z_range);
    occupancy_max_z_ = declare_parameter("occupancy_max_z", 100.0, occupancy_max_z_desc);
  }
  min_x_size_ = declare_parameter("min_x_size", 0.0);
  min_y_size_ = declare_parameter("min_y_size", 0.0);

  {
    rcl_interfaces::msg::ParameterDescriptor filter_speckles_desc;
    filter_speckles_desc.description = "Filter speckle nodes (with no neighbors)";
    filter_speckles_ = declare_parameter("filter_speckles", false, filter_speckles_desc);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor filter_ground_plane_desc;
    filter_ground_plane_desc.description = "Filter ground plane";
    filter_ground_plane_ =
      declare_parameter("filter_ground_plane", false, filter_ground_plane_desc);
  }
  {
    // distance of points from plane for RANSAC
    rcl_interfaces::msg::ParameterDescriptor ground_filter_distance_desc;
    ground_filter_distance_desc.description =
      "Distance threshold to consider a point as ground";
    rcl_interfaces::msg::FloatingPointRange ground_filter_distance_range;
    ground_filter_distance_range.from_value = 0.001;
    ground_filter_distance_range.to_value = 1.0;
    ground_filter_distance_desc.floating_point_range.push_back(ground_filter_distance_range);
    ground_filter_distance_ =
      declare_parameter("ground_filter.distance", 0.04, ground_filter_distance_desc);
  }
  {
    // angular derivation of found plane:
    rcl_interfaces::msg::ParameterDescriptor ground_filter_angle_desc;
    ground_filter_angle_desc.description =
      "Angular threshold of the detected plane from the horizontal plane to be detected as ground";
    rcl_interfaces::msg::FloatingPointRange ground_filter_angle_range;
    ground_filter_angle_range.from_value = 0.001;
    ground_filter_angle_range.to_value = 15.0;
    ground_filter_angle_desc.floating_point_range.push_back(ground_filter_angle_range);
    ground_filter_angle_ = declare_parameter("ground_filter.angle", 0.15);
  }
  {
    // distance of found plane from z=0 to be detected as ground (e.g. to exclude tables)
    rcl_interfaces::msg::ParameterDescriptor ground_filter_plane_distance_desc;
    ground_filter_plane_distance_desc.description =
      "Distance threshold from z=0 for a plane to be detected as ground";
    rcl_interfaces::msg::FloatingPointRange ground_filter_plane_distance_range;
    ground_filter_plane_distance_range.from_value = 0.001;
    ground_filter_plane_distance_range.to_value = 1.0;
    ground_filter_plane_distance_desc.floating_point_range.push_back(
      ground_filter_plane_distance_range
    );
    ground_filter_plane_distance_ =
      declare_parameter("ground_filter.plane_distance", 0.07, ground_filter_plane_distance_desc);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor max_range_desc;
    max_range_desc.description = "Sensor maximum range";
    rcl_interfaces::msg::FloatingPointRange max_range_range;
    max_range_range.from_value = -1.0;
    max_range_range.to_value = 100.0;
    max_range_desc.floating_point_range.push_back(max_range_range);
    max_range_ = declare_parameter("sensor_model.max_range", -1.0, max_range_desc);
  }

  res_ = declare_parameter("resolution", 0.05);

  rcl_interfaces::msg::ParameterDescriptor prob_hit_desc;
  prob_hit_desc.description =
    "Probabilities for hits in the sensor model when dynamically building a map";
  rcl_interfaces::msg::FloatingPointRange prob_hit_range;
  prob_hit_range.from_value = 0.5;
  prob_hit_range.to_value = 1.0;
  prob_hit_desc.floating_point_range.push_back(prob_hit_range);
  const double prob_hit = declare_parameter("sensor_model.hit", 0.7, prob_hit_desc);

  rcl_interfaces::msg::ParameterDescriptor prob_miss_desc;
  prob_miss_desc.description =
    "Probabilities for misses in the sensor model when dynamically building a map";
  rcl_interfaces::msg::FloatingPointRange prob_miss_range;
  prob_miss_range.from_value = 0.0;
  prob_miss_range.to_value = 0.5;
  prob_miss_desc.floating_point_range.push_back(prob_miss_range);
  const double prob_miss = declare_parameter("sensor_model.miss", 0.4, prob_miss_desc);

  rcl_interfaces::msg::ParameterDescriptor prob_min_desc;
  prob_min_desc.description =
    "Minimum probability for clamping when dynamically building a map";
  rcl_interfaces::msg::FloatingPointRange prob_min_range;
  prob_min_range.from_value = 0.0;
  prob_min_range.to_value = 1.0;
  prob_min_desc.floating_point_range.push_back(prob_min_range);
  const double thres_min = declare_parameter("sensor_model.min", 0.12, prob_min_desc);

  rcl_interfaces::msg::ParameterDescriptor prob_max_desc;
  prob_max_desc.description =
    "Maximum probability for clamping when dynamically building a map";
  rcl_interfaces::msg::FloatingPointRange prob_max_range;
  prob_max_range.from_value = 0.0;
  prob_max_range.to_value = 1.0;
  prob_max_desc.floating_point_range.push_back(prob_max_range);
  const double thres_max = declare_parameter("sensor_model.max", 0.97, prob_max_desc);

  {
    rcl_interfaces::msg::ParameterDescriptor compress_map_desc;
    compress_map_desc.description = "Compresses the map losslessly";
    compress_map_ = declare_parameter("compress_map", true, compress_map_desc);
  }
  {
    rcl_interfaces::msg::ParameterDescriptor incremental_2D_projection_desc;
    incremental_2D_projection_desc.description = "Incremental 2D projection";
    incremental_2D_projection_ =
      declare_parameter("incremental_2D_projection", false, incremental_2D_projection_desc);
  }

  if (filter_ground_plane_ && (point_cloud_min_z_ > 0.0 || point_cloud_max_z_ < 0.0)) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "You enabled ground filtering but incoming pointclouds will be pre-filtered in [" <<
        point_cloud_min_z_ << ", " << point_cloud_max_z_ << "], excluding the ground level z=0. "
        "This will not work.");
  }

  if (use_height_map_ && use_colored_map_) {
    RCLCPP_WARN_STREAM(
      get_logger(),
      "You enabled both height map and RGB color registration. "
      "This is contradictory. Defaulting to height map.");
    use_colored_map_ = false;
  }

  if (use_colored_map_) {
#ifdef COLOR_OCTOMAP_SERVER
    RCLCPP_INFO_STREAM(get_logger(), "Using RGB color registration (if information available)");
#else
    RCLCPP_ERROR_STREAM(
      get_logger(),
      "Colored map requested in launch file - node not running/compiled to support colors, "
      "please define COLOR_OCTOMAP_SERVER and recompile or launch the octomap_color_server node");
#endif
  }

  // initialize octomap object & params
  octree_ = std::make_unique<OcTreeT>(res_);
  octree_->setProbHit(prob_hit);
  octree_->setProbMiss(prob_miss);
  octree_->setClampingThresMin(thres_min);
  octree_->setClampingThresMax(thres_max);
  tree_depth_ = octree_->getTreeDepth();
  max_tree_depth_ = tree_depth_;
  {
    rcl_interfaces::msg::ParameterDescriptor max_depth_desc;
    max_depth_desc.description =
      "Maximum depth when traversing the octree to send out markers."
      "16: full depth / max. resolution";
    rcl_interfaces::msg::IntegerRange max_depth_range;
    max_depth_range.from_value = 1;
    max_depth_range.to_value = 16;
    max_depth_desc.integer_range.push_back(max_depth_range);
    declare_parameter("max_depth", static_cast<int64_t>(max_tree_depth_), max_depth_desc);
  }
  gridmap_.info.resolution = res_;

  color_.r = declare_parameter("color.r", 0.0);
  color_.g = declare_parameter("color.g", 0.0);
  color_.b = declare_parameter("color.b", 1.0);
  color_.a = declare_parameter("color.a", 1.0);

  color_free_.r = declare_parameter("color_free.r", 0.0);
  color_free_.g = declare_parameter("color_free.g", 1.0);
  color_free_.b = declare_parameter("color_free.b", 0.0);
  color_free_.r = declare_parameter("color_free.a", 1.0);

  publish_free_space_ = declare_parameter("publish_free_space", false);

  latched_topics_ = declare_parameter("latch", true);
  if (latched_topics_) {
    RCLCPP_INFO(
      get_logger(),
      "Publishing latched (single publish will take longer, "
      "all topics are prepared)");
  } else {
    RCLCPP_INFO(
      get_logger(),
      "Publishing non-latched (topics are only prepared as needed, "
      "will only be re-published on map change");
  }
  auto qos = latched_topics_ ? rclcpp::QoS{1}.transient_local() : rclcpp::QoS{1};
  marker_pub_ = create_publisher<MarkerArray>("occupied_cells_vis_array", qos);
  binary_map_pub_ = create_publisher<Octomap>("octomap_binary", qos);
  full_map_pub_ = create_publisher<Octomap>("octomap_full", qos);
  point_cloud_pub_ = create_publisher<PointCloud2>("octomap_point_cloud_centers", qos);
  map_pub_ = create_publisher<OccupancyGrid>("projected_map", qos.keep_last(5));
  fmarker_pub_ = create_publisher<MarkerArray>("free_cells_vis_array", qos);

  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ =
    std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  using std::chrono_literals::operator""s;
  point_cloud_sub_.subscribe(this, "cloud_in", rmw_qos_profile_sensor_data);
  tf_point_cloud_sub_ = std::make_shared<tf2_ros::MessageFilter<PointCloud2>>(
    point_cloud_sub_, *tf2_buffer_, world_frame_id_, 5, this->get_node_logging_interface(),
    this->get_node_clock_interface(), 5s);

  tf_point_cloud_sub_->registerCallback(&OctomapServer::insertCloudCallback, this);

  octomap_binary_srv_ = create_service<OctomapSrv>(
    "octomap_binary", std::bind(&OctomapServer::onOctomapBinarySrv, this, _1, _2));
  octomap_full_srv_ = create_service<OctomapSrv>(
    "octomap_full", std::bind(&OctomapServer::onOctomapFullSrv, this, _1, _2));
  clear_bbox_srv_ = create_service<BBoxSrv>(
    "~/clear_bbox", std::bind(&OctomapServer::clearBBoxSrv, this, _1, _2));
  reset_srv_ = create_service<ResetSrv>(
    "~/reset", std::bind(&OctomapServer::resetSrv, this, _1, _2));

  // set parameter callback
  set_param_res_ =
    this->add_on_set_parameters_callback(std::bind(&OctomapServer::onParameter, this, _1));

  const auto filename = declare_parameter("octomap_path", "");
  if (!openFile(filename)) {
    RCLCPP_WARN(get_logger(), "Could not open file %s", filename.c_str());
  }
}

bool OctomapServer::openFile(const std::string & filename)
{
  if (filename.length() <= 3) {
    return false;
  }

  std::string suffix = filename.substr(filename.length() - 3, 3);
  if (suffix == ".bt") {
    if (!octree_->readBinary(filename)) {
      return false;
    }
  } else if (suffix == ".ot") {
    std::unique_ptr<octomap::AbstractOcTree> tree{octomap::AbstractOcTree::read(filename)};
    if (!tree) {
      return false;
    }
    octree_ = std::unique_ptr<OcTreeT>(dynamic_cast<OcTreeT *>(tree.release()));
    if (!octree_) {
      RCLCPP_ERROR(
        get_logger(),
        "Could not read OcTree in file, currently there are no other types supported in .ot");
      return false;
    }

  } else {
    return false;
  }

  RCLCPP_INFO(
    get_logger(), "Octomap file %s loaded (%zu nodes).", filename.c_str(),
    octree_->size());

  tree_depth_ = octree_->getTreeDepth();
  max_tree_depth_ = tree_depth_;
  res_ = octree_->getResolution();
  gridmap_.info.resolution = res_;
  double min_x{};
  double min_y{};
  double min_z{};
  double max_x{};
  double max_y{};
  double max_z{};
  octree_->getMetricMin(min_x, min_y, min_z);
  octree_->getMetricMax(max_x, max_y, max_z);

  update_bbox_min_[0] = octree_->coordToKey(min_x);
  update_bbox_min_[1] = octree_->coordToKey(min_y);
  update_bbox_min_[2] = octree_->coordToKey(min_z);

  update_bbox_max_[0] = octree_->coordToKey(max_x);
  update_bbox_max_[1] = octree_->coordToKey(max_y);
  update_bbox_max_[2] = octree_->coordToKey(max_z);

  publishAll(now());

  return true;
}

void OctomapServer::insertCloudCallback(const PointCloud2::ConstSharedPtr cloud)
{
  const auto start_time = rclcpp::Clock{}.now();

  //
  // ground filtering in base frame
  //
  PCLPointCloud pc;  // input cloud for filtering and ground-detection
  pcl::fromROSMsg(*cloud, pc);

  geometry_msgs::msg::TransformStamped sensor_to_world_transform_stamped;
  try {
    sensor_to_world_transform_stamped = tf2_buffer_->lookupTransform(
      world_frame_id_, cloud->header.frame_id, cloud->header.stamp,
      rclcpp::Duration::from_seconds(1.0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    return;
  }

  Eigen::Matrix4f sensor_to_world =
    tf2::transformToEigen(sensor_to_world_transform_stamped.transform).matrix().cast<float>();

  // set up filter for height range, also removes NANs:
  pcl::PassThrough<PCLPoint> pass_x;
  pass_x.setFilterFieldName("x");
  pass_x.setFilterLimits(point_cloud_min_x_, point_cloud_max_x_);
  pcl::PassThrough<PCLPoint> pass_y;
  pass_y.setFilterFieldName("y");
  pass_y.setFilterLimits(point_cloud_min_y_, point_cloud_max_y_);
  pcl::PassThrough<PCLPoint> pass_z;
  pass_z.setFilterFieldName("z");
  pass_z.setFilterLimits(point_cloud_min_z_, point_cloud_max_z_);

  PCLPointCloud pc_ground;  // segmented ground plane
  PCLPointCloud pc_nonground;  // everything else

  if (filter_ground_plane_) {
    geometry_msgs::msg::TransformStamped sensor_to_base_transform_stamped;
    geometry_msgs::msg::TransformStamped base_to_world_transform_stamped;
    try {
      tf2_buffer_->canTransform(
        base_frame_id_, cloud->header.frame_id, cloud->header.stamp,
        rclcpp::Duration::from_seconds(0.2));
      sensor_to_base_transform_stamped = tf2_buffer_->lookupTransform(
        base_frame_id_, cloud->header.frame_id, cloud->header.stamp,
        rclcpp::Duration::from_seconds(1.0));
      base_to_world_transform_stamped = tf2_buffer_->lookupTransform(
        world_frame_id_, base_frame_id_, cloud->header.stamp,
        rclcpp::Duration::from_seconds(1.0));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "Transform error for ground plane filter: " << ex.what() << ", quitting callback.\n"
          "You need to set the base_frame_id or disable filter_ground.");
    }


    Eigen::Matrix4f sensor_to_base =
      tf2::transformToEigen(sensor_to_base_transform_stamped.transform).matrix().cast<float>();
    Eigen::Matrix4f base_to_world =
      tf2::transformToEigen(base_to_world_transform_stamped.transform).matrix().cast<float>();

    // transform pointcloud from sensor frame to fixed robot frame
    pcl::transformPointCloud(pc, pc, sensor_to_base);
    pass_x.setInputCloud(pc.makeShared());
    pass_x.filter(pc);
    pass_y.setInputCloud(pc.makeShared());
    pass_y.filter(pc);
    pass_z.setInputCloud(pc.makeShared());
    pass_z.filter(pc);
    filterGroundPlane(pc, pc_ground, pc_nonground);

    // transform clouds to world frame for insertion
    pcl::transformPointCloud(pc_ground, pc_ground, base_to_world);
    pcl::transformPointCloud(pc_nonground, pc_nonground, base_to_world);
  } else {
    // directly transform to map frame:
    pcl::transformPointCloud(pc, pc, sensor_to_world);

    // just filter height range:
    pass_x.setInputCloud(pc.makeShared());
    pass_x.filter(pc);
    pass_y.setInputCloud(pc.makeShared());
    pass_y.filter(pc);
    pass_z.setInputCloud(pc.makeShared());
    pass_z.filter(pc);

    pc_nonground = pc;
    // pc_nonground is empty without ground segmentation
    pc_ground.header = pc.header;
    pc_nonground.header = pc.header;
  }

  const auto & t = sensor_to_world_transform_stamped.transform.translation;
  tf2::Vector3 sensor_to_world_vec3{t.x, t.y, t.z};
  insertScan(sensor_to_world_vec3, pc_ground, pc_nonground);

  double total_elapsed = (rclcpp::Clock{}.now() - start_time).seconds();
  RCLCPP_DEBUG(
    get_logger(),
    "Pointcloud insertion in OctomapServer done (%zu+%zu pts (ground/nonground), %f sec)",
    pc_ground.size(), pc_nonground.size(), total_elapsed);

  publishAll(cloud->header.stamp);
}

void OctomapServer::insertScan(
  const tf2::Vector3 & sensor_origin_tf, const PCLPointCloud & ground,
  const PCLPointCloud & nonground)
{
  const auto sensor_origin = octomap::pointTfToOctomap(sensor_origin_tf);

  if (!octree_->coordToKeyChecked(sensor_origin, update_bbox_min_) ||
    !octree_->coordToKeyChecked(sensor_origin, update_bbox_max_))
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Could not generate Key for origin " << sensor_origin);
  }

  // instead of direct scan insertion, compute update to filter ground:
  octomap::KeySet free_cells, occupied_cells;
  // insert ground points only as free:
  for (PCLPointCloud::const_iterator it = ground.begin(); it != ground.end(); ++it) {
    octomap::point3d point(it->x, it->y, it->z);
    // maxrange check
    if ((max_range_ > 0.0) && ((point - sensor_origin).norm() > max_range_) ) {
      point = sensor_origin + (point - sensor_origin).normalized() * max_range_;
    }

    // only clear space (ground points)
    if (octree_->computeRayKeys(sensor_origin, point, key_ray_)) {
      free_cells.insert(key_ray_.begin(), key_ray_.end());
    }

    octomap::OcTreeKey end_key;
    if (octree_->coordToKeyChecked(point, end_key)) {
      updateMinKey(end_key, update_bbox_min_);
      updateMaxKey(end_key, update_bbox_max_);
    } else {
      RCLCPP_ERROR_STREAM(get_logger(), "Could not generate Key for endpoint " << point);
    }
  }

  // all other points: free on ray, occupied on endpoint:
  for (PCLPointCloud::const_iterator it = nonground.begin(); it != nonground.end(); ++it) {
    octomap::point3d point(it->x, it->y, it->z);
    // maxrange check
    if ((max_range_ < 0.0) || ((point - sensor_origin).norm() <= max_range_) ) {
      // free cells
      if (octree_->computeRayKeys(sensor_origin, point, key_ray_)) {
        free_cells.insert(key_ray_.begin(), key_ray_.end());
      }
      // occupied endpoint
      octomap::OcTreeKey key;
      if (octree_->coordToKeyChecked(point, key)) {
        occupied_cells.insert(key);

        updateMinKey(key, update_bbox_min_);
        updateMaxKey(key, update_bbox_max_);

#ifdef COLOR_OCTOMAP_SERVER  // NB: Only read and interpret color if it's an occupied node
        octree_->averageNodeColor(it->x, it->y, it->z, /*r=*/ it->r, /*g=*/ it->g, /*b=*/ it->b);
#endif
      }
    } else {  // ray longer than maxrange
      octomap::point3d new_end = sensor_origin + (point - sensor_origin).normalized() * max_range_;
      if (octree_->computeRayKeys(sensor_origin, new_end, key_ray_)) {
        free_cells.insert(key_ray_.begin(), key_ray_.end());

        octomap::OcTreeKey end_key;
        if (octree_->coordToKeyChecked(new_end, end_key)) {
          free_cells.insert(end_key);
          updateMinKey(end_key, update_bbox_min_);
          updateMaxKey(end_key, update_bbox_max_);
        } else {
          RCLCPP_ERROR_STREAM(get_logger(), "Could not generate Key for endpoint " << new_end);
        }
      }
    }
  }

  // mark free cells only if not seen occupied in this cloud
  for (auto it = free_cells.begin(), end = free_cells.end(); it != end; ++it) {
    if (occupied_cells.find(*it) == occupied_cells.end()) {
      octree_->updateNode(*it, false);
    }
  }

  // now mark all occupied cells:
  for (auto it = occupied_cells.begin(), end = occupied_cells.end(); it != end; it++) {
    octree_->updateNode(*it, true);
  }

  // TODO(someone): eval lazy+updateInner vs. proper insertion
  // non-lazy by default (updateInnerOccupancy() too slow for large maps)
  // octree_->updateInnerOccupancy();
  octomap::point3d min_pt, max_pt;
  RCLCPP_DEBUG_STREAM(
    get_logger(),
    "Bounding box keys (before): " << update_bbox_min_[0] << " " << update_bbox_min_[1] << " " <<
      update_bbox_min_[2] << " / " << update_bbox_max_[0] << " " << update_bbox_max_[1] << " " <<
      update_bbox_max_[2]);

  // TODO(someone): snap max / min keys to larger voxels by max_tree_depth_
  // if (max_tree_depth_ < 16)
  // {
  //    // this should give us the first key at depthmax_tree_depth_
  //    // that is smaller or equal to update_bbox_min_ (i.e. lower left in 2D grid coordinates)
  //    OcTreeKey tmpMin = getIndexKey(update_bbox_min_,max_tree_depth_);
  //    // see above, now add something to find upper right
  //    OcTreeKey tmpMax = getIndexKey(update_bbox_max_,max_tree_depth_);
  //    tmpMax[0]+= octree_->getNodeSize(max_tree_depth_ ) - 1;
  //    tmpMax[1]+= octree_->getNodeSize(max_tree_depth_ ) - 1;
  //    tmpMax[2]+= octree_->getNodeSize(max_tree_depth_ ) - 1;
  //    update_bbox_min_ = tmpMin;
  //    update_bbox_max_ = tmpMax;
  // }

  // TODO(someone): we could also limit the bbx to be
  // within the map bounds here (see publishing check)
  min_pt = octree_->keyToCoord(update_bbox_min_);
  max_pt = octree_->keyToCoord(update_bbox_max_);
  RCLCPP_DEBUG_STREAM(get_logger(), "Updated area bounding box: " << min_pt << " - " << max_pt);
  RCLCPP_DEBUG_STREAM(
    get_logger(),
    "Bounding box keys (after): " << update_bbox_min_[0] << " " << update_bbox_min_[1] << " " <<
      update_bbox_min_[2] << " / " << update_bbox_max_[0] << " " << update_bbox_max_[1] << " " <<
      update_bbox_max_[2]);

  if (compress_map_) {
    octree_->prune();
  }
}


void OctomapServer::publishAll(const rclcpp::Time & rostime)
{
  const auto start_time = rclcpp::Clock{}.now();
  const size_t octomap_size = octree_->size();
  // TODO(someone): estimate num occ. voxels for size of arrays (reserve)
  if (octomap_size <= 1) {
    RCLCPP_WARN(get_logger(), "Nothing to publish, octree is empty");
    return;
  }

  bool publish_free_marker_array_ = publish_free_space_ &&
    (latched_topics_ ||
    fmarker_pub_->get_subscription_count() +
    fmarker_pub_->get_intra_process_subscription_count() > 0);
  bool publish_marker_array =
    (latched_topics_ ||
    marker_pub_->get_subscription_count() +
    marker_pub_->get_intra_process_subscription_count() > 0);
  bool publish_point_cloud =
    (latched_topics_ ||
    point_cloud_pub_->get_subscription_count() +
    point_cloud_pub_->get_intra_process_subscription_count() > 0);
  bool publish_binary_map =
    (latched_topics_ ||
    binary_map_pub_->get_subscription_count() +
    binary_map_pub_->get_intra_process_subscription_count() > 0);
  bool publish_full_map =
    (latched_topics_ ||
    full_map_pub_->get_subscription_count() +
    full_map_pub_->get_intra_process_subscription_count() > 0);
  publish_2d_map_ =
    (latched_topics_ ||
    map_pub_->get_subscription_count() +
    map_pub_->get_intra_process_subscription_count() > 0);

  // init markers for free space:
  MarkerArray free_nodes_vis;
  // each array stores all cubes of a different size, one for each depth level:
  free_nodes_vis.markers.resize(tree_depth_ + 1);

  geometry_msgs::msg::Pose pose;

  // init markers:
  MarkerArray occupied_nodes_vis;
  // each array stores all cubes of a different size, one for each depth level:
  occupied_nodes_vis.markers.resize(tree_depth_ + 1);

  // init pointcloud:
  pcl::PointCloud<PCLPoint> pcl_cloud;

  // call pre-traversal hook:
  handlePreNodeTraversal(rostime);

  // now, traverse all leafs in the tree:
  for (OcTreeT::iterator it = octree_->begin(max_tree_depth_),
    end = octree_->end(); it != end; ++it)
  {
    bool in_update_bbox = isInUpdateBBX(it);

    // call general hook:
    handleNode(it);
    if (in_update_bbox) {
      handleNodeInBBX(it);
    }

    if (octree_->isNodeOccupied(*it)) {
      double z = it.getZ();
      double half_size = it.getSize() / 2.0;
      if (z + half_size > occupancy_min_z_ && z - half_size < occupancy_max_z_) {
        double x = it.getX();
        double y = it.getY();
#ifdef COLOR_OCTOMAP_SERVER
        int r = it->getColor().r;
        int g = it->getColor().g;
        int b = it->getColor().b;
#endif

        // Ignore speckles in the map:
        if (filter_speckles_ && (it.getDepth() == tree_depth_ + 1) && isSpeckleNode(it.getKey())) {
          RCLCPP_DEBUG(get_logger(), "Ignoring single speckle at (%f,%f,%f)", x, y, z);
          continue;
        }  // else: current octree node is no speckle, send it out

        handleOccupiedNode(it);
        if (in_update_bbox) {
          handleOccupiedNodeInBBX(it);
        }


        // create marker:
        if (publish_marker_array) {
          unsigned idx = it.getDepth();
          assert(idx < occupied_nodes_vis.markers.size());

          geometry_msgs::msg::Point cube_center;
          cube_center.x = x;
          cube_center.y = y;
          cube_center.z = z;

          occupied_nodes_vis.markers[idx].points.push_back(cube_center);
          if (use_height_map_) {
            double min_x{};
            double min_y{};
            double min_z{};
            double max_x{};
            double max_y{};
            double max_z{};
            octree_->getMetricMin(min_x, min_y, min_z);
            octree_->getMetricMax(max_x, max_y, max_z);

            double h =
              (1.0 -
              std::min(
                std::max((cube_center.z - min_z) / (max_z - min_z), 0.0),
                1.0)) * color_factor_;
            occupied_nodes_vis.markers[idx].colors.push_back(heightMapColor(h));
          }

#ifdef COLOR_OCTOMAP_SERVER
          if (use_colored_map_) {
            ColorRGBA _color;
            _color.r = (r / 255.);
            _color.g = (g / 255.);
            _color.b = (b / 255.);
            // TODO(someone): EVALUATE: potentially use occupancy as measure for alpha channel?
            _color.a = 1.0;
            occupied_nodes_vis.markers[idx].colors.push_back(_color);
          }
#endif
        }

        // insert into pointcloud:
        if (publish_point_cloud) {
#ifdef COLOR_OCTOMAP_SERVER
          PCLPoint _point = PCLPoint();
          _point.x = x;
          _point.y = y;
          _point.z = z;
          _point.r = r;
          _point.g = g;
          _point.b = b;
          pcl_cloud.push_back(_point);
#else
          pcl_cloud.push_back(PCLPoint(x, y, z));
#endif
        }
      }
    } else {  // node not occupied => mark as free in 2D map if unknown so far
      double z = it.getZ();
      double half_size = it.getSize() / 2.0;
      if (z + half_size > occupancy_min_z_ && z - half_size < occupancy_max_z_) {
        handleFreeNode(it);
        if (in_update_bbox) {
          handleFreeNodeInBBX(it);
        }

        if (publish_free_space_) {
          double x = it.getX();
          double y = it.getY();

          // create marker for free space:
          if (publish_free_marker_array_) {
            unsigned idx = it.getDepth();
            assert(idx < free_nodes_vis.markers.size());

            geometry_msgs::msg::Point cube_center;
            cube_center.x = x;
            cube_center.y = y;
            cube_center.z = z;

            free_nodes_vis.markers[idx].points.push_back(cube_center);
          }
        }
      }
    }
  }

  // call post-traversal hook:
  handlePostNodeTraversal(rostime);

  // finish MarkerArray:
  if (publish_marker_array) {
    for (size_t i = 0; i < occupied_nodes_vis.markers.size(); ++i) {
      double size = octree_->getNodeSize(i);

      occupied_nodes_vis.markers[i].header.frame_id = world_frame_id_;
      occupied_nodes_vis.markers[i].header.stamp = rostime;
      occupied_nodes_vis.markers[i].ns = "map";
      occupied_nodes_vis.markers[i].id = i;
      occupied_nodes_vis.markers[i].type = visualization_msgs::msg::Marker::CUBE_LIST;
      occupied_nodes_vis.markers[i].scale.x = size;
      occupied_nodes_vis.markers[i].scale.y = size;
      occupied_nodes_vis.markers[i].scale.z = size;
      if (!use_colored_map_) {
        occupied_nodes_vis.markers[i].color = color_;
      }


      if (occupied_nodes_vis.markers[i].points.size() > 0) {
        occupied_nodes_vis.markers[i].action = visualization_msgs::msg::Marker::ADD;
      } else {
        occupied_nodes_vis.markers[i].action = visualization_msgs::msg::Marker::DELETE;
      }
    }

    marker_pub_->publish(occupied_nodes_vis);
  }


  // finish FreeMarkerArray:
  if (publish_free_marker_array_) {
    for (size_t i = 0; i < free_nodes_vis.markers.size(); ++i) {
      double size = octree_->getNodeSize(i);

      free_nodes_vis.markers[i].header.frame_id = world_frame_id_;
      free_nodes_vis.markers[i].header.stamp = rostime;
      free_nodes_vis.markers[i].ns = "map";
      free_nodes_vis.markers[i].id = i;
      free_nodes_vis.markers[i].type = visualization_msgs::msg::Marker::CUBE_LIST;
      free_nodes_vis.markers[i].scale.x = size;
      free_nodes_vis.markers[i].scale.y = size;
      free_nodes_vis.markers[i].scale.z = size;
      free_nodes_vis.markers[i].color = color_free_;


      if (free_nodes_vis.markers[i].points.size() > 0) {
        free_nodes_vis.markers[i].action = visualization_msgs::msg::Marker::ADD;
      } else {
        free_nodes_vis.markers[i].action = visualization_msgs::msg::Marker::DELETE;
      }
    }

    fmarker_pub_->publish(free_nodes_vis);
  }
  // finish pointcloud:
  if (publish_point_cloud) {
    PointCloud2 cloud;
    pcl::toROSMsg(pcl_cloud, cloud);
    cloud.header.frame_id = world_frame_id_;
    cloud.header.stamp = rostime;
    point_cloud_pub_->publish(cloud);
  }

  if (publish_binary_map) {
    publishBinaryOctoMap(rostime);
  }

  if (publish_full_map) {
    publishFullOctoMap(rostime);
  }
  double total_elapsed = (rclcpp::Clock{}.now() - start_time).seconds();
  RCLCPP_DEBUG(get_logger(), "Map publishing in OctomapServer took %f sec", total_elapsed);
}

bool OctomapServer::onOctomapBinarySrv(
  [[maybe_unused]] const std::shared_ptr<OctomapSrv::Request> req,
  const std::shared_ptr<OctomapSrv::Response> res)
{
  const auto start_time = rclcpp::Clock{}.now();
  RCLCPP_INFO(get_logger(), "Sending binary map data on service request");
  res->map.header.frame_id = world_frame_id_;
  res->map.header.stamp = now();
  if (!octomap_msgs::binaryMapToMsg(*octree_, res->map)) {
    return false;
  }

  double total_elapsed = (rclcpp::Clock{}.now() - start_time).seconds();
  RCLCPP_INFO(get_logger(), "Binary octomap sent in %f sec", total_elapsed);
  return true;
}

bool OctomapServer::onOctomapFullSrv(
  [[maybe_unused]] const std::shared_ptr<OctomapSrv::Request> req,
  const std::shared_ptr<OctomapSrv::Response> res)
{
  RCLCPP_INFO(get_logger(), "Sending full map data on service request");
  res->map.header.frame_id = world_frame_id_;
  res->map.header.stamp = now();


  if (!octomap_msgs::fullMapToMsg(*octree_, res->map)) {
    return false;
  }

  return true;
}

bool OctomapServer::clearBBoxSrv(
  const std::shared_ptr<BBoxSrv::Request> req,
  [[maybe_unused]] const std::shared_ptr<BBoxSrv::Response> resp)
{
  const auto min = octomap::pointMsgToOctomap(req->min);
  const auto max = octomap::pointMsgToOctomap(req->max);

  const double thres_min = octree_->getClampingThresMin();
  for (auto it = octree_->begin_leafs_bbx(min, max),
    end = octree_->end_leafs_bbx(); it != end; ++it)
  {
    it->setLogOdds(octomap::logodds(thres_min));
    // octree_->updateNode(it.getKey(), -6.0f);
  }
  // TODO(someone): eval which is faster (setLogOdds+updateInner or updateNode)
  octree_->updateInnerOccupancy();

  publishAll(now());

  return true;
}

bool OctomapServer::resetSrv(
  [[maybe_unused]] const std::shared_ptr<ResetSrv::Request> req,
  [[maybe_unused]] const std::shared_ptr<ResetSrv::Response> resp)
{
  MarkerArray occupied_nodes_vis;
  occupied_nodes_vis.markers.resize(tree_depth_ + 1);
  const auto rostime = now();
  octree_->clear();
  // clear 2D map:
  gridmap_.data.clear();
  gridmap_.info.height = 0.0;
  gridmap_.info.width = 0.0;
  gridmap_.info.resolution = 0.0;
  gridmap_.info.origin.position.x = 0.0;
  gridmap_.info.origin.position.y = 0.0;

  RCLCPP_INFO(get_logger(), "Cleared octomap");
  publishAll(rostime);

  publishBinaryOctoMap(rostime);
  for (size_t i = 0; i < occupied_nodes_vis.markers.size(); ++i) {
    occupied_nodes_vis.markers[i].header.frame_id = world_frame_id_;
    occupied_nodes_vis.markers[i].header.stamp = rostime;
    occupied_nodes_vis.markers[i].ns = "map";
    occupied_nodes_vis.markers[i].id = i;
    occupied_nodes_vis.markers[i].type = visualization_msgs::msg::Marker::CUBE_LIST;
    occupied_nodes_vis.markers[i].action = visualization_msgs::msg::Marker::DELETE;
  }

  marker_pub_->publish(occupied_nodes_vis);

  MarkerArray free_nodes_vis;
  free_nodes_vis.markers.resize(tree_depth_ + 1);

  for (size_t i = 0; i < free_nodes_vis.markers.size(); ++i) {
    free_nodes_vis.markers[i].header.frame_id = world_frame_id_;
    free_nodes_vis.markers[i].header.stamp = rostime;
    free_nodes_vis.markers[i].ns = "map";
    free_nodes_vis.markers[i].id = i;
    free_nodes_vis.markers[i].type = visualization_msgs::msg::Marker::CUBE_LIST;
    free_nodes_vis.markers[i].action = visualization_msgs::msg::Marker::DELETE;
  }
  fmarker_pub_->publish(free_nodes_vis);
  return true;
}

void OctomapServer::publishBinaryOctoMap(const rclcpp::Time & rostime) const
{
  Octomap map;
  map.header.frame_id = world_frame_id_;
  map.header.stamp = rostime;
  if (octomap_msgs::binaryMapToMsg(*octree_, map)) {
    binary_map_pub_->publish(map);
  } else {
    RCLCPP_ERROR(get_logger(), "Error serializing OctoMap");
  }
}

void OctomapServer::publishFullOctoMap(const rclcpp::Time & rostime) const
{
  Octomap map;
  map.header.frame_id = world_frame_id_;
  map.header.stamp = rostime;
  if (octomap_msgs::fullMapToMsg(*octree_, map)) {
    full_map_pub_->publish(map);
  } else {
    RCLCPP_ERROR(get_logger(), "Error serializing OctoMap");
  }
}

void OctomapServer::filterGroundPlane(
  const PCLPointCloud & pc, PCLPointCloud & ground,
  PCLPointCloud & nonground) const
{
  ground.header = pc.header;
  nonground.header = pc.header;

  if (pc.size() < 50) {
    RCLCPP_WARN(
      get_logger(), "Pointcloud in OctomapServer too small, skipping ground plane extraction");
    nonground = pc;
  } else {
    // plane detection for ground plane removal:
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    // Create the segmentation object and set up:
    pcl::SACSegmentation<PCLPoint> seg;
    seg.setOptimizeCoefficients(true);
    // TODO(someone): maybe a filtering based on the surface normals
    // might be more robust / accurate?
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(200);
    seg.setDistanceThreshold(ground_filter_distance_);
    seg.setAxis(Eigen::Vector3f(0, 0, 1));
    seg.setEpsAngle(ground_filter_angle_);


    PCLPointCloud cloud_filtered(pc);
    // Create the filtering object
    pcl::ExtractIndices<PCLPoint> extract;
    bool ground_plane_found = false;

    while (cloud_filtered.size() > 10 && !ground_plane_found) {
      seg.setInputCloud(cloud_filtered.makeShared());
      seg.segment(*inliers, *coefficients);
      if (inliers->indices.size() == 0) {
        RCLCPP_INFO(get_logger(), "PCL segmentation did not find any plane.");

        break;
      }

      extract.setInputCloud(cloud_filtered.makeShared());
      extract.setIndices(inliers);

      if (std::abs(coefficients->values.at(3)) < ground_filter_plane_distance_) {
        RCLCPP_DEBUG(
          get_logger(),
          "Ground plane found: %zu/%zu inliers. Coeff: %f %f %f %f",
          inliers->indices.size(), cloud_filtered.size(),
          coefficients->values.at(0), coefficients->values.at(1), coefficients->values.at(
            2), coefficients->values.at(3));
        extract.setNegative(false);
        extract.filter(ground);

        // remove ground points from full pointcloud:
        // workaround for PCL bug:
        if (inliers->indices.size() != cloud_filtered.size()) {
          extract.setNegative(true);
          PCLPointCloud cloud_out;
          extract.filter(cloud_out);
          nonground += cloud_out;
          cloud_filtered = cloud_out;
        }

        ground_plane_found = true;
      } else {
        RCLCPP_DEBUG(
          get_logger(),
          "Horizontal plane (not ground) found: %zu/%zu inliers. Coeff: %f %f %f %f",
          inliers->indices.size(), cloud_filtered.size(),
          coefficients->values.at(0), coefficients->values.at(1), coefficients->values.at(
            2), coefficients->values.at(3));
        pcl::PointCloud<PCLPoint> cloud_out;
        extract.setNegative(false);
        extract.filter(cloud_out);
        nonground += cloud_out;
        // debug
        //            pcl::PCDWriter writer;
        //            writer.write<PCLPoint>("nonground_plane.pcd",cloud_out, false);

        // remove current plane from scan for next iteration:
        // workaround for PCL bug:
        if (inliers->indices.size() != cloud_filtered.size()) {
          extract.setNegative(true);
          cloud_out.points.clear();
          extract.filter(cloud_out);
          cloud_filtered = cloud_out;
        } else {
          cloud_filtered.points.clear();
        }
      }
    }
    // TODO(someone): also do this if overall starting pointcloud too small?
    if (!ground_plane_found) {  // no plane found or remaining points too small
      RCLCPP_WARN(get_logger(), "No ground plane found in scan");
      // do a rough fitlering on height to prevent spurious obstacles
      pcl::PassThrough<PCLPoint> second_pass;
      second_pass.setFilterFieldName("z");
      second_pass.setFilterLimits(-ground_filter_plane_distance_, ground_filter_plane_distance_);
      second_pass.setInputCloud(pc.makeShared());
      second_pass.filter(ground);
      second_pass.setFilterLimitsNegative(true);
      second_pass.filter(nonground);
    }
    // debug:
    //        pcl::PCDWriter writer;
    //        if (pc_ground.size() > 0)
    //          writer.write<PCLPoint>("ground.pcd",pc_ground, false);
    //        if (pc_nonground.size() > 0)
    //          writer.write<PCLPoint>("nonground.pcd",pc_nonground, false);
  }
}

void OctomapServer::handlePreNodeTraversal(const rclcpp::Time & rostime)
{
  if (publish_2d_map_) {
    // init projected 2D map:
    gridmap_.header.frame_id = world_frame_id_;
    gridmap_.header.stamp = rostime;
    MapMetaData old_map_info = gridmap_.info;

    // TODO(someone): move most of this stuff into c'tor
    // and init map only once (adjust if size changes)
    double min_x{};
    double min_y{};
    double min_z{};
    double max_x{};
    double max_y{};
    double max_z{};
    octree_->getMetricMin(min_x, min_y, min_z);
    octree_->getMetricMax(max_x, max_y, max_z);

    octomap::point3d min_pt(min_x, min_y, min_z);
    octomap::point3d max_pt(max_x, max_y, max_z);
    octomap::OcTreeKey min_key = octree_->coordToKey(min_pt, max_tree_depth_);
    octomap::OcTreeKey max_key = octree_->coordToKey(max_pt, max_tree_depth_);

    RCLCPP_DEBUG(
      get_logger(),
      "min_key: %d %d %d / max_key: %d %d %d", min_key[0], min_key[1], min_key[2], max_key[0],
      max_key[1], max_key[2]);

    // add padding if requested (= new min/max_pts in x&y):
    double half_padded_x = 0.5 * min_x_size_;
    double half_padded_y = 0.5 * min_y_size_;
    min_x = std::min(min_x, -half_padded_x);
    max_x = std::max(max_x, half_padded_x);
    min_y = std::min(min_y, -half_padded_y);
    max_y = std::max(max_y, half_padded_y);
    min_pt = octomap::point3d(min_x, min_y, min_z);
    max_pt = octomap::point3d(max_x, max_y, max_z);

    octomap::OcTreeKey padded_max_key;
    if (!octree_->coordToKeyChecked(min_pt, max_tree_depth_, padded_min_key_)) {
      RCLCPP_ERROR(
        get_logger(),
        "Could not create padded min OcTree key at %f %f %f", min_pt.x(), min_pt.y(),
        min_pt.z());
      return;
    }
    if (!octree_->coordToKeyChecked(max_pt, max_tree_depth_, padded_max_key)) {
      RCLCPP_ERROR(
        get_logger(),
        "Could not create padded max OcTree key at %f %f %f", max_pt.x(), max_pt.y(),
        max_pt.z());
      return;
    }

    RCLCPP_DEBUG(
      get_logger(),
      "Padded MinKey: %d %d %d / padded MaxKey: %d %d %d", padded_min_key_[0],
      padded_min_key_[1], padded_min_key_[2], padded_max_key[0], padded_max_key[1],
      padded_max_key[2]);
    assert(padded_max_key[0] >= max_key[0] && padded_max_key[1] >= max_key[1]);

    multires_2d_scale_ = 1 << (tree_depth_ - max_tree_depth_);
    gridmap_.info.width = (padded_max_key[0] - padded_min_key_[0]) / multires_2d_scale_ + 1;
    gridmap_.info.height = (padded_max_key[1] - padded_min_key_[1]) / multires_2d_scale_ + 1;

    [[maybe_unused]] int map_origin_x = min_key[0] - padded_min_key_[0];
    [[maybe_unused]] int map_origin_y = min_key[1] - padded_min_key_[1];
    assert(map_origin_x >= 0 && map_origin_y >= 0);

    // might not exactly be min / max of octree:
    octomap::point3d origin = octree_->keyToCoord(padded_min_key_, tree_depth_);
    double grid_res = octree_->getNodeSize(max_tree_depth_);
    project_complete_map_ =
      (!incremental_2D_projection_ || (std::abs(grid_res - gridmap_.info.resolution) > 1e-6));
    gridmap_.info.resolution = grid_res;
    gridmap_.info.origin.position.x = origin.x() - grid_res * 0.5;
    gridmap_.info.origin.position.y = origin.y() - grid_res * 0.5;
    if (max_tree_depth_ != tree_depth_) {
      gridmap_.info.origin.position.x -= res_ / 2.0;
      gridmap_.info.origin.position.y -= res_ / 2.0;
    }

    // workaround for  multires. projection not working properly for inner nodes:
    // force re-building complete map
    if (max_tree_depth_ < tree_depth_) {
      project_complete_map_ = true;
    }

    if (project_complete_map_) {
      RCLCPP_DEBUG(get_logger(), "Rebuilding complete 2D map");
      gridmap_.data.clear();
      // init to unknown:
      gridmap_.data.resize(gridmap_.info.width * gridmap_.info.height, -1);

    } else {
      if (mapChanged(old_map_info, gridmap_.info)) {
        RCLCPP_DEBUG(
          get_logger(), "2D grid map size changed to %dx%d", gridmap_.info.width,
          gridmap_.info.height);
        adjustMapData(gridmap_, old_map_info);
      }
      OccupancyGrid::_data_type::iterator startIt;
      size_t mapUpdateBBXmin_x =
        std::max(
        0,
        (static_cast<int>(update_bbox_min_[0]) - static_cast<int>(padded_min_key_[0])) /
        static_cast<int>(multires_2d_scale_));
      size_t mapUpdateBBXmin_y =
        std::max(
        0,
        (static_cast<int>(update_bbox_min_[1]) - static_cast<int>(padded_min_key_[1])) /
        static_cast<int>(multires_2d_scale_));
      size_t mapUpdateBBXmax_x =
        std::min(
        static_cast<int>(gridmap_.info.width - 1),
        (static_cast<int>(update_bbox_max_[0]) - static_cast<int>(padded_min_key_[0])) /
        static_cast<int>(multires_2d_scale_));
      size_t mapUpdateBBXmax_y =
        std::min(
        static_cast<int>(gridmap_.info.height - 1),
        (static_cast<int>(update_bbox_max_[1]) - static_cast<int>(padded_min_key_[1])) /
        static_cast<int>(multires_2d_scale_));

      assert(mapUpdateBBXmax_x > mapUpdateBBXmin_x);
      assert(mapUpdateBBXmax_y > mapUpdateBBXmin_y);

      size_t numCols = mapUpdateBBXmax_x - mapUpdateBBXmin_x + 1;

      // test for max idx:
      uint max_idx = gridmap_.info.width * mapUpdateBBXmax_y + mapUpdateBBXmax_x;
      if (max_idx >= gridmap_.data.size()) {
        RCLCPP_ERROR(
          get_logger(),
          "BBX index not valid: %d (max index %zu for size %d x %d) "
          "update-BBX is: [%zu %zu]-[%zu %zu]", max_idx,
          gridmap_.data.size(), gridmap_.info.width, gridmap_.info.height,
          mapUpdateBBXmin_x, mapUpdateBBXmin_y, mapUpdateBBXmax_x, mapUpdateBBXmax_y);
      }

      // reset proj. 2D map in bounding box:
      for (unsigned int j = mapUpdateBBXmin_y; j <= mapUpdateBBXmax_y; ++j) {
        std::fill_n(
          gridmap_.data.begin() + gridmap_.info.width * j + mapUpdateBBXmin_x,
          numCols, -1);
      }
    }
  }
}

void OctomapServer::handlePostNodeTraversal([[maybe_unused]] const rclcpp::Time & rostime)
{
  if (publish_2d_map_) {
    map_pub_->publish(gridmap_);
  }
}

void OctomapServer::handleOccupiedNode(const OcTreeT::iterator & it)
{
  if (publish_2d_map_ && project_complete_map_) {
    update2DMap(it, true);
  }
}

void OctomapServer::handleFreeNode(const OcTreeT::iterator & it)
{
  if (publish_2d_map_ && project_complete_map_) {
    update2DMap(it, false);
  }
}

void OctomapServer::handleOccupiedNodeInBBX(const OcTreeT::iterator & it)
{
  if (publish_2d_map_ && project_complete_map_) {
    update2DMap(it, true);
  }
}

void OctomapServer::handleFreeNodeInBBX(const OcTreeT::iterator & it)
{
  if (publish_2d_map_ && project_complete_map_) {
    update2DMap(it, false);
  }
}

void OctomapServer::update2DMap(const OcTreeT::iterator & it, bool occupied)
{
  // update 2D map (occupied always overrides):
  if (it.getDepth() == max_tree_depth_) {
    unsigned idx = mapIdx(it.getKey());
    if (occupied) {
      gridmap_.data[mapIdx(it.getKey())] = 100;
    } else if (gridmap_.data[idx] == -1) {
      gridmap_.data[idx] = 0;
    }

  } else {
    int int_size = 1 << (max_tree_depth_ - it.getDepth());
    octomap::OcTreeKey min_key = it.getIndexKey();
    for (int dx = 0; dx < int_size; dx++) {
      int i = (min_key[0] + dx - padded_min_key_[0]) / multires_2d_scale_;
      for (int dy = 0; dy < int_size; dy++) {
        unsigned idx = mapIdx(i, (min_key[1] + dy - padded_min_key_[1]) / multires_2d_scale_);
        if (occupied) {
          gridmap_.data[idx] = 100;
        } else if (gridmap_.data[idx] == -1) {
          gridmap_.data[idx] = 0;
        }
      }
    }
  }
}


bool OctomapServer::isSpeckleNode(const octomap::OcTreeKey & n_key) const
{
  octomap::OcTreeKey key;
  bool neighbor_found = false;
  for (key[2] = n_key[2] - 1; !neighbor_found && key[2] <= n_key[2] + 1; ++key[2]) {
    for (key[1] = n_key[1] - 1; !neighbor_found && key[1] <= n_key[1] + 1; ++key[1]) {
      for (key[0] = n_key[0] - 1; !neighbor_found && key[0] <= n_key[0] + 1; ++key[0]) {
        if (key != n_key) {
          octomap::OcTreeNode * node = octree_->search(key);
          if (node && octree_->isNodeOccupied(node)) {
            // we have a neighbor=> break!
            neighbor_found = true;
          }
        }
      }
    }
  }

  return neighbor_found;
}

rcl_interfaces::msg::SetParametersResult OctomapServer::onParameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  int64_t max_tree_depth{get_parameter("max_depth").as_int()};
  update_param(parameters, "max_depth", max_tree_depth);
  max_tree_depth_ = static_cast<size_t>(max_tree_depth);
  update_param(parameters, "point_cloud_min_z", point_cloud_min_z_);
  update_param(parameters, "point_cloud_max_z", point_cloud_max_z_);
  update_param(parameters, "occupancy_min_z", occupancy_min_z_);
  update_param(parameters, "occupancy_max_z", occupancy_max_z_);
  update_param(parameters, "filter_speckles", filter_speckles_);
  update_param(parameters, "filter_ground_plane", filter_ground_plane_);
  update_param(parameters, "compress_map", compress_map_);
  update_param(parameters, "incremental_2D_projection", incremental_2D_projection_);
  update_param(parameters, "ground_filter_distance", ground_filter_distance_);
  update_param(parameters, "ground_filter_angle", ground_filter_angle_);
  update_param(parameters, "ground_filter_plane_distance", ground_filter_plane_distance_);
  update_param(parameters, "sensor_model.max_range", max_range_);
  double sensor_model_min{get_parameter("sensor_model.min").as_double()};
  update_param(parameters, "sensor_model.min", sensor_model_min);
  octree_->setClampingThresMin(sensor_model_min);
  double sensor_model_max{get_parameter("sensor_model.max").as_double()};
  update_param(parameters, "sensor_model.max", sensor_model_max);
  octree_->setClampingThresMax(sensor_model_max);
  double sensor_model_hit{get_parameter("sensor_model.hit").as_double()};
  update_param(parameters, "sensor_model.hit", sensor_model_hit);
  octree_->setProbHit(sensor_model_hit);
  double sensor_model_miss{get_parameter("sensor_model.miss").as_double()};
  update_param(parameters, "sensor_model.miss", sensor_model_miss);
  octree_->setProbMiss(sensor_model_miss);
  publishAll(now());

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void OctomapServer::adjustMapData(OccupancyGrid & map, const MapMetaData & old_map_info) const
{
  if (map.info.resolution != old_map_info.resolution) {
    RCLCPP_ERROR(get_logger(), "Resolution of map changed, cannot be adjusted");
    return;
  }

  int i_off =
    static_cast<int>((old_map_info.origin.position.x - map.info.origin.position.x) /
    map.info.resolution + 0.5);
  int j_off =
    static_cast<int>((old_map_info.origin.position.y - map.info.origin.position.y) /
    map.info.resolution + 0.5);

  if (i_off < 0 || j_off < 0 ||
    old_map_info.width + i_off > map.info.width ||
    old_map_info.height + j_off > map.info.height)
  {
    RCLCPP_ERROR(
      get_logger(), "New 2D map does not contain old map area, this case is not implemented");
    return;
  }

  OccupancyGrid::_data_type old_map_data = map.data;

  map.data.clear();
  // init to unknown:
  map.data.resize(map.info.width * map.info.height, -1);

  OccupancyGrid::_data_type::iterator from_start, from_end, to_start;

  for (size_t j = 0; j < old_map_info.height; ++j) {
    // copy chunks, row by row:
    from_start = old_map_data.begin() + j * old_map_info.width;
    from_end = from_start + old_map_info.width;
    to_start = map.data.begin() + ((j + j_off) * gridmap_.info.width + i_off);
    copy(from_start, from_end, to_start);

//    for (int i =0; i < int(old_map_info.width); ++i){
//      map.data[gridmap_.info.width*(j+j_off) +i+i_off] = oldMapData[old_map_info.width*j +i];
//    }
  }
}


ColorRGBA OctomapServer::heightMapColor(double h)
{
  ColorRGBA color;
  color.a = 1.0;
  // blend over HSV-values (more colors)

  double s = 1.0;
  double v = 1.0;

  h -= floor(h);
  h *= 6;
  int i{};
  double m{};
  double n{};
  double f{};

  i = floor(h);
  f = h - i;
  if (!(i & 1)) {
    f = 1 - f;  // if i is even
  }
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i) {
    case 6:
    case 0:
      color.r = v; color.g = n; color.b = m;
      break;
    case 1:
      color.r = n; color.g = v; color.b = m;
      break;
    case 2:
      color.r = m; color.g = v; color.b = n;
      break;
    case 3:
      color.r = m; color.g = n; color.b = v;
      break;
    case 4:
      color.r = n; color.g = m; color.b = v;
      break;
    case 5:
      color.r = v; color.g = m; color.b = n;
      break;
    default:
      color.r = 1; color.g = 0.5; color.b = 0.5;
      break;
  }

  return color;
}

#ifdef COLOR_OCTOMAP_SERVER
using ColorOctomapServer = OctomapServer;
#endif
}  // namespace octomap_server

#include <rclcpp_components/register_node_macro.hpp>
#ifdef COLOR_OCTOMAP_SERVER
RCLCPP_COMPONENTS_REGISTER_NODE(octomap_server::ColorOctomapServer)
#else
RCLCPP_COMPONENTS_REGISTER_NODE(octomap_server::OctomapServer)
#endif
