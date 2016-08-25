/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Kentaro Wada.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <algorithm>
#include <limits>
#include <string>

#include <octomap_server/LabelOctomapServer.h>

namespace octomap_server
{

bool is_equal(double a, double b, double epsilon = 1.0e-7)
{
  return std::abs(a - b) < epsilon;
}

LabelOctomapServer::LabelOctomapServer() :
  nh_(),
  pnh_(ros::NodeHandle("~")),
  sub_point_cloud_(NULL),
  sub_obj_proba_img_(NULL),
  reconfigure_server_(config_mutex_),
  octree_(NULL),
  max_range_(-1.0),
  world_frame_id_("/map"),
  use_height_map_(true),
  use_colored_map_(false),
  color_factor_(0.8),
  publish_rate_(0),
  latched_topics_(true),
  publish_free_space_(false),
  resolution_(0.05),
  tree_depth_(0),
  max_tree_depth_(0),
  occupancy_min_z_(-std::numeric_limits<double>::max()),
  occupancy_max_z_(std::numeric_limits<double>::max()),
  min_size_x_(0.0),
  min_size_y_(0.0),
  filter_speckles_(false),
  compress_map_(true),
  incremental_update_(false),
  init_config_(true)
{
  pnh_.param("frame_id", world_frame_id_, world_frame_id_);
  pnh_.param("height_map", use_height_map_, use_height_map_);
  pnh_.param("colored_map", use_colored_map_, use_colored_map_);
  pnh_.param("color_factor", color_factor_, color_factor_);

  pnh_.param("occupancy_min_z", occupancy_min_z_, occupancy_min_z_);
  pnh_.param("occupancy_max_z", occupancy_max_z_, occupancy_max_z_);
  pnh_.param("min_x_size", min_size_x_, min_size_x_);
  pnh_.param("min_y_size", min_size_y_, min_size_y_);
  pnh_.param("filter_speckles", filter_speckles_, filter_speckles_);
  pnh_.param("resolution", resolution_, resolution_);

  double prob_hit;
  double prob_miss;
  double threshold_min;
  double threshold_max;
  pnh_.param("sensor_model/hit", prob_hit, 0.7);
  pnh_.param("sensor_model/miss", prob_miss, 0.4);
  pnh_.param("sensor_model/min", threshold_min, 0.12);
  pnh_.param("sensor_model/max", threshold_max, 0.97);
  pnh_.param("sensor_model/max_range", max_range_, max_range_);

  pnh_.param("compress_map", compress_map_, compress_map_);
  pnh_.param("incremental_2D_projection", incremental_update_, incremental_update_);

  if (use_height_map_ && use_colored_map_)
  {
    ROS_WARN("You enabled both height map and RGB color registration. This is contradictory. Defaulting to height map.");
    use_colored_map_ = false;
  }

  if (use_colored_map_)
  {
#ifdef COLOR_OCTOMAP_SERVER
    ROS_INFO("Using RGB color registration (if information available)");
#else
    ROS_ERROR("Colored map requested in launch file - node not running/compiled to support colors, "
              "please define COLOR_OCTOMAP_SERVER and recompile or launch the octomap_color_server node");
#endif
  }

  // initialize octomap object & params
  octree_ = new OcTreeT(resolution_);
  octree_->setProbHit(prob_hit);
  octree_->setProbMiss(prob_miss);
  octree_->setClampingThresMin(threshold_min);
  octree_->setClampingThresMax(threshold_max);
  tree_depth_ = octree_->getTreeDepth();
  max_tree_depth_ = tree_depth_;
  gridmap_.info.resolution = resolution_;

  double r, g, b, a;
  pnh_.param("color/r", r, 0.0);
  pnh_.param("color/g", g, 0.0);
  pnh_.param("color/b", b, 1.0);
  pnh_.param("color/a", a, 1.0);
  color_.r = r;
  color_.g = g;
  color_.b = b;
  color_.a = a;

  pnh_.param("color_free/r", r, 0.0);
  pnh_.param("color_free/g", g, 1.0);
  pnh_.param("color_free/b", b, 0.0);
  pnh_.param("color_free/a", a, 1.0);
  color_free_.r = r;
  color_free_.g = g;
  color_free_.b = b;
  color_free_.a = a;

  pnh_.param("publish_free_space", publish_free_space_, publish_free_space_);

  pnh_.param("latch", latched_topics_, latched_topics_);
  if (latched_topics_)
  {
    ROS_INFO("Publishing latched (single publish will take longer, all topics are prepared)");
  }
  else
  {
    ROS_INFO("Publishing non-latched (topics are only prepared as needed, will only be re-published on map change");
  }

  pnh_.param("publish_rate", publish_rate_, publish_rate_);
  if (publish_rate_ > 0)
  {
    timer_periodical_publish_ = pnh_.createTimer(
        ros::Duration(1.0 / publish_rate_),
        &LabelOctomapServer::periodicalPublishCallback,
        this,
        /*oneshot=*/false);
  }

  pub_marker_ = nh_.advertise<visualization_msgs::MarkerArray>("occupied_cells_vis_array", 1, latched_topics_);
  pub_binary_map_ = nh_.advertise<octomap_msgs::Octomap>("octomap_binary", 1, latched_topics_);
  pub_full_map_ = nh_.advertise<octomap_msgs::Octomap>("octomap_full", 1, latched_topics_);
  pub_point_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("octomap_point_cloud_centers", 1, latched_topics_);
  pub_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("projected_map", 5, latched_topics_);
  pub_fmarker_ = nh_.advertise<visualization_msgs::MarkerArray>("free_cells_vis_array", 1, latched_topics_);

  sub_point_cloud_ = new message_filters::Subscriber<sensor_msgs::PointCloud2>(nh_, "cloud_in", 5);
  sub_obj_proba_img_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_, "proba_image_in", 5);
  async_ = new message_filters::Synchronizer<ApproximateSyncPolicy>(100);
  async_->connectInput(*sub_point_cloud_, *sub_obj_proba_img_);
  async_->registerCallback(boost::bind(&LabelOctomapServer::insertCallback, this, _1, _2));

  srv_octomap_binary_ = nh_.advertiseService("octomap_binary", &LabelOctomapServer::octomapBinarySrv, this);
  srv_octomap_full_ = nh_.advertiseService("octomap_full", &LabelOctomapServer::octomapFullSrv, this);
  srv_clear_bbx_ = pnh_.advertiseService("clear_bbx", &LabelOctomapServer::clearBBXSrv, this);
  srv_reset_ = pnh_.advertiseService("reset", &LabelOctomapServer::resetSrv, this);

  reconfigure_server_.setCallback(boost::bind(&LabelOctomapServer::reconfigureCallback, this, _1, _2));
}

LabelOctomapServer::~LabelOctomapServer()
{
  if (sub_obj_proba_img_)
  {
    delete sub_obj_proba_img_;
    sub_obj_proba_img_ = NULL;
  }

  if (sub_point_cloud_)
  {
    delete sub_point_cloud_;
    sub_point_cloud_ = NULL;
  }

  if (octree_)
  {
    delete octree_;
    octree_ = NULL;
  }
}

bool LabelOctomapServer::openFile(const std::string& filename)
{
  if (filename.length() <= 3)
  {
    return false;
  }

  std::string suffix = filename.substr(filename.length()-3, 3);
  if (suffix== ".bt")
  {
    if (!octree_->readBinary(filename))
    {
      return false;
    }
  }
  else if (suffix == ".ot")
  {
    octomap::AbstractOcTree* tree = octomap::AbstractOcTree::read(filename);
    if (!tree)
    {
      return false;
    }
    if (octree_)
    {
      delete octree_;
      octree_ = NULL;
    }
    octree_ = dynamic_cast<OcTreeT*>(tree);
    if (!octree_)
    {
      ROS_ERROR("Could not read OcTree in file, currently there are no other types supported in .ot");
      return false;
    }
  }
  else
  {
    return false;
  }

  ROS_INFO("Octomap file %s loaded (%zu nodes).", filename.c_str(), octree_->size());

  tree_depth_ = octree_->getTreeDepth();
  max_tree_depth_ = tree_depth_;
  resolution_ = octree_->getResolution();
  gridmap_.info.resolution = resolution_;
  double min_x, min_y, min_z;
  double max_x, max_y, max_z;
  octree_->getMetricMin(min_x, min_y, min_z);
  octree_->getMetricMax(max_x, max_y, max_z);

  update_bbx_min_[0] = octree_->coordToKey(min_x);
  update_bbx_min_[1] = octree_->coordToKey(min_y);
  update_bbx_min_[2] = octree_->coordToKey(min_z);

  update_bbx_max_[0] = octree_->coordToKey(max_x);
  update_bbx_max_[1] = octree_->coordToKey(max_y);
  update_bbx_max_[2] = octree_->coordToKey(max_z);

  publishAll();

  return true;
}

void LabelOctomapServer::insertCallback(
    const sensor_msgs::PointCloud2::ConstPtr& cloud,
    const sensor_msgs::Image::ConstPtr& imgmsg)
{
  insertScan(cloud, imgmsg);
  publishAll(cloud->header.stamp);
}

void LabelOctomapServer::insertScan(
    const sensor_msgs::PointCloud2::ConstPtr& cloud,
    const sensor_msgs::Image::ConstPtr& imgmsg)
{
  if (!(cloud->height == imgmsg->height && cloud->width == imgmsg->width)) {
    ROS_ERROR("Input point cloud and image has different size. cloud: (%d, %d), image: (%d, %d)",
              cloud->width, cloud->height, imgmsg->width, imgmsg->height);
    return;
  }

  // Get transform of sensor to the world
  tf::StampedTransform sensor_to_world_tf;
  try
  {
    tf_listener_.lookupTransform(world_frame_id_, cloud->header.frame_id, cloud->header.stamp, sensor_to_world_tf);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
    return;
  }

  // Get sensor origin
  tf::Point sensor_origin_tf = sensor_to_world_tf.getOrigin();
  octomap::point3d sensor_origin = octomap::pointTfToOctomap(sensor_origin_tf);

  PCLPointCloud pc;
  pcl::fromROSMsg(*cloud, pc);

  // transform clouds to world frame for insertion
  Eigen::Matrix4f sensor_to_world;
  pcl_ros::transformAsMatrix(sensor_to_world_tf, sensor_to_world);
  pcl::transformPointCloud(pc, pc, sensor_to_world);

  cv::Mat proba_img = cv_bridge::toCvCopy(imgmsg, imgmsg->encoding)->image;

  if (!octree_->coordToKeyChecked(sensor_origin, update_bbx_min_) ||
      !octree_->coordToKeyChecked(sensor_origin, update_bbx_max_))
  {
    ROS_ERROR_STREAM("Could not generate Key for origin " << sensor_origin);
  }

#ifdef COLOR_OCTOMAP_SERVER
  unsigned char* colors = new unsigned char[3];
#endif

  // instead of direct scan insertion, compute update to filter ground:
  octomap::KeySet free_cells;
  octomap::KeySet occupied_cells;

  // all other points: free on ray, occupied on endpoint:
  for (size_t index = 0; index < pc.points.size(); index++)
  {
    int width_index = index % cloud->width;
    int height_index = index / cloud->width;
    if (isnan(pc.points[index].x) || isnan(pc.points[index].y) || isnan(pc.points[index].z))
    {
      // Skip NaN points
      continue;
    }
    octomap::point3d point(pc.points[index].x, pc.points[index].y, pc.points[index].z);
    float label_proba = proba_img.at<float>(height_index, width_index);
    // maxrange check
    if ((max_range_ < 0.0) || ((point - sensor_origin).norm() <= max_range_))
    {
      // free cells
      if (octree_->computeRayKeys(sensor_origin, point, key_ray_))
      {
        free_cells.insert(key_ray_.begin(), key_ray_.end());
      }
      // occupied endpoint
      octomap::OcTreeKey key;
      if (octree_->coordToKeyChecked(point, key))
      {
        octree_->updateNode(key, octomap::logodds(label_proba));

        updateMinKey(key, update_bbx_min_);
        updateMaxKey(key, update_bbx_max_);
      }
    }
    else
    {
      // ray longer than maxrange:
      octomap::point3d new_end = sensor_origin + (point - sensor_origin).normalized() * max_range_;
      if (octree_->computeRayKeys(sensor_origin, new_end, key_ray_))
      {
        free_cells.insert(key_ray_.begin(), key_ray_.end());

        octomap::OcTreeKey end_key;
        if (octree_->coordToKeyChecked(new_end, end_key))
        {
          updateMinKey(end_key, update_bbx_min_);
          updateMaxKey(end_key, update_bbx_max_);
        }
        else
        {
          ROS_ERROR_STREAM("Could not generate Key for endpoint " << new_end);
        }
      }
    }
  }

  // mark free cells only if not seen occupied in this cloud
  for (octomap::KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it) {
    if (occupied_cells.find(*it) == occupied_cells.end()) {
      octree_->updateNode(*it, false);  // set object probability as 0.5
    }
  }

  // TODO: eval lazy+updateInner vs. proper insertion
  // non-lazy by default (updateInnerOccupancy() too slow for large maps)
  // octree_->updateInnerOccupancy();
  octomap::point3d min_point, max_point;
  ROS_DEBUG_STREAM("Bounding box keys (before): " << update_bbx_min_[0] << " " <<update_bbx_min_[1] << " " <<
                   update_bbx_min_[2] << " / " <<update_bbx_max_[0] << " "<< update_bbx_max_[1] << " " << update_bbx_max_[2]);

  // TODO: we could also limit the bbx to be within the map bounds here (see publishing check)
  min_point = octree_->keyToCoord(update_bbx_min_);
  max_point = octree_->keyToCoord(update_bbx_max_);
  ROS_DEBUG_STREAM("Updated area bounding box: " << min_point << " - " << max_point);
  ROS_DEBUG_STREAM("Bounding box keys (after): " << update_bbx_min_[0] << " " <<update_bbx_min_[1] << " " <<
                   update_bbx_min_[2] << " / " <<update_bbx_max_[0] << " " << update_bbx_max_[1] << " " << update_bbx_max_[2]);

  if (compress_map_)
  {
    octree_->prune();
  }
}

void LabelOctomapServer::periodicalPublishCallback(const ros::TimerEvent& event)
{
  publishAll(event.current_real);
}

void LabelOctomapServer::publishAll(const ros::Time& rostime)
{
  ros::WallTime start_time = ros::WallTime::now();

  // TODO: estimate num occ. voxels for size of arrays (reserve)
  if (octree_->size() <= 1)
  {
    ROS_WARN("Nothing to publish, octree is empty");
    return;
  }

  bool publish_free_marker_array = publish_free_space_ && (latched_topics_ || pub_fmarker_.getNumSubscribers() > 0);
  bool publish_marker_array = (latched_topics_ || pub_marker_.getNumSubscribers() > 0);
  bool publish_point_cloud = (latched_topics_ || pub_point_cloud_.getNumSubscribers() > 0);
  bool publish_binary_map = (latched_topics_ || pub_binary_map_.getNumSubscribers() > 0);
  bool publish_full_map = (latched_topics_ || pub_full_map_.getNumSubscribers() > 0);
  publish_2d_map_ = (latched_topics_ || pub_map_.getNumSubscribers() > 0);

  // init markers for free space:
  visualization_msgs::MarkerArray free_nodes_vis;
  // each array stores all cubes of a different size, one for each depth level:
  free_nodes_vis.markers.resize(tree_depth_+1);

  geometry_msgs::Pose pose;
  pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  // init markers:
  visualization_msgs::MarkerArray occupied_nodes_vis;
  // each array stores all cubes of a different size, one for each depth level:
  occupied_nodes_vis.markers.resize(tree_depth_+1);

  // init pointcloud:
  pcl::PointCloud<PCLPoint> pcl_cloud;

  // call pre-traversal hook:
  handlePreNodeTraversal(rostime);

  // now, traverse all leafs in the tree:
  for (OcTreeT::iterator it = octree_->begin(max_tree_depth_), end = octree_->end();
       it != end; ++it)
  {
    bool in_update_bbx = isInUpdateBBX(it);

    // call general hook:
    handleNode(it);
    if (in_update_bbx)
    {
      handleNodeInBBX(it);
    }

    if (octree_->isNodeOccupied(*it))
    {
      double z = it.getZ();
      if (z > occupancy_min_z_ && z < occupancy_max_z_)
      {
        double size = it.getSize();
        double x = it.getX();
        double y = it.getY();
#ifdef COLOR_OCTOMAP_SERVER
        int r = it->getColor().r;
        int g = it->getColor().g;
        int b = it->getColor().b;
#endif

        // Ignore speckles in the map:
        if (filter_speckles_ && (it.getDepth() == tree_depth_ + 1) && isSpeckleNode(it.getKey()))
        {
          ROS_DEBUG("Ignoring single speckle at (%f, %f, %f)", x, y, z);
          continue;
        } // else: current octree node is no speckle, send it out

        handleOccupiedNode(it);
        if (in_update_bbx)
        {
          handleOccupiedNodeInBBX(it);
        }

        // create marker
        if (publish_marker_array)
        {
          unsigned idx = it.getDepth();
          assert(idx < occupied_nodes_vis.markers.size());

          geometry_msgs::Point cube_center;
          cube_center.x = x;
          cube_center.y = y;
          cube_center.z = z;

          occupied_nodes_vis.markers[idx].points.push_back(cube_center);
          if (use_height_map_)
          {
            double min_x, min_y, min_z, max_x, max_y, max_z;
            octree_->getMetricMin(min_x, min_y, min_z);
            octree_->getMetricMax(max_x, max_y, max_z);

            double h = (1.0 - std::min(std::max((cube_center.z-min_z)/ (max_z - min_z), 0.0), 1.0)) *color_factor_;
            occupied_nodes_vis.markers[idx].colors.push_back(heightMapColor(h));
          }

#ifdef COLOR_OCTOMAP_SERVER
          if (use_colored_map_)
          {
            // TODO/EVALUATE: potentially use occupancy as measure for alpha channel?
            std_msgs::ColorRGBA color;
            color.r = (r / 255.);
            color.g = (g / 255.);
            color.b = (b / 255.);
            color.a = 1.;
            occupied_nodes_vis.markers[idx].colors.push_back(color);
          }
#endif
        }

        // insert into pointcloud:
        if (publish_point_cloud)
        {
#ifdef COLOR_OCTOMAP_SERVER
          PCLPoint _point = PCLPoint();
          _point.x = x; _point.y = y; _point.z = z;
          _point.r = r; _point.g = g; _point.b = b;
          pcl_cloud.push_back(_point);
#else
          pcl_cloud.push_back(PCLPoint(x, y, z));
#endif
        }
      }
    }
    else
    {
      // node not occupied => mark as free in 2D map if unknown so far
      double z = it.getZ();
      if (z > occupancy_min_z_ && z < occupancy_max_z_)
      {
        handleFreeNode(it);
        if (in_update_bbx)
        {
          handleFreeNodeInBBX(it);
        }

        if (publish_free_space_)
        {
          double x = it.getX();
          double y = it.getY();

          // create marker for free space:
          if (publish_free_marker_array)
          {
            unsigned idx = it.getDepth();
            assert(idx < free_nodes_vis.markers.size());

            geometry_msgs::Point cube_center;
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
  if (publish_marker_array)
  {
    for (unsigned i= 0; i < occupied_nodes_vis.markers.size(); ++i)
    {
      double size = octree_->getNodeSize(i);

      occupied_nodes_vis.markers[i].header.frame_id = world_frame_id_;
      occupied_nodes_vis.markers[i].header.stamp = rostime;
      occupied_nodes_vis.markers[i].ns = "map";
      occupied_nodes_vis.markers[i].id = i;
      occupied_nodes_vis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
      occupied_nodes_vis.markers[i].scale.x = size;
      occupied_nodes_vis.markers[i].scale.y = size;
      occupied_nodes_vis.markers[i].scale.z = size;
      if (!use_colored_map_)
      {
        occupied_nodes_vis.markers[i].color = color_;
      }

      if (occupied_nodes_vis.markers[i].points.size() > 0)
      {
        occupied_nodes_vis.markers[i].action = visualization_msgs::Marker::ADD;
      }
      else
      {
        occupied_nodes_vis.markers[i].action = visualization_msgs::Marker::DELETE;
      }
    }

    pub_marker_.publish(occupied_nodes_vis);
  }


  // finish FreeMarkerArray
  if (publish_free_marker_array)
  {
    for (unsigned i= 0; i < free_nodes_vis.markers.size(); ++i)
    {
      double size = octree_->getNodeSize(i);

      free_nodes_vis.markers[i].header.frame_id = world_frame_id_;
      free_nodes_vis.markers[i].header.stamp = rostime;
      free_nodes_vis.markers[i].ns = "map";
      free_nodes_vis.markers[i].id = i;
      free_nodes_vis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
      free_nodes_vis.markers[i].scale.x = size;
      free_nodes_vis.markers[i].scale.y = size;
      free_nodes_vis.markers[i].scale.z = size;
      free_nodes_vis.markers[i].color = color_free_;

      if (free_nodes_vis.markers[i].points.size() > 0)
      {
        free_nodes_vis.markers[i].action = visualization_msgs::Marker::ADD;
      }
      else
      {
        free_nodes_vis.markers[i].action = visualization_msgs::Marker::DELETE;
      }
    }

    pub_fmarker_.publish(free_nodes_vis);
  }

  // finish pointcloud:
  if (publish_point_cloud)
  {
    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg(pcl_cloud, cloud);
    cloud.header.frame_id = world_frame_id_;
    cloud.header.stamp = rostime;
    cloud.is_dense = false;
    pub_point_cloud_.publish(cloud);
  }

  if (publish_binary_map)
  {
    publishBinaryOctoMap(rostime);
  }

  if (publish_full_map)
  {
    publishFullOctoMap(rostime);
  }

  double total_elapsed = (ros::WallTime::now() - start_time).toSec();
  ROS_DEBUG("Map publishing in LabelOctomapServer took %f sec", total_elapsed);
}

bool LabelOctomapServer::octomapBinarySrv(OctomapSrv::Request  &req,
    OctomapSrv::Response &res)
{
  ros::WallTime start_time = ros::WallTime::now();
  ROS_INFO("Sending binary map data on service request");
  res.map.header.frame_id = world_frame_id_;
  res.map.header.stamp = ros::Time::now();
  if (!octomap_msgs::binaryMapToMsg(*octree_, res.map))
    return false;

  double total_elapsed = (ros::WallTime::now() - start_time).toSec();
  ROS_INFO("Binary octomap sent in %f sec", total_elapsed);
  return true;
}

bool LabelOctomapServer::octomapFullSrv(OctomapSrv::Request  &req,
    OctomapSrv::Response &res)
{
  ROS_INFO("Sending full map data on service request");
  res.map.header.frame_id = world_frame_id_;
  res.map.header.stamp = ros::Time::now();


  if (!octomap_msgs::fullMapToMsg(*octree_, res.map))
    return false;

  return true;
}

bool LabelOctomapServer::clearBBXSrv(BBXSrv::Request& req, BBXSrv::Response& resp)
{
  octomap::point3d min = octomap::pointMsgToOctomap(req.min);
  octomap::point3d max = octomap::pointMsgToOctomap(req.max);

  double threshold_min = octree_->getClampingThresMin();
  for (OcTreeT::leaf_bbx_iterator it = octree_->begin_leafs_bbx(min, max),
      end = octree_->end_leafs_bbx(); it != end; ++it)
  {
    it->setLogOdds(octomap::logodds(threshold_min));
    // octree_->updateNode(it.getKey(), -6.0f);
  }
  // TODO: eval which is faster (setLogOdds+updateInner or updateNode)
  octree_->updateInnerOccupancy();

  publishAll(ros::Time::now());

  return true;
}

bool LabelOctomapServer::resetSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp)
{
  visualization_msgs::MarkerArray occupied_nodes_vis;
  occupied_nodes_vis.markers.resize(tree_depth_ +1);
  ros::Time rostime = ros::Time::now();
  octree_->clear();
  // clear 2D map:
  gridmap_.data.clear();
  gridmap_.info.height = 0.0;
  gridmap_.info.width = 0.0;
  gridmap_.info.resolution = 0.0;
  gridmap_.info.origin.position.x = 0.0;
  gridmap_.info.origin.position.y = 0.0;

  ROS_INFO("Cleared octomap");
  publishAll(rostime);

  publishBinaryOctoMap(rostime);
  for (unsigned i= 0; i < occupied_nodes_vis.markers.size(); ++i)
  {
    occupied_nodes_vis.markers[i].header.frame_id = world_frame_id_;
    occupied_nodes_vis.markers[i].header.stamp = rostime;
    occupied_nodes_vis.markers[i].ns = "map";
    occupied_nodes_vis.markers[i].id = i;
    occupied_nodes_vis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    occupied_nodes_vis.markers[i].action = visualization_msgs::Marker::DELETE;
  }

  pub_marker_.publish(occupied_nodes_vis);

  visualization_msgs::MarkerArray free_nodes_vis;
  free_nodes_vis.markers.resize(tree_depth_ +1);

  for (unsigned i= 0; i < free_nodes_vis.markers.size(); ++i)
  {
    free_nodes_vis.markers[i].header.frame_id = world_frame_id_;
    free_nodes_vis.markers[i].header.stamp = rostime;
    free_nodes_vis.markers[i].ns = "map";
    free_nodes_vis.markers[i].id = i;
    free_nodes_vis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    free_nodes_vis.markers[i].action = visualization_msgs::Marker::DELETE;
  }
  pub_fmarker_.publish(free_nodes_vis);

  return true;
}

void LabelOctomapServer::publishBinaryOctoMap(const ros::Time& rostime) const
{
  octomap_msgs::Octomap map;
  map.header.frame_id = world_frame_id_;
  map.header.stamp = rostime;

  if (octomap_msgs::binaryMapToMsg(*octree_, map))
  {
    pub_binary_map_.publish(map);
  }
  else
  {
    ROS_ERROR("Error serializing OctoMap");
  }
}

void LabelOctomapServer::publishFullOctoMap(const ros::Time& rostime) const
{
  octomap_msgs::Octomap map;
  map.header.frame_id = world_frame_id_;
  map.header.stamp = rostime;

  if (octomap_msgs::fullMapToMsg(*octree_, map))
  {
    pub_full_map_.publish(map);
  }
  else
  {
    ROS_ERROR("Error serializing OctoMap");
  }
}

void LabelOctomapServer::handlePreNodeTraversal(const ros::Time& rostime)
{
  if (publish_2d_map_)
  {
    // init projected 2D map:
    gridmap_.header.frame_id = world_frame_id_;
    gridmap_.header.stamp = rostime;
    nav_msgs::MapMetaData old_map_info = gridmap_.info;

    // TODO: move most of this stuff into c'tor and init map only once (adjust if size changes)
    double min_x, min_y, min_z, max_x, max_y, max_z;
    octree_->getMetricMin(min_x, min_y, min_z);
    octree_->getMetricMax(max_x, max_y, max_z);

    octomap::point3d min_point(min_x, min_y, min_z);
    octomap::point3d max_point(max_x, max_y, max_z);
    octomap::OcTreeKey min_key = octree_->coordToKey(min_point, max_tree_depth_);
    octomap::OcTreeKey max_key = octree_->coordToKey(max_point, max_tree_depth_);

    ROS_DEBUG("MinKey: %d %d %d / MaxKey: %d %d %d", min_key[0], min_key[1], min_key[2], max_key[0], max_key[1], max_key[2]);

    // add padding if requested (= new min/max_points in x&y):
    double half_padded_x = 0.5 * min_size_x_;
    double half_padded_y = 0.5 * min_size_y_;
    min_x = std::min(min_x, -half_padded_x);
    max_x = std::max(max_x, half_padded_x);
    min_y = std::min(min_y, -half_padded_y);
    max_y = std::max(max_y, half_padded_y);
    min_point = octomap::point3d(min_x, min_y, min_z);
    max_point = octomap::point3d(max_x, max_y, max_z);

    octomap::OcTreeKey padded_max_key;
    if (!octree_->coordToKeyChecked(min_point, max_tree_depth_, padded_min_key_))
    {
      ROS_ERROR("Could not create padded min OcTree key at %f %f %f", min_point.x(), min_point.y(), min_point.z());
      return;
    }
    if (!octree_->coordToKeyChecked(max_point, max_tree_depth_, padded_max_key))
    {
      ROS_ERROR("Could not create padded max OcTree key at %f %f %f", max_point.x(), max_point.y(), max_point.z());
      return;
    }

    ROS_DEBUG("Padded MinKey: %d %d %d / padded MaxKey: %d %d %d",
              padded_min_key_[0], padded_min_key_[1], padded_min_key_[2],
              padded_max_key[0], padded_max_key[1], padded_max_key[2]);
    assert(padded_max_key[0] >= max_key[0] && padded_max_key[1] >= max_key[1]);

    multires_2d_scale_ = 1 << (tree_depth_ - max_tree_depth_);
    gridmap_.info.width = (padded_max_key[0] - padded_min_key_[0])/multires_2d_scale_ +1;
    gridmap_.info.height = (padded_max_key[1] - padded_min_key_[1])/multires_2d_scale_ +1;

    int mapOriginX = min_key[0] - padded_min_key_[0];
    int mapOriginY = min_key[1] - padded_min_key_[1];
    assert(mapOriginX >= 0 && mapOriginY >= 0);

    // might not exactly be min / max of octree:
    octomap::point3d origin = octree_->keyToCoord(padded_min_key_, tree_depth_);
    double gridRes = octree_->getNodeSize(max_tree_depth_);
    project_complete_map_ = (!incremental_update_ || (std::abs(gridRes-gridmap_.info.resolution) > 1e-6));
    gridmap_.info.resolution = gridRes;
    gridmap_.info.origin.position.x = origin.x() - gridRes * 0.5;
    gridmap_.info.origin.position.y = origin.y() - gridRes * 0.5;
    if (max_tree_depth_ != tree_depth_)
    {
      gridmap_.info.origin.position.x -= resolution_ / 2.0;
      gridmap_.info.origin.position.y -= resolution_ / 2.0;
    }

    // workaround for  multires. projection not working properly for inner nodes:
    // force re-building complete map
    if (max_tree_depth_ < tree_depth_)
    {
      project_complete_map_ = true;
    }

    if (project_complete_map_)
    {
      ROS_DEBUG("Rebuilding complete 2D map");
      gridmap_.data.clear();
      // init to unknown:
      gridmap_.data.resize(gridmap_.info.width * gridmap_.info.height, -1);
    }
    else
    {
      if (mapChanged(old_map_info, gridmap_.info))
      {
        ROS_DEBUG("2D grid map size changed to %dx%d", gridmap_.info.width, gridmap_.info.height);
        adjustMapData(gridmap_, old_map_info);
      }
      size_t map_update_bbx_min_x = std::max(0,
          (static_cast<int>(update_bbx_min_[0]) - static_cast<int>(padded_min_key_[0]))
          / static_cast<int>(multires_2d_scale_));
      size_t map_update_bbx_min_y = std::max(0,
          (static_cast<int>(update_bbx_min_[1]) - static_cast<int>(padded_min_key_[1]))
          / static_cast<int>(multires_2d_scale_));
      size_t map_update_bbx_max_x = std::min(static_cast<int>(gridmap_.info.width-1),
          (static_cast<int>(update_bbx_max_[0]) - static_cast<int>(padded_min_key_[0]))
          / static_cast<int>(multires_2d_scale_));
      size_t map_update_bbx_max_y = std::min(static_cast<int>(gridmap_.info.height-1),
          (static_cast<int>(update_bbx_max_[1]) - static_cast<int>(padded_min_key_[1]))
          / static_cast<int>(multires_2d_scale_));

      assert(map_update_bbx_max_x > map_update_bbx_min_x);
      assert(map_update_bbx_max_y > map_update_bbx_min_y);

      size_t num_cols = map_update_bbx_max_x - map_update_bbx_min_x + 1;

      // test for max idx:
      uint max_idx = gridmap_.info.width*map_update_bbx_max_y + map_update_bbx_max_x;
      if (max_idx  >= gridmap_.data.size())
      {
        ROS_ERROR("BBX index not valid: %d (max index %zu for size %d x %d) update-BBX is: [%zu %zu]-[%zu %zu]",
                  max_idx, gridmap_.data.size(), gridmap_.info.width, gridmap_.info.height,
                  map_update_bbx_min_x, map_update_bbx_min_y, map_update_bbx_max_x, map_update_bbx_max_y);
      }

      // reset proj. 2D map in bounding box:
      for (unsigned int j = map_update_bbx_min_y; j <= map_update_bbx_max_y; ++j)
      {
        std::fill_n(gridmap_.data.begin() + gridmap_.info.width*j+map_update_bbx_min_x, num_cols, -1);
      }
    }
  }
}

void LabelOctomapServer::handlePostNodeTraversal(const ros::Time& rostime)
{
  if (publish_2d_map_)
  {
    pub_map_.publish(gridmap_);
  }
}

void LabelOctomapServer::handleOccupiedNode(const OcTreeT::iterator& it)
{
  if (publish_2d_map_ && project_complete_map_)
  {
    update2DMap(it, true);
  }
}

void LabelOctomapServer::handleFreeNode(const OcTreeT::iterator& it)
{
  if (publish_2d_map_ && project_complete_map_)
  {
    update2DMap(it, false);
  }
}

void LabelOctomapServer::handleOccupiedNodeInBBX(const OcTreeT::iterator& it)
{
  if (publish_2d_map_ && !project_complete_map_)
  {
    update2DMap(it, true);
  }
}

void LabelOctomapServer::handleFreeNodeInBBX(const OcTreeT::iterator& it)
{
  if (publish_2d_map_ && !project_complete_map_)
  {
    update2DMap(it, false);
  }
}

void LabelOctomapServer::update2DMap(const OcTreeT::iterator& it, bool occupied)
{
  // update 2D map (occupied always overrides):

  if (it.getDepth() == max_tree_depth_)
  {
    unsigned idx = mapIdx(it.getKey());
    if (occupied)
    {
      gridmap_.data[mapIdx(it.getKey())] = 100;
    }
    else if (gridmap_.data[idx] == -1)
    {
      gridmap_.data[idx] = 0;
    }
  }
  else
  {
    int intSize = 1 << (max_tree_depth_ - it.getDepth());
    octomap::OcTreeKey min_key = it.getIndexKey();
    for (int dx = 0; dx < intSize; dx++)
    {
      int i = (min_key[0] + dx - padded_min_key_[0]) / multires_2d_scale_;
      for (int dy = 0; dy < intSize; dy++)
      {
        unsigned idx = mapIdx(i, (min_key[1] + dy - padded_min_key_[1]) / multires_2d_scale_);
        if (occupied)
        {
          gridmap_.data[idx] = 100;
        }
        else if (gridmap_.data[idx] == -1)
        {
          gridmap_.data[idx] = 0;
        }
      }
    }
  }
}

bool LabelOctomapServer::isSpeckleNode(const octomap::OcTreeKey& nKey) const
{
  octomap::OcTreeKey key;
  bool neighborFound = false;
  for (key[2] = nKey[2] - 1; !neighborFound && key[2] <= nKey[2] + 1; ++key[2])
  {
    for (key[1] = nKey[1] - 1; !neighborFound && key[1] <= nKey[1] + 1; ++key[1])
    {
      for (key[0] = nKey[0] - 1; !neighborFound && key[0] <= nKey[0] + 1; ++key[0])
      {
        if (key != nKey)
        {
          octomap::OcTreeNode* node = octree_->search(key);
          if (node && octree_->isNodeOccupied(node))
          {
            // we have a neighbor => break!
            neighborFound = true;
          }
        }
      }
    }
  }

  return neighborFound;
}

void LabelOctomapServer::reconfigureCallback(octomap_server::OctomapServerConfig& config, uint32_t level)
{
  if (max_tree_depth_ != unsigned(config.max_depth))
  {
    max_tree_depth_ = unsigned(config.max_depth);
  }
  else
  {
    occupancy_min_z_             = config.occupancy_min_z;
    occupancy_max_z_             = config.occupancy_max_z;
    filter_speckles_            = config.filter_speckles;
    compress_map_               = config.compress_map;
    incremental_update_         = config.incremental_2D_projection;

    // Parameters with a namespace require an special treatment at the beginning, as dynamic reconfigure
    // will overwrite them because the server is not able to match parameters' names.
    if (init_config_)
    {
      // If parameters do not have the default value, dynamic reconfigure server should be updated.
      if (!is_equal(max_range_, -1.0))
      {
        config.sensor_model_max_range = max_range_;
      }
      if (!is_equal(octree_->getProbHit(), 0.7))
      {
        config.sensor_model_hit = octree_->getProbHit();
      }
      if (!is_equal(octree_->getProbMiss(), 0.4))
      {
        config.sensor_model_miss = octree_->getProbMiss();
      }
      if (!is_equal(octree_->getClampingThresMin(), 0.12))
      {
        config.sensor_model_min = octree_->getClampingThresMin();
      }
      if (!is_equal(octree_->getClampingThresMax(), 0.97))
      {
        config.sensor_model_max = octree_->getClampingThresMax();
      }
      init_config_ = false;

      boost::recursive_mutex::scoped_lock reconf_lock(config_mutex_);
      reconfigure_server_.updateConfig(config);
    }
    else
    {
      max_range_ = config.sensor_model_max_range;
      octree_->setClampingThresMin(config.sensor_model_min);
      octree_->setClampingThresMax(config.sensor_model_max);

      // Checking values that might create unexpected behaviors.
      if (is_equal(config.sensor_model_hit, 1.0))
      {
        config.sensor_model_hit -= 1.0e-6;
      }
      octree_->setProbHit(config.sensor_model_hit);
      if (is_equal(config.sensor_model_miss, 0.0))
      {
        config.sensor_model_miss += 1.0e-6;
      }
      octree_->setProbMiss(config.sensor_model_miss);
    }
  }
  publishAll();
}

void LabelOctomapServer::adjustMapData(nav_msgs::OccupancyGrid& map, const nav_msgs::MapMetaData& old_map_info) const
{
  if (map.info.resolution != old_map_info.resolution)
  {
    ROS_ERROR("Resolution of map changed, cannot be adjusted");
    return;
  }

  int i_off = static_cast<int>((old_map_info.origin.position.x - map.info.origin.position.x) / map.info.resolution + 0.5);
  int j_off = static_cast<int>((old_map_info.origin.position.y - map.info.origin.position.y)/map.info.resolution + 0.5);

  if (i_off < 0 || j_off < 0
      || old_map_info.width  + i_off > map.info.width
      || old_map_info.height + j_off > map.info.height)
  {
    ROS_ERROR("New 2D map does not contain old map area, this case is not implemented");
    return;
  }

  nav_msgs::OccupancyGrid::_data_type old_map_data = map.data;

  map.data.clear();
  // init to unknown:
  map.data.resize(map.info.width * map.info.height, -1);

  nav_msgs::OccupancyGrid::_data_type::iterator from_start;
  nav_msgs::OccupancyGrid::_data_type::iterator from_end;
  nav_msgs::OccupancyGrid::_data_type::iterator to_start;

  for (int j = 0; j < static_cast<int>(old_map_info.height); ++j)
  {
    // copy chunks, row by row:
    from_start = old_map_data.begin() + j * old_map_info.width;
    from_end = from_start + old_map_info.width;
    to_start = map.data.begin() + ((j + j_off) * gridmap_.info.width + i_off);
    copy(from_start, from_end, to_start);

    //    for (int i =0; i < int(old_map_info.width); ++i){
    //      map.data[gridmap_.info.width*(j+j_off) +i+i_off] = old_map_data[old_map_info.width*j +i];
    //    }
  }
}

std_msgs::ColorRGBA LabelOctomapServer::heightMapColor(double h)
{
  std_msgs::ColorRGBA color;
  color.a = 1.0;
  // blend over HSV-values (more colors)

  double s = 1.0;
  double v = 1.0;

  h -= floor(h);
  h *= 6;
  int i;
  double m, n, f;

  i = floor(h);
  f = h - i;
  if (!(i & 1))
    f = 1 - f; // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i)
  {
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

}  // namespace octomap_server
