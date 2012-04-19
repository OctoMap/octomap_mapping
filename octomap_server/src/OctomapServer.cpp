/**
* octomap_server: A Tool to serve 3D OctoMaps in ROS (binary and as visualization)
* (inspired by the ROS map_saver)
* @author A. Hornung, University of Freiburg, Copyright (C) 2010-2012.
* @see http://octomap.sourceforge.net/
* License: BSD
*/

/*
 * Copyright (c) 2010-2012, A. Hornung, University of Freiburg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the University of Freiburg nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <octomap_server/OctomapServer.h>

using namespace octomap;

namespace octomap_server{

OctomapServer::OctomapServer(const std::string& filename)
: m_nh(),
  m_pointCloudSub(NULL),
  m_tfPointCloudSub(NULL),
  m_octoMap(NULL),
  m_maxRange(-1.0),
  m_worldFrameId("/map"), m_baseFrameId("base_footprint"),
  m_useHeightMap(true),
  m_colorFactor(0.8),
  m_latchedTopics(false),
  m_res(0.05),
  m_treeDepth(0),
  m_maxTreeDepth(0),
  m_probHit(0.7), m_probMiss(0.4),
  m_thresMin(0.12), m_thresMax(0.97),
  m_pointcloudMinZ(-std::numeric_limits<double>::max()),
  m_pointcloudMaxZ(std::numeric_limits<double>::max()),
  m_occupancyMinZ(-std::numeric_limits<double>::max()),
  m_occupancyMaxZ(std::numeric_limits<double>::max()),
  m_minSizeX(0.0), m_minSizeY(0.0),
  m_filterSpeckles(false), m_filterGroundPlane(false),
  m_groundFilterDistance(0.04), m_groundFilterAngle(0.15), m_groundFilterPlaneDistance(0.07)
{
  ros::NodeHandle private_nh("~");
  private_nh.param("frame_id", m_worldFrameId, m_worldFrameId);
  private_nh.param("base_frame_id", m_baseFrameId, m_baseFrameId);
  private_nh.param("height_map", m_useHeightMap, m_useHeightMap);
  private_nh.param("color_factor", m_colorFactor, m_colorFactor);

  private_nh.param("pointcloud_min_z", m_pointcloudMinZ,m_pointcloudMinZ);
  private_nh.param("pointcloud_max_z", m_pointcloudMaxZ,m_pointcloudMaxZ);
  private_nh.param("occupancy_min_z", m_occupancyMinZ,m_occupancyMinZ);
  private_nh.param("occupancy_max_z", m_occupancyMaxZ,m_occupancyMaxZ);
  private_nh.param("min_x_size", m_minSizeX,m_minSizeX);
  private_nh.param("min_y_size", m_minSizeY,m_minSizeY);

  private_nh.param("filter_speckles", m_filterSpeckles, m_filterSpeckles);
  private_nh.param("filter_ground", m_filterGroundPlane, m_filterGroundPlane);
  // distance of points from plane for RANSAC
  private_nh.param("ground_filter/distance", m_groundFilterDistance, m_groundFilterDistance);
  // angular derivation of found plane:
  private_nh.param("ground_filter/angle", m_groundFilterAngle, m_groundFilterAngle);
  // distance of found plane from z=0 to be detected as ground (e.g. to exclude tables)
  private_nh.param("ground_filter/plane_distance", m_groundFilterPlaneDistance, m_groundFilterPlaneDistance);

  private_nh.param("sensor_model/max_range", m_maxRange, m_maxRange);

  private_nh.param("resolution", m_res, m_res);
  private_nh.param("sensor_model/hit", m_probHit, m_probHit);
  private_nh.param("sensor_model/miss", m_probMiss, m_probMiss);
  private_nh.param("sensor_model/min", m_thresMin, m_thresMin);
  private_nh.param("sensor_model/max", m_thresMax, m_thresMax);


  // initialize octomap object & params
  m_octoMap = new OcTreeROS(m_res);
  m_octoMap->octree.setProbHit(m_probHit);
  m_octoMap->octree.setProbMiss(m_probMiss);
  m_octoMap->octree.setClampingThresMin(m_thresMin);
  m_octoMap->octree.setClampingThresMax(m_thresMax);
  m_treeDepth = m_octoMap->octree.getTreeDepth();
  m_maxTreeDepth = m_treeDepth;
  m_gridmap.info.resolution = m_res;


  double r, g, b, a;
  private_nh.param("color/r", r, 0.0);
  private_nh.param("color/g", g, 0.0);
  private_nh.param("color/b", b, 1.0);
  private_nh.param("color/a", a, 1.0);
  m_color.r = r;
  m_color.g = g;
  m_color.b = b;
  m_color.a = a;

  bool staticMap = false;
  if (filename != "")
    staticMap = true;

  m_latchedTopics = staticMap;
  private_nh.param("latch", m_latchedTopics, m_latchedTopics);

  m_markerPub = m_nh.advertise<visualization_msgs::MarkerArray>("occupied_cells_vis_array", 1, m_latchedTopics);
  m_binaryMapPub = m_nh.advertise<octomap_msgs::OctomapBinary>("octomap_binary", 1, m_latchedTopics);
  m_pointCloudPub = m_nh.advertise<sensor_msgs::PointCloud2>("octomap_point_cloud_centers", 1, m_latchedTopics);
  m_collisionObjectPub = m_nh.advertise<arm_navigation_msgs::CollisionObject>("octomap_collision_object", 1, m_latchedTopics);
  m_mapPub = m_nh.advertise<nav_msgs::OccupancyGrid>("map", 5, m_latchedTopics);
  m_cmapPub = m_nh.advertise<arm_navigation_msgs::CollisionMap>("collision_map_out", 1, m_latchedTopics);


  // a filename to load is set => distribute a static map latched
  if (staticMap){
    if (m_octoMap->octree.readBinary(filename)){
      ROS_INFO("Octomap file %s loaded (%zu nodes).", filename.c_str(),m_octoMap->octree.size());
      m_treeDepth = m_octoMap->octree.getTreeDepth();
      m_maxTreeDepth = m_treeDepth;
      m_res = m_octoMap->octree.getResolution();
      m_gridmap.info.resolution = m_res;

      publishAll();
    } else{
      ROS_ERROR("Could not open requested file %s, exiting.", filename.c_str());
      exit(-1);
    }
  } else { // otherwise: do scan integration



    m_pointCloudSub = new message_filters::Subscriber<sensor_msgs::PointCloud2> (m_nh, "cloud_in", 5);
    m_tfPointCloudSub = new tf::MessageFilter<sensor_msgs::PointCloud2> (*m_pointCloudSub, m_tfListener, m_worldFrameId, 5);
    m_tfPointCloudSub->registerCallback(boost::bind(&OctomapServer::insertCloudCallback, this, _1));
  }

  m_octomapService = m_nh.advertiseService("octomap_binary", &OctomapServer::serviceCallback, this);
  m_clearBBXService = private_nh.advertiseService("clear_bbx", &OctomapServer::clearBBXSrv, this);
  m_resetService = private_nh.advertiseService("reset", &OctomapServer::resetSrv, this);

  dynamic_reconfigure::Server<OctomapServerConfig>::CallbackType f;

  f = boost::bind(&OctomapServer::reconfigureCallback, this, _1, _2);
  m_reconfigureServer.setCallback(f);
}

OctomapServer::~OctomapServer(){
  if (m_pointCloudSub)
    delete m_pointCloudSub;

  if (m_tfPointCloudSub)
    delete m_tfPointCloudSub;

  if (m_octoMap)
    delete m_octoMap;

}

void OctomapServer::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud){
  ros::WallTime startTime = ros::WallTime::now();


  //
  // ground filtering in base frame
  //
  PCLPointCloud pc; // input cloud for filtering and ground-detection
  pcl::fromROSMsg(*cloud, pc);

  tf::StampedTransform sensorToWorldTf;
  try {
    m_tfListener.lookupTransform(m_worldFrameId, cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);
  } catch(tf::TransformException& ex){
    ROS_ERROR_STREAM( "Transform error of sensor data: " << ex.what() << ", quitting callback");
    return;
  }

  Eigen::Matrix4f sensorToWorld;
  pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);


  // set up filter for height range, also removes NANs:
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setFilterFieldName("z");
  pass.setFilterLimits(m_pointcloudMinZ, m_pointcloudMaxZ);

  PCLPointCloud pc_ground; // segmented ground plane
  PCLPointCloud pc_nonground; // everything else

  if (m_filterGroundPlane){
    tf::StampedTransform sensorToBaseTf, baseToWorldTf;
    try{
      m_tfListener.waitForTransform(m_baseFrameId, cloud->header.frame_id, cloud->header.stamp, ros::Duration(0.2));
      m_tfListener.lookupTransform(m_baseFrameId, cloud->header.frame_id, cloud->header.stamp, sensorToBaseTf);
      m_tfListener.lookupTransform(m_worldFrameId, m_baseFrameId, cloud->header.stamp, baseToWorldTf);


    }catch(tf::TransformException& ex){
      ROS_ERROR_STREAM( "Transform error for ground plane filter: " << ex.what() << ", quitting callback.\n"
                        "You need to set the base_frame_id or disable filter_ground.");
    }


    Eigen::Matrix4f sensorToBase, baseToWorld;
    pcl_ros::transformAsMatrix(sensorToBaseTf, sensorToBase);
    pcl_ros::transformAsMatrix(baseToWorldTf, baseToWorld);

    // transform pointcloud from sensor frame to fixed robot frame
    pcl::transformPointCloud(pc, pc, sensorToBase);
    pass.setInputCloud(pc.makeShared());
    pass.filter(pc);
    filterGroundPlane(pc, pc_ground, pc_nonground);

    // transform clouds to world frame for insertion
    pcl::transformPointCloud(pc_ground, pc_ground, baseToWorld);
    pcl::transformPointCloud(pc_nonground, pc_nonground, baseToWorld);
  } else {
    // directly transform to map frame:
    pcl::transformPointCloud(pc, pc, sensorToWorld);

    // just filter height range:
    pass.setInputCloud(pc.makeShared());
    pass.filter(pc);

    pc_nonground = pc;
    // pc_nonground is empty without ground segmentation
    pc_ground.header = pc.header;
    pc_nonground.header = pc.header;
  }


  insertScan(sensorToWorldTf.getOrigin(), pc_ground, pc_nonground);

  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_DEBUG("Pointcloud insertion in OctomapServer done (%zu+%zu pts (ground/nonground), %f sec)", pc_ground.size(), pc_nonground.size(), total_elapsed);

  publishAll(cloud->header.stamp);
}

void OctomapServer::insertScan(const tf::Point& sensorOriginTf, const PCLPointCloud& ground, const PCLPointCloud& nonground){
  point3d sensorOrigin = pointTfToOctomap(sensorOriginTf);

  // instead of direct scan insertion, compute update to filter ground:
  KeySet free_cells, occupied_cells;
  // insert ground points only as free:
  for (PCLPointCloud::const_iterator it = ground.begin(); it != ground.end(); ++it){
    point3d point(it->x, it->y, it->z);
    // maxrange check
    if ((m_maxRange > 0.0) && ((point - sensorOrigin).norm() > m_maxRange) ) {
      point = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
    }

    // only clear space (ground points)
    if (m_octoMap->octree.computeRayKeys(sensorOrigin, point, m_keyRay)){
      free_cells.insert(m_keyRay.begin(), m_keyRay.end());
    }
  }

  // all other points: free on ray, occupied on endpoint:
  for (PCLPointCloud::const_iterator it = nonground.begin(); it != nonground.end(); ++it){
    point3d point(it->x, it->y, it->z);
    // maxrange check
    if ((m_maxRange < 0.0) || ((point - sensorOrigin).norm() <= m_maxRange) ) {

      // free cells
      if (m_octoMap->octree.computeRayKeys(sensorOrigin, point, m_keyRay)){
        free_cells.insert(m_keyRay.begin(), m_keyRay.end());
      }
      // occupied endpoint
      OcTreeKey key;
      if (m_octoMap->octree.genKey(point, key)){
        occupied_cells.insert(key);
      }
    } else {// ray longer than maxrange:;
      point3d new_end = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
      if (m_octoMap->octree.computeRayKeys(sensorOrigin, new_end, m_keyRay)){
        free_cells.insert(m_keyRay.begin(), m_keyRay.end());
      }
    }
  }

  // mark free cells only if not seen occupied in this cloud
  for(KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it){
    if (occupied_cells.find(*it) == occupied_cells.end()){
      m_octoMap->octree.updateNode(*it, false);
    }
  }

  // now mark all occupied cells:
  for (KeySet::iterator it = occupied_cells.begin(), end=free_cells.end(); it!= end; it++) {
    m_octoMap->octree.updateNode(*it, true);
  }

  // TODO: eval lazy+updateInner vs. proper insertion
  // non-lazy by default (updateInnerOccupancy() too slow for large maps)
  //m_octoMap->octree.updateInnerOccupancy();

  // prune by default:
  m_octoMap->octree.prune();


}


void OctomapServer::publishAll(const ros::Time& rostime){
  ros::WallTime startTime = ros::WallTime::now();
  size_t octomapSize = m_octoMap->octree.size();
  // TODO: estimate num occ. voxels for size of arrays (reserve)
  if (octomapSize <= 1){
    ROS_WARN("Nothing to publish, octree is empty");
    return;
  }

  bool publishCollisionMap = (m_latchedTopics || m_cmapPub.getNumSubscribers() > 0);
  bool publishCollisionObject = (m_latchedTopics || m_collisionObjectPub.getNumSubscribers() > 0);
  bool publishMarkerArray = (m_latchedTopics || m_markerPub.getNumSubscribers() > 0);
  bool publishPointCloud = (m_latchedTopics || m_pointCloudPub.getNumSubscribers() > 0);
  bool publishOctoMap = (m_latchedTopics || m_binaryMapPub.getNumSubscribers() > 0);

  // init collision object:
  arm_navigation_msgs::CollisionObject collisionObject;
  collisionObject.header.frame_id = m_worldFrameId;
  collisionObject.header.stamp = rostime;
  collisionObject.id = "map";
  arm_navigation_msgs::OrientedBoundingBox collObjBox;
  collObjBox.axis.x = collObjBox.axis.y = 0.0;
  collObjBox.axis.z = 1.0;
  collObjBox.angle = 0.0;

  //init collision map:
  arm_navigation_msgs::CollisionMap collisionMap;
  collisionMap.header.frame_id = m_worldFrameId;
  collisionMap.header.stamp = rostime;
  arm_navigation_msgs::Shape collObjShape;
  collObjShape.type = arm_navigation_msgs::Shape::BOX;
  collObjShape.dimensions.resize(3);

  geometry_msgs::Pose pose;
  pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

  // init markers:
  visualization_msgs::MarkerArray occupiedNodesVis;
  // each array stores all cubes of a different size, one for each depth level:
  occupiedNodesVis.markers.resize(m_treeDepth+1);

  // init pointcloud:
  pcl::PointCloud<pcl::PointXYZ> pclCloud;

  // call pre-traversal hook:
  handlePreNodeTraversal(rostime);

  // now, traverse all leafs in the tree:
  for (OcTreeROS::OcTreeType::iterator it = m_octoMap->octree.begin(m_maxTreeDepth),
      end = m_octoMap->octree.end(); it != end; ++it)
  {

    // call general hook:
    handleNode(it);

    if (m_octoMap->octree.isNodeOccupied(*it)){
      double z = it.getZ();
      if (z > m_occupancyMinZ && z < m_occupancyMaxZ)
      {
        double size = it.getSize();
        double x = it.getX();
        double y = it.getY();
        octomap::OcTreeKey nKey = it.getKey();
        // Ignore speckles in the map:
        if (m_filterSpeckles && (it.getDepth() == m_treeDepth +1) && isSpeckleNode(nKey)){
          ROS_DEBUG("Ignoring single speckle at (%f,%f,%f)", x, y, z);
          continue;
        } // else: current octree node is no speckle, send it out

        handleOccupiedNode(it);


        // create collision object:
        if (publishCollisionObject){
          collObjShape.dimensions[0] = collObjShape.dimensions[1] = collObjShape.dimensions[2] = size;
          collisionObject.shapes.push_back(collObjShape);
          pose.position.x = x;
          pose.position.y = y;
          pose.position.z = z;
          collisionObject.poses.push_back(pose);
        }

        if (publishCollisionMap){
          collObjBox.extents.x = collObjBox.extents.y = collObjBox.extents.z = size;

          collObjBox.center.x = x;
          collObjBox.center.y = y;
          collObjBox.center.z = z;

          collisionMap.boxes.push_back(collObjBox);

        }

        //create marker:
        if (publishMarkerArray){
          unsigned idx = it.getDepth();
          assert(idx < occupiedNodesVis.markers.size());

          geometry_msgs::Point cubeCenter;
          cubeCenter.x = x;
          cubeCenter.y = y;
          cubeCenter.z = z;

          occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
          if (m_useHeightMap){
            double minX, minY, minZ, maxX, maxY, maxZ;
            m_octoMap->octree.getMetricMin(minX, minY, minZ);
            m_octoMap->octree.getMetricMax(maxX, maxY, maxZ);

            double h = (1.0 - std::min(std::max((cubeCenter.z-minZ)/ (maxZ - minZ), 0.0), 1.0)) *m_colorFactor;
            occupiedNodesVis.markers[idx].colors.push_back(heightMapColor(h));
          }
        }

        // insert into pointcloud:
        if (publishPointCloud)
          pclCloud.push_back(pcl::PointXYZ(x, y, z));

      }
    } else{ // node not occupied => mark as free in 2D map if unknown so far
      handleFreeNode(it);
    }
  }

  // call post-traversal hook:
  handlePostNodeTraversal(rostime);

  // finish MarkerArray:
  if (publishMarkerArray){
    for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i){
      double size = m_octoMap->octree.getNodeSize(i);

      occupiedNodesVis.markers[i].header.frame_id = m_worldFrameId;
      occupiedNodesVis.markers[i].header.stamp = rostime;
      occupiedNodesVis.markers[i].ns = "map";
      occupiedNodesVis.markers[i].id = i;
      occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
      occupiedNodesVis.markers[i].scale.x = size;
      occupiedNodesVis.markers[i].scale.y = size;
      occupiedNodesVis.markers[i].scale.z = size;
      occupiedNodesVis.markers[i].color = m_color;


      if (occupiedNodesVis.markers[i].points.size() > 0)
        occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
      else
        occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
    }


    m_markerPub.publish(occupiedNodesVis);
  }

  // finish pointcloud:
  if (publishPointCloud){
    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg (pclCloud, cloud);
    cloud.header.frame_id = m_worldFrameId;
    cloud.header.stamp = rostime;
    m_pointCloudPub.publish(cloud);
  }

  if (publishCollisionObject)
    m_collisionObjectPub.publish(collisionObject);

  if (publishCollisionMap)
    m_cmapPub.publish(collisionMap);

  if (publishOctoMap)
    publishMap(rostime);

  double total_elapsed = (ros::WallTime::now() - startTime).toSec();
  ROS_DEBUG("Map publishing in OctomapServer took %f sec", total_elapsed);

}


bool OctomapServer::serviceCallback(octomap_msgs::GetOctomap::Request  &req,
		octomap_msgs::GetOctomap::Response &res)
{
  ROS_INFO("Sending map data on service request");
  res.map.header.frame_id = m_worldFrameId;
  res.map.header.stamp = ros::Time::now();
  octomap::octomapMapToMsgData(m_octoMap->octree, res.map.data);

  return true;
}

bool OctomapServer::clearBBXSrv(octomap_msgs::ClearBBXRegionRequest& req, octomap_msgs::ClearBBXRegionRequest& resp){
  OcTreeROS::OcTreeType::leaf_bbx_iterator it, end;
  point3d min = pointMsgToOctomap(req.min);
  point3d max = pointMsgToOctomap(req.max);

  for(OcTreeROS::OcTreeType::leaf_bbx_iterator it = m_octoMap->octree.begin_leafs_bbx(min,max),
      end=m_octoMap->octree.end_leafs_bbx(); it!= end; ++it){

    it->setLogOdds(m_thresMin);
    //			m_octoMap->octree.updateNode(it.getKey(), -6.0f);
  }
  // TODO: eval which is faster (setLogOdds+updateInner or updateNode)
  m_octoMap->octree.updateInnerOccupancy();

  publishAll(ros::Time::now());

  return true;
}

bool OctomapServer::resetSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp) {
  visualization_msgs::MarkerArray occupiedNodesVis;
  occupiedNodesVis.markers.resize(m_treeDepth +1);
  ros::Time rostime = ros::Time::now();
  m_octoMap->octree.clear();

  ROS_INFO("Cleared octomap");
  publishMap(rostime);
  for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i){

    occupiedNodesVis.markers[i].header.frame_id = m_worldFrameId;
    occupiedNodesVis.markers[i].header.stamp = rostime;
    occupiedNodesVis.markers[i].ns = "map";
    occupiedNodesVis.markers[i].id = i;
    occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
  }


  m_markerPub.publish(occupiedNodesVis);
  return true;
}

void OctomapServer::publishMap(const ros::Time& rostime) const{

  octomap_msgs::OctomapBinary map;
  map.header.frame_id = m_worldFrameId;
  map.header.stamp = rostime;

  octomap::octomapMapToMsgData(m_octoMap->octree, map.data);

  m_binaryMapPub.publish(map);
}


void OctomapServer::filterGroundPlane(const PCLPointCloud& pc, PCLPointCloud& ground, PCLPointCloud& nonground) const{
  ground.header = pc.header;
  nonground.header = pc.header;

  if (pc.size() < 50){
    ROS_WARN("Pointcloud in OctomapServer too small, skipping ground plane extraction");
    nonground = pc;
  } else {
    // plane detection for ground plane removal:
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // Create the segmentation object and set up:
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients (true);
    // TODO: maybe a filtering based on the surface normals might be more robust / accurate?
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(200);
    seg.setDistanceThreshold (m_groundFilterDistance);
    seg.setAxis(Eigen::Vector3f(0,0,1));
    seg.setEpsAngle(m_groundFilterAngle);


    PCLPointCloud cloud_filtered(pc);
    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    bool groundPlaneFound = false;

    while(cloud_filtered.size() > 10 && !groundPlaneFound){
      seg.setInputCloud(cloud_filtered.makeShared());
      seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0){
        ROS_WARN("No plane found in cloud.");

        break;
      }

      extract.setInputCloud(cloud_filtered.makeShared());
      extract.setIndices(inliers);

      if (std::abs(coefficients->values.at(3)) < m_groundFilterPlaneDistance){
        ROS_DEBUG("Ground plane found: %zu/%zu inliers. Coeff: %f %f %f %f", inliers->indices.size(), cloud_filtered.size(),
                  coefficients->values.at(0), coefficients->values.at(1), coefficients->values.at(2), coefficients->values.at(3));
        extract.setNegative (false);
        extract.filter (ground);

        // remove ground points from full pointcloud:
        // workaround for PCL bug:
        if(inliers->indices.size() != cloud_filtered.size()){
          extract.setNegative(true);
          PCLPointCloud cloud_out;
          extract.filter(cloud_out);
          nonground += cloud_out;
          cloud_filtered = cloud_out;
        }

        groundPlaneFound = true;
      } else{
        ROS_DEBUG("Horizontal plane (not ground) found: %zu/%zu inliers. Coeff: %f %f %f %f", inliers->indices.size(), cloud_filtered.size(),
                  coefficients->values.at(0), coefficients->values.at(1), coefficients->values.at(2), coefficients->values.at(3));
        pcl::PointCloud<pcl::PointXYZ> cloud_out;
        extract.setNegative (false);
        extract.filter(cloud_out);
        nonground +=cloud_out;
        // debug
        //            pcl::PCDWriter writer;
        //            writer.write<pcl::PointXYZ>("nonground_plane.pcd",cloud_out, false);

        // remove current plane from scan for next iteration:
        // workaround for PCL bug:
        if(inliers->indices.size() != cloud_filtered.size()){
          extract.setNegative(true);
          cloud_out.points.clear();
          extract.filter(cloud_out);
          cloud_filtered = cloud_out;
        } else{
          cloud_filtered.points.clear();
        }
      }

    }
    // TODO: also do this if overall starting pointcloud too small?
    if (!groundPlaneFound){ // no plane found or remaining points too small
      ROS_WARN("No ground plane found in scan");

      // do a rough fitlering on height to prevent spurious obstacles
      pcl::PassThrough<pcl::PointXYZ> second_pass;
      second_pass.setFilterFieldName("z");
      second_pass.setFilterLimits(-m_groundFilterPlaneDistance, m_groundFilterPlaneDistance);
      second_pass.setInputCloud(pc.makeShared());
      second_pass.filter(ground);

      second_pass.setFilterLimitsNegative (true);
      second_pass.filter(nonground);
    }

    // debug:
    //        pcl::PCDWriter writer;
    //        if (pc_ground.size() > 0)
    //          writer.write<pcl::PointXYZ>("ground.pcd",pc_ground, false);
    //        if (pc_nonground.size() > 0)
    //          writer.write<pcl::PointXYZ>("nonground.pcd",pc_nonground, false);

  }


}

void OctomapServer::handlePreNodeTraversal(const ros::Time& rostime){
  m_publish2DMap = (m_latchedTopics || m_mapPub.getNumSubscribers() > 0);

  // FIXME: fix crash and remove
  if (m_maxTreeDepth != m_treeDepth)
    m_publish2DMap = false;

  if (m_publish2DMap){
    // init projected 2D map:
    m_gridmap.header.frame_id = m_worldFrameId;
    m_gridmap.header.stamp = rostime;

    // TODO: move most of this stuff into c'tor and init map only once (adjust if size changes)
    double minX, minY, minZ, maxX, maxY, maxZ;
    m_octoMap->octree.getMetricMin(minX, minY, minZ);
    m_octoMap->octree.getMetricMax(maxX, maxY, maxZ);

    octomap::point3d minPt(minX, minY, minZ);
    octomap::point3d maxPt(maxX, maxY, maxZ);
    octomap::OcTreeKey minKey, maxKey, curKey;
    if (!m_octoMap->octree.genKey(minPt, minKey)){
      ROS_ERROR("Could not create min OcTree key at %f %f %f", minPt.x(), minPt.y(), minPt.z());
      return;
    }

    if (!m_octoMap->octree.genKey(maxPt, maxKey)){
      ROS_ERROR("Could not create max OcTree key at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());
      return;
    }
    m_octoMap->octree.genKeyAtDepth(minKey, m_maxTreeDepth, minKey);
    m_octoMap->octree.genKeyAtDepth(maxKey, m_maxTreeDepth, maxKey);

    ROS_DEBUG("MinKey: %d %d %d / MaxKey: %d %d %d", minKey[0], minKey[1], minKey[2], maxKey[0], maxKey[1], maxKey[2]);

    // add padding if requested (= new min/maxPts in x&y):
    double halfPaddedX = 0.5*m_minSizeX;
    double halfPaddedY = 0.5*m_minSizeY;
    minX = std::min(minX, -halfPaddedX);
    maxX = std::max(maxX, halfPaddedX);
    minY = std::min(minY, -halfPaddedY);
    maxY = std::max(maxY, halfPaddedY);
    minPt = octomap::point3d(minX, minY, minZ);
    maxPt = octomap::point3d(maxX, maxY, maxZ);

    OcTreeKey paddedMaxKey;
    if (!m_octoMap->octree.genKey(minPt, m_paddedMinKey)){
      ROS_ERROR("Could not create padded min OcTree key at %f %f %f", minPt.x(), minPt.y(), minPt.z());
      return;
    }
    if (!m_octoMap->octree.genKey(maxPt, paddedMaxKey)){
      ROS_ERROR("Could not create padded max OcTree key at %f %f %f", maxPt.x(), maxPt.y(), maxPt.z());
      return;
    }
    m_octoMap->octree.genKeyAtDepth(m_paddedMinKey, m_maxTreeDepth, m_paddedMinKey);
    m_octoMap->octree.genKeyAtDepth(paddedMaxKey, m_maxTreeDepth, paddedMaxKey);

    ROS_DEBUG("Padded MinKey: %d %d %d / padded MaxKey: %d %d %d", m_paddedMinKey[0], m_paddedMinKey[1], m_paddedMinKey[2], paddedMaxKey[0], paddedMaxKey[1], paddedMaxKey[2]);
    assert(paddedMaxKey[0] >= maxKey[0] && paddedMaxKey[1] >= maxKey[1]);

    m_gridmap.info.width = paddedMaxKey[0] - m_paddedMinKey[0] +1;
    m_gridmap.info.height = paddedMaxKey[1] - m_paddedMinKey[1] +1;
    int mapOriginX = minKey[0] - m_paddedMinKey[0];
    int mapOriginY = minKey[1] - m_paddedMinKey[1];
    assert(mapOriginX >= 0 && mapOriginY >= 0);

    // might not exactly be min / max of octree:
    octomap::point3d origin;
    m_octoMap->octree.genCoords(m_paddedMinKey, m_treeDepth, origin);
    double gridRes = m_octoMap->octree.getNodeSize(m_maxTreeDepth);
    m_gridmap.info.resolution = gridRes;
    m_gridmap.info.origin.position.x = origin.x() - gridRes*0.5;
    m_gridmap.info.origin.position.y = origin.y() - gridRes*0.5;

    // Allocate space to hold the data (init to unknown)
    m_gridmap.data.resize(m_gridmap.info.width * m_gridmap.info.height, -1);
  }

}

void OctomapServer::handlePostNodeTraversal(const ros::Time& rostime){

  if (m_publish2DMap)
    m_mapPub.publish(m_gridmap);

}

void OctomapServer::handleOccupiedNode(const OcTreeROS::OcTreeType::iterator& it){
  // update 2D map (occupied always overrides):
  if (m_publish2DMap){
    if (it.getDepth() == m_maxTreeDepth){
      octomap::OcTreeKey nKey = it.getKey(); // TODO: remove intermedate obj (1.4)
      int i = nKey[0] - m_paddedMinKey[0];
      int j = nKey[1] - m_paddedMinKey[1];
      m_gridmap.data[m_gridmap.info.width*j + i] = 100;
    } else{
      int intSize = 1 << (m_treeDepth - it.getDepth());
      octomap::OcTreeKey minKey=it.getIndexKey();
      for(int dx=0; dx < intSize; dx++){
        int i = minKey[0]+dx - m_paddedMinKey[0];
        for(int dy=0; dy < intSize; dy++){
          int j = minKey[1]+dy - m_paddedMinKey[1];
          m_gridmap.data[m_gridmap.info.width*j + i] = 100;
        }
      }
    }
  }

}

void OctomapServer::handleFreeNode(const OcTreeROS::OcTreeType::iterator& it){
  if (m_publish2DMap){
    if (it.getDepth() == m_maxTreeDepth){
      octomap::OcTreeKey nKey = it.getKey(); //TODO: remove intermedate obj (1.4)
      int i = nKey[0] - m_paddedMinKey[0];
      int j = nKey[1] - m_paddedMinKey[1];
      if (m_gridmap.data[m_gridmap.info.width*j + i] == -1){
        m_gridmap.data[m_gridmap.info.width*j + i] = 0;
      }
    } else{
      int intSize = 1 << (m_treeDepth - it.getDepth());
      octomap::OcTreeKey minKey=it.getIndexKey();
      for(int dx=0; dx < intSize; dx++){
        int i = minKey[0]+dx - m_paddedMinKey[0];
        for(int dy=0; dy < intSize; dy++){
          int j = minKey[1]+dy - m_paddedMinKey[1];
          if (m_gridmap.data[m_gridmap.info.width*j + i] == -1){
            m_gridmap.data[m_gridmap.info.width*j + i] = 0;
          }
        }
      }
    }
  }
}



bool OctomapServer::isSpeckleNode(const OcTreeKey&nKey) const {
  OcTreeKey key;
  bool neighborFound = false;
  for (key[2] = nKey[2] - 1; !neighborFound && key[2] <= nKey[2] + 1; ++key[2]){
    for (key[1] = nKey[1] - 1; !neighborFound && key[1] <= nKey[1] + 1; ++key[1]){
      for (key[0] = nKey[0] - 1; !neighborFound && key[0] <= nKey[0] + 1; ++key[0]){
        if (key != nKey){
          OcTreeNode* node = m_octoMap->octree.search(key);
          if (node && m_octoMap->octree.isNodeOccupied(node)){
            // we have a neighbor => break!
            neighborFound = true;
          }
        }
      }
    }
  }

  return neighborFound;
}

void OctomapServer::reconfigureCallback(octomap_server::OctomapServerConfig& config, uint32_t level){
  m_maxTreeDepth = config.max_depth;
  if (m_latchedTopics)
    publishAll();


}

std_msgs::ColorRGBA OctomapServer::heightMapColor(double h) const {

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
}



