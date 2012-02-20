/**
* octomap_server: A Tool to serve 3D OctoMaps in ROS (binary and as visualization)
* (inspired by the ROS map_saver)
* @author A. Hornung, University of Freiburg, Copyright (C) 2010-2011.
* @see http://octomap.sourceforge.net/
* License: BSD
*/

/*
 * Copyright (c) 2010-2011, A. Hornung, University of Freiburg
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


#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/ColorRGBA.h>
#include <arm_navigation_msgs/CollisionObject.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>

#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>


#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <octomap_ros/OctomapBinary.h>
#include <octomap_ros/GetOctomap.h>
#include <octomap_ros/ClearBBXRegion.h>
#include <octomap_ros/OctomapROS.h>
#include <octomap/OcTreeKey.h>


namespace octomap {
	class OctomapServer{

	public:
	  typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud;


		OctomapServer(const std::string& filename= "");
		virtual ~OctomapServer();
		bool serviceCallback(octomap_ros::GetOctomap::Request  &req, octomap_ros::GetOctomap::Response &res);
		bool clearBBXSrv(octomap_ros::ClearBBXRegionRequest& req, octomap_ros::ClearBBXRegionRequest& resp);
		bool resetSrv(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);

		void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);

	protected:
		void publishMap(const ros::Time& rostime = ros::Time::now()) const;
		void publishAll(const ros::Time& rostime = ros::Time::now());

		/// @brief Clear the internal OctoMap
		void resetOctomap();

		/**
		 * @brief update occupancy map with a scan labeled as ground and nonground.
		 * The scans should be in the global map frame.
		 *
		 * @param sensorOrigin origin of the measurements for raycasting
		 * @param ground scan endpoints on the ground plane (only clear space)
		 * @param nonground all other endpoints (clear up to occupied endpoint)
		 */
		virtual void insertScan(const tf::Point& sensorOrigin, const PCLPointCloud& ground, const PCLPointCloud& nonground);

		/// label the input cloud "pc" into ground and nonground. Should be in the robot's fixed frame (not world!)
		void filterGroundPlane(const PCLPointCloud& pc, PCLPointCloud& ground, PCLPointCloud& nonground) const;

		/**
		 * @brief Find speckle nodes (single occupied voxels with no neighbors). Only works on lowest resolution!
		 * @param key
		 * @return
		 */
		bool isSpeckleNode(const OcTreeKey& key) const;

		/// hook that is called after traversing all nodes
		void handlePreNodeTraversal(const ros::Time& rostime);

		/// hook that is called when traversing all nodes of the updated Octree (does nothing here)
		void handleNode(const OcTreeROS::OcTreeType::iterator& it) {};

		/// hook that is called when traversing occupied nodes of the updated Octree (updates 2D map projection here)
		void handleOccupiedNode(const OcTreeROS::OcTreeType::iterator& it);

		/// hook that is called when traversing free nodes of the updated Octree (updates 2D map projection here)
		void handleFreeNode(const OcTreeROS::OcTreeType::iterator& it);

		/// hook that is called after traversing all nodes
		void handlePostNodeTraversal(const ros::Time& rostime);

		std_msgs::ColorRGBA heightMapColor(double h) const;
		ros::NodeHandle m_nh;
		ros::Publisher m_markerPub, m_binaryMapPub, m_pointCloudPub, m_collisionObjectPub, m_mapPub;
		message_filters::Subscriber<sensor_msgs::PointCloud2>* m_pointCloudSub;
		tf::MessageFilter<sensor_msgs::PointCloud2>* m_tfPointCloudSub;
		ros::ServiceServer m_octomapService, m_clearBBXService, m_resetService;
		tf::TransformListener m_tfListener;

		OcTreeROS *m_octoMap;
		KeyRay m_keyRay;  // temp storage for ray casting
		double m_maxRange;
		std::string m_worldFrameId; // the map frame
		std::string m_baseFrameId; // base of the robot for ground plane filtering
		bool m_useHeightMap;
		std_msgs::ColorRGBA m_color;
		double m_colorFactor;

		bool m_latchedTopics;

		double m_res;
		unsigned m_treeDepth;
		double m_probHit;
		double m_probMiss;
		double m_thresMin;
		double m_thresMax;

		double m_pointcloudMinZ;
		double m_pointcloudMaxZ;
		double m_occupancyMinZ;
		double m_occupancyMaxZ;
		double m_minSizeX;
		double m_minSizeY;
		bool m_filterSpeckles;

		bool m_filterGroundPlane;
		double m_groundFilterDistance;
		double m_groundFilterAngle;
		double m_groundFilterPlaneDistance;

		// downprojected 2D map:
		nav_msgs::OccupancyGrid m_gridmap;
		bool m_publish2DMap;
		OcTreeKey m_paddedMinKey;
	};
}

