/**
* octomap_server: A Tool to serve 3D OctoMaps in ROS (binary and as visualization)
* (inspired by the ROS map_saver)
* @author A. Hornung, University of Freiburg, Copyright (C) 2009.
* @see http://octomap.sourceforge.net/
* License: BSD
*/

/*
 * Copyright (c) 2010, A. Hornung, University of Freiburg
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
#include <std_msgs/ColorRGBA.h>
#include <mapping_msgs/CollisionObject.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <octomap_ros/OctomapBinary.h>
#include <octomap_ros/GetOctomap.h>
#include <octomap_ros/OctomapROS.h>


namespace octomap {
	class OctomapServer{
	public:
		OctomapServer(const std::string& filename= "");
		virtual ~OctomapServer();
		bool serviceCallback(octomap_ros::GetOctomap::Request  &req,
				octomap_ros::GetOctomap::Response &res);

		void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);

	private:
		void connectCallback(const ros::SingleSubscriberPublisher& pub);
		std_msgs::ColorRGBA heightMapColor(double h) const;
		void publishMap(const ros::Time& rostime = ros::Time::now());
		void publishMarkers(const ros::Time& rostime = ros::Time::now());
		void publishPointCloud(const ros::Time& rostime = ros::Time::now());
		void publishCollisionObject(const ros::Time& rostime = ros::Time::now());
		ros::NodeHandle m_nh;
		ros::Publisher m_markerPub, m_binaryMapPub, m_pointCloudPub, m_collisionObjectPub;
		message_filters::Subscriber<sensor_msgs::PointCloud2>* m_pointCloudSub;
		tf::MessageFilter<sensor_msgs::PointCloud2>* m_tfPointCloudSub;
		ros::ServiceServer m_service;
		tf::TransformListener m_tfListener;

		OcTreeROS m_octoMap;
		double m_maxRange;
		std::string m_frameId;
		bool m_useHeightMap;
		std_msgs::ColorRGBA m_color;
		double m_colorFactor;
		double m_visMinZ;
		double m_visMaxZ;
	};
}

