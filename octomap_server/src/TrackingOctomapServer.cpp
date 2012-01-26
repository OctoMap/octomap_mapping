#include <octomap_server/TrackingOctomapServer.h>
#include <string>

namespace octomap {

TrackingOctomapServer::TrackingOctomapServer(const std::string& filename) :
	OctomapServer()
{
	//read tree if necessary
	if (filename != "") {
		if (m_octoMap->octree.readBinary(filename)) {
			ROS_INFO("Octomap file %s loaded (%zu nodes).", filename.c_str(), m_octoMap->octree.size());
			m_treeDepth = m_octoMap->octree.getTreeDepth();
			m_res = m_octoMap->octree.getResolution();
			m_gridmap.info.resolution = m_res;

			publishAll();
		} else {
			ROS_ERROR("Could not open requested file %s, exiting.", filename.c_str());
			exit(-1);
		}
	}

	ros::NodeHandle private_nh("~");

	std::string changeSetTopic = "changes";

	private_nh.param("topic_changes", changeSetTopic, changeSetTopic);
	private_nh.param("track_changes", track_changes, false);
	private_nh.param("listen_changes", listen_changes, false);

	if (track_changes && listen_changes) {
		ROS_WARN("OctoMapServer: It might not be useful to publish changes and at the same time listen to them."
				"Setting 'track_changes' to false!");
		track_changes = false;
	}

	if (track_changes) {
		ROS_INFO("starting server");
		pubChangeSet = private_nh.advertise<sensor_msgs::PointCloud2>(
				changeSetTopic, 1);
		m_octoMap->octree.enableChangeDetection(true);
	}

	if (listen_changes) {
		ROS_INFO("starting client");
		subChangeSet = private_nh.subscribe(changeSetTopic, 1,
				&TrackingOctomapServer::trackCallback, this);
	}
}

TrackingOctomapServer::~TrackingOctomapServer() {
}

void TrackingOctomapServer::insertScan(const tf::Point & sensorOrigin, const PCLPointCloud & ground, const PCLPointCloud & nonground) {
	OctomapServer::insertScan(sensorOrigin, ground, nonground);

	if (track_changes) {
		trackChanges();
	}
}

void TrackingOctomapServer::trackChanges() {
	KeyBoolMap::const_iterator startPnt = m_octoMap->octree.changedKeysBegin();
	KeyBoolMap::const_iterator endPnt = m_octoMap->octree.changedKeysEnd();

	pcl::PointCloud<pcl::PointXYZI> changedCells = pcl::PointCloud<pcl::PointXYZI>();

	int c = 0;
	for (KeyBoolMap::const_iterator iter = startPnt; iter != endPnt; ++iter) {
		c++;
		OcTreeNode* node = m_octoMap->octree.search(iter->first);

		bool occupied = m_octoMap->octree.isNodeOccupied(node);

		pcl::PointXYZI pnt;
		pnt.x = iter->first.k[0];
		pnt.y = iter->first.k[1];
		pnt.z = iter->first.k[2];

		if (occupied) {
			pnt.intensity = 1000;
		}
		else {
			pnt.intensity = -1000;
		}

		changedCells.push_back(pnt);
	}

	sensor_msgs::PointCloud2 changed;
	pcl::toROSMsg(changedCells, changed);
	changed.header.frame_id = "/talker/changes";
	changed.header.stamp = ros::Time().now();
	pubChangeSet.publish(changed);
	ROS_DEBUG("[server] sending %d changed entries", (int)changedCells.size());

	m_octoMap->octree.resetChangeDetection();
	ROS_DEBUG("[server] octomap size after updating: %d", (int)m_octoMap->octree.calcNumNodes());
}

void TrackingOctomapServer::trackCallback(sensor_msgs::PointCloud2Ptr cloud) {
	pcl::PointCloud<pcl::PointXYZI> cells;
	pcl::fromROSMsg(*cloud, cells);
	ROS_DEBUG("[client] size of newly occupied cloud: %i", (int)cells.points.size());

	for (size_t i = 0; i < cells.points.size(); i++) {
		pcl::PointXYZI& pnt = cells.points[i];
		m_octoMap->octree.updateNode(OcTreeKey(pnt.x, pnt.y, pnt.z), pnt.intensity, false);
	}

	m_octoMap->octree.updateInnerOccupancy();
	ROS_DEBUG("[client] octomap size after updating: %d", (int)m_octoMap->octree.calcNumNodes());
}

} /* namespace octomap */
