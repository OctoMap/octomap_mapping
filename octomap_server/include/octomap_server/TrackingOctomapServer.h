#ifndef TRACKINGOCTOMAPSERVER_H_
#define TRACKINGOCTOMAPSERVER_H_

#include "OctomapServer.h"

namespace octomap {

class TrackingOctomapServer: public octomap::OctomapServer {
public:
	TrackingOctomapServer(const std::string& filename = "");
	virtual ~TrackingOctomapServer();

	void trackCallback(sensor_msgs::PointCloud2Ptr cloud);
	void insertScan(const tf::Point& sensorOrigin, const PCLPointCloud& ground, const PCLPointCloud& nonground);

protected:
	void trackChanges();

	bool listen_changes;
	bool track_changes;
	ros::Publisher pubFreeChangeSet;
	ros::Publisher pubChangeSet;
	ros::Subscriber subChangeSet;
	ros::Subscriber subFreeChanges;
};

} /* namespace octomap */
#endif /* TRACKINGOCTOMAPSERVER_H_ */
