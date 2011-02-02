#ifndef OCTOMAP_CONVERT_MSGS_H
#define OCTOMAP_CONVERT_MSGS_H

#include <octomap/octomap.h>
#include <octomap_ros/OctomapBinary.h>
#include <octomap_ros/OctomapBinaryWithPose.h>

namespace octomap {
  /**
   * Converts an octomap map structure to a ROS octomap msg as binary data
   *
   * @param octomap input OcTree
   * @param mapMsg output msg
   */
  static inline void octomapMapToMsg(const OcTree& octomap, octomap_ros::OctomapBinary& mapMsg){
    // conversion via stringstream

    // TODO: read directly into buffer? see
    // http://stackoverflow.com/questions/132358/how-to-read-file-content-into-istringstream
    std::stringstream datastream;
    octomap.writeBinaryConst(datastream);
    std::string datastring = datastream.str();
    mapMsg.header.stamp = ros::Time::now();
    mapMsg.data = std::vector<int8_t>(datastring.begin(), datastring.end());
  }

  /**
   * Converts a ROS octomap msg (binary data) to an octomap map structure
   *
   * @param mapMsg
   * @param octomap
   */
  static inline void octomapMsgToMap(const octomap_ros::OctomapBinary& mapMsg, octomap::OcTree& octomap){
    std::stringstream datastream;
    assert(mapMsg.data.size() > 0);
    datastream.write((const char*) &mapMsg.data[0], mapMsg.data.size());
    octomap.readBinary(datastream);
  }

}


#endif

