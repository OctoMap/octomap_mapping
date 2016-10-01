#include <boost/thread.hpp>

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>
#include <visualization_msgs/MarkerArray.h>

namespace octomap_server
{

namespace cmap_jet
{

float interpolate(float val, float y0, float x0, float y1, float x1) {
    return (val-x0) * (y1-y0) / (x1-x0) + y0;
}

float base(float val) {
    if (val <= -0.75) return 0;
    else if (val <= -0.25) return interpolate( val, 0.0, -0.75, 1.0, -0.25 );
    else if (val <= 0.25) return 1.0;
    else if (val <= 0.75) return interpolate( val, 1.0, 0.25, 0.0, 0.75 );
    else return 0.0;
}

float red( float gray ) {
    return base(gray - 0.5);
}
float green(float gray) {
    return base(gray);
}
float blue(float gray) {
    return base(gray + 0.5);
}

} // namespace cmap_jet

class OctoMapToMarkerArray : public nodelet::Nodelet
{
public:
  OctoMapToMarkerArray() {}
protected:
  virtual void onInit();
  void convert(const octomap_msgs::Octomap::ConstPtr& octomap_msg);

  ros::NodeHandle pnh_;

  ros::Publisher pub_marker_array_;
  ros::Subscriber sub_octomap_;
};

void OctoMapToMarkerArray::onInit()
{
  pnh_ = getPrivateNodeHandle();
  pub_marker_array_ = pnh_.advertise<visualization_msgs::MarkerArray>("output", /*queue_size=*/1);
  sub_octomap_ = pnh_.subscribe("input", 1, &OctoMapToMarkerArray::convert, this);
}

void OctoMapToMarkerArray::convert(const octomap_msgs::Octomap::ConstPtr& octomap_msg)
{
  visualization_msgs::MarkerArray marker_array_msg;

  // creating octree
  octomap::OcTree* octree = NULL;
  octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*octomap_msg);
  if (tree)
  {
    octree = dynamic_cast<octomap::OcTree*>(tree);
  }
  if (!octree)
  {
    ROS_ERROR("Failed to create octree structure");
    return;
  }

  // convert octomap -> marker_array
  unsigned tree_depth = octree->getTreeDepth();
  marker_array_msg.markers.resize(tree_depth + 1);
  for (octomap::OcTree::iterator it = octree->begin(tree_depth), end = octree->end();
       it != end; ++it)
  {
    if (octree->isNodeOccupied(*it))
    {
      geometry_msgs::Point cube_center;
      cube_center.x = it.getX();
      cube_center.y = it.getY();
      cube_center.z = it.getZ();

      float cell_probability = (*it).getOccupancy();
      std_msgs::ColorRGBA color;
      color.r = cmap_jet::red(cell_probability);
      color.g = cmap_jet::green(cell_probability);
      color.b = cmap_jet::blue(cell_probability);
      color.a = 1;

      unsigned idx = it.getDepth();
      marker_array_msg.markers[idx].points.push_back(cube_center);
      marker_array_msg.markers[idx].colors.push_back(color);
    }
  }

  // finalize marker_array to publish
  for (size_t i= 0; i < marker_array_msg.markers.size(); ++i)
  {
    double size = octree->getNodeSize(i);

    marker_array_msg.markers[i].header = octomap_msg->header;
    marker_array_msg.markers[i].scale.x = size;
    marker_array_msg.markers[i].scale.y = size;
    marker_array_msg.markers[i].scale.z = size;
    marker_array_msg.markers[i].id = i;
    marker_array_msg.markers[i].ns = "map";
    marker_array_msg.markers[i].type = visualization_msgs::Marker::CUBE_LIST;

    if (marker_array_msg.markers[i].points.size() > 0)
    {
      marker_array_msg.markers[i].action = visualization_msgs::Marker::ADD;
    }
    else
    {
      marker_array_msg.markers[i].action = visualization_msgs::Marker::DELETE;
    }
  }

  pub_marker_array_.publish(marker_array_msg);
}

} // namespace octomap_server

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(octomap_server::OctoMapToMarkerArray, nodelet::Nodelet);
