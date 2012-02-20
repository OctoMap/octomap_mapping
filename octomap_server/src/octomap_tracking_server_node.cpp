#include <ros/ros.h>
#include <octomap_server/TrackingOctomapServer.h>

#define USAGE "\nUSAGE: octomap_tracking_server <map.bt>\n" \
              "  map.bt: octomap 3D map file to read\n"

using namespace octomap;

int main(int argc, char** argv){
  ros::init(argc, argv, "octomap_tracking_server");
  std::string mapFilename("");

  if (argc > 2 || (argc == 2 && std::string(argv[1]) == "-h")){
	  ROS_ERROR("%s", USAGE);
	  exit(-1);
  }

  if (argc == 2)
	  mapFilename = std::string(argv[1]);

  try{
	  TrackingOctomapServer ms(mapFilename);
	  ros::spin();
  }catch(std::runtime_error& e){
	  ROS_ERROR("octomap_server exception: %s", e.what());
	  return -1;
  }

  return 0;
}
