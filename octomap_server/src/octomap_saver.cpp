/**
* octomap_saver: Simple example which requests binary octomaps and stores them to a file.
*
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
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <fstream>

#if ROS_VERSION_MINIMUM(1,8,0)
  #include <octomap_msgs/GetOctomap.h>
  using octomap_msgs::GetOctomap;
#else
  #include <octomap_ros/GetOctomap.h>
  using octomap_ros::GetOctomap;
#endif

#define USAGE "\nUSAGE: octomap_saver <map.bt>\n" \
		"  map.bt: filename of map to be saved\n"

using namespace std;

/**
* @brief Map generation node.
*/
class MapSaver{
public:
  MapSaver(const std::string& mapname){
    ros::NodeHandle n;
    const static std::string servname = "octomap_binary";
    ROS_INFO("Requesting the map from %s...", n.resolveName(servname).c_str());
    GetOctomap::Request req;
    GetOctomap::Response resp;
    while(n.ok() && !ros::service::call(servname, req, resp))
    {
      ROS_WARN("Request to %s failed; trying again...", n.resolveName(servname).c_str());
      usleep(1000000);
    }

    if (n.ok()){ // skip when CTRL-C
      ROS_INFO("Map received, saving to %s", mapname.c_str());
      ofstream mapfile(mapname.c_str(), ios_base::binary);

      if (!mapfile.is_open()){
        ROS_ERROR("Could not open file %s for writing", mapname.c_str());
      } else {
        octomap::OcTree octomap(0.1);
        octomap::octomapMsgToMap(resp.map, octomap);
        octomap.writeBinary(mapname);

        ROS_INFO("Finished writing %zu nodes to file %s (res: %f)", octomap.size(), mapname.c_str(), octomap.getResolution());

        // write out stream directly (old format, no header)
        //mapfile.write((char*)&resp.map.data[0], resp.map.data.size());
        //mapfile.close();
      }
    }
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "octomap_saver");
  std::string mapFilename("");
  if (argc == 2)
    mapFilename = std::string(argv[1]);
  else{
    ROS_ERROR("%s", USAGE);
    exit(-1);
  }

  try{
    MapSaver ms(mapFilename);
  }catch(std::runtime_error& e){
    ROS_ERROR("map_saver exception: %s", e.what());
    return -1;
  }

  return 0;
}


