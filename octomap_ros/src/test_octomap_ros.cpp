
#include <octomap/octomap.h>
#include <octomap_ros/OctomapROS.h>
#include <octomap_ros/conversions.h>
#include <tf/transform_datatypes.h>

using namespace octomap;
using namespace std;

#define _1DEG 0.01745329251994329577

int main(int argc, char** argv) {


  //##############################################################     

  OcTreeROS tree (0.05);

  //  point3d origin (10.01, 10.01, 10.02);
  point3d origin (0.01, 0.01, 0.02);
  geometry_msgs::Point originPt = pointOctomapToMsg(origin);

  point3d point_on_surface (2.01,0.01,0.01);

  cout << "generating sphere at " << origin << " ..." << endl;

  for (int i=0; i<360; i++) {    
    for (int j=0; j<360; j++) {
      geometry_msgs::Point endPt = pointOctomapToMsg(origin+point_on_surface);
      if (!tree.insertRay(originPt, endPt)) {
        cout << "ERROR while inserting ray from " << origin << " to " << point_on_surface << endl;
      }
      point_on_surface.rotate_IP (0,0,_1DEG);
    }
    point_on_surface.rotate_IP (0,_1DEG,0);
  }


  cout << "done.\n\n";
  tf::Point pointTf(0.0, 0.0, 0.0);
  OcTreeROS::NodeType* treeNode = tree.search(pointTf);
  if (treeNode)
    cout << "Occupancy of node at (0, 0, 0) = " << treeNode->getOccupancy() << " (should be free)\n";
  else
    cerr << "ERROR: OcTreeNode not found (NULL)\n";

  treeNode = tree.search(tf::Point(10.01, 2.01, 0.01));
  if (treeNode)
    cout << "Occupancy of node at (0.01, 2.01, 0.01) = " << treeNode->getOccupancy() << " (should be occupied)\n";
  else
    cerr << "ERROR: OcTreeNode not found (NULL)\n";

  cout << "writing to sphere.bt..." << endl;
  tree.octree.writeBinary("sphere.bt");

  // -----------------------------------------------

  cout << "\ncasting rays ..." << endl;

  OcTreeROS sampled_surface(0.05);

  point3d direction = point3d (1.0,0.0,0.0);
  point3d obstacle(0,0,0);

  unsigned int hit (0);
  unsigned int miss (0);
  double mean_dist(0);

  for (int i=0; i<360; i++) {    
    for (int j=0; j<360; j++) {
      geometry_msgs::Point directionPt = pointOctomapToMsg(direction);
      geometry_msgs::Point obstaclePt;
      if (!tree.castRay(originPt, directionPt, obstaclePt, true, 3.)) {
        miss++;
      }
      else {
        hit++;
        point3d obstacle = pointMsgToOctomap(obstaclePt);
        mean_dist += (obstacle - origin).norm();
        sampled_surface.octree.updateNode(obstacle, true);
      }
      direction.rotate_IP (0,0,_1DEG);
    }
    direction.rotate_IP (0,_1DEG,0);
  }
  cout << "done." << endl;

  mean_dist /= (double) hit;
  std::cout << " hits / misses: " << hit  << " / " << miss << std::endl;
  std::cout << " mean obstacle dist: " << mean_dist << std::endl;

  cout << "writing sampled_surface.bt" << endl;
  sampled_surface.octree.writeBinary("sampled_surface.bt");


	// -----------------------------------------------
	cout << "\ngenerating single rays..." << endl;
	OcTreeROS single_beams(0.03333);
	int num_beams = 17;
	double beamLength = 10.0;
	point3d single_origin (1.0, 0.45, 0.45);
	geometry_msgs::Point single_originPt = pointOctomapToMsg(single_origin);
	point3d single_end(beamLength, 0.0, 0.0);
	
	
	for (int i=0; i<num_beams; i++) {
    geometry_msgs::Point single_endPt = pointOctomapToMsg(single_origin+single_end);
	  if (!single_beams.insertRay(single_originPt, single_endPt)) {
				cout << "ERROR while inserting ray from " << single_origin << " to " << single_end << endl;
	  }
			single_end.rotate_IP (0,0,2.0*M_PI/num_beams);
		}
	
	  cout << "done." << endl;
		cout << "writing to beams.bt..." << endl;
		single_beams.octree.writeBinary("beams.bt");

}
