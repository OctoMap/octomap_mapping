## Overview

General information about !OctoMap is available at
<http://octomap.github.com> and in the publication
["OctoMap: An Efficient Probabilistic 3D Mapping Framework Based on Octrees"](http://www.informatik.uni-freiburg.de/~hornunga/pub/hornung13auro.pdf)
by A. Hornung, K. M. Wurm, M. Bennewitz, C. Stachniss, and W. Burgard
(Autonomous Robots Journal, 2013).

BibTeX: 

```
@ARTICLE{hornung13auro,
  author = {Armin Hornung and Kai M. Wurm and Maren Bennewitz and Cyrill
  Stachniss and Wolfram Burgard},
  title = {{OctoMap}: An Efficient Probabilistic {3D} Mapping Framework Based
  on Octrees},
  journal = {Autonomous Robots},
  year = 2013,
  url = {http://octomap.github.com},
  doi = {10.1007/s10514-012-9321-0},
  note = {Software available at \url{http://octomap.github.com}}
}
```

 Please cite our paper if you use
!OctoMap in your research.

## ROS Node API

### dynamic_reconfigure API

**octomap_server** and **octomap_server_multilayer** offer a
dynamic_reconfigure interface to change the displayed map resolution on
the fly (since version 0.3.8). Note that this will not change the
resolution of the underlying OctoMap, but only of the published marker /
collision topics (e.g. for visualization).

### octomap_server

octomap_server builds and distributes volumetric 3D occupancy maps as
!OctoMap binary stream and in various ROS-compatible formats e.g. for
obstacle avoidance or visualization. The map can be a static !OctoMap
.bt file (as command line argument) or can be incrementally built from
incoming range data (as !PointCloud2). octomap_server starts with an
empty map if no command line argument is given. In general,
octomap_server creates and publishes only on topics that are subscribed.
Since some can be time-consuming to build for large maps, only subscribe
to topics you absolutely need (e.g. in RViz) and set the "latch"
parameter for false when building maps.

#### Subscribed Topics

- `cloud_in`
  ([sensor_msgs/msg/PointCloud2](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/PointCloud2.html)):
  Incoming 3D point cloud for scan integration. You need to remap this
  topic to your sensor data and provide a tf transform between the
  sensor data and the static map frame. The frame_id of the point cloud
  has to be the sensor frame. Raytracing (clearing of free space) is
  always done from the origin of this frame at the time of the point
  cloud.

#### Published Topics

- `octomap_binary`
  ([octomap_msgs/msg/Octomap](https://docs.ros.org/en/rolling/p/octomap_msgs/msg/Octomap.html)):
  The complete maximum-likelihood occupancy map as compact !OctoMap
  binary stream, encoding free and occupied space. The binary message
  only distinguishes between free and occupied space but is smaller. See
  [octomap_msgs](https://index.ros.org/p/octomap_msgs) for deserializing
  the message.
- `octomap_full`
  ([octomap_msgs/msg/Octomap](https://docs.ros.org/en/rolling/p/octomap_msgs/msg/Octomap.html)):
  The complete maximum-likelihood occupancy map as compact !OctoMap
  binary stream, encoding free and occupied space. The full message
  contains the complete probabilities and all additional data stored in
  the tree. See [octomap_msgs](https://index.ros.org/p/octomap_msgs) for
  deserializing the message.
- `occupied_cells_vis_array`
  ([visualization_msgs/msg/MarkerArray](https://docs.ros.org/en/rolling/p/visualization_msgs/msg/MarkerArray.html)):
  All occupied voxels as "box" markers for visualization in RViz. Be
  sure to subscribe to the topic **occupied_cells_vis** in RViz!
- `octomap_point_cloud_centers`
  ([sensor_msgs/msg/PointCloud2](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/PointCloud2.html)):
  The centers of all occupied voxels as point cloud, useful for
  visualization. Note that this will have gaps as the points have no
  volumetric size and !OctoMap voxels can have different resolutions!
  Use the !MarkerArray topic instead.
- `map (up to fuerte) / projected_map (since fuerte)`
  ([nav_msgs/msg/OccupancyGrid](https://docs.ros.org/en/rolling/p/nav_msgs/msg/OccupancyGrid.html)):
  Downprojected 2D occupancy map from the 3D map. Be sure to remap this
  topic if you have another 2D map server running. **New / changed in
  octomap_mapping 0.4.4**: The topic is now `projected_map` by default
  to avoid collisions with static 2D maps

#### Services

- `octomap_binary`
  ([octomap_msgs/srv/GetOctomap](https://docs.ros.org/en/rolling/p/octomap_msgs/srv/GetOctomap.html)):
  The complete maximum-likelihood occupancy map as compact !OctoMap
  binary stream, encoding free and occupied space.
- `~clear_bbx`
  ([octomap_msgs/srv/BoundingBoxQuery](https://docs.ros.org/en/rolling/p/octomap_msgs/srv/BoundingBoxQuery.html)):
  Clears a region in the 3D occupancy map, setting all voxels in the
  region to "free"
- `~reset`
  ([std_srvs/srv/Empty](https://docs.ros.org/en/rolling/p/std_srvs/srv/Empty.html)):
  Resets the complete map

#### Parameters

- `~frame_id` (`string`, default: `/map`): Static global frame in which
  the map will be published. A transform from sensor data to this frame
  needs to be available when dynamically building maps.
- `~resolution` (`float`, default: `0.05`): Resolution in meter for the
  map when starting with an empty map. Otherwise the loaded file's
  resolution is used.
- `~base_frame_id` (`string`, default: `base_footprint`): The robot's
  base frame in which ground plane detection is performed (if enabled)
- `~height_map` (`bool`, default: `true`): Whether visualization should
  encode height with different colors
- `~color/[r/g/b/a]` (`float`): Color for visualizing occupied cells
  when heigh_map=False, in range \[0:1\]
- `~sensor_model/max_range` (`float`, default: `-1 (unlimited)`):
  Maximum range in meter for inserting point cloud data when dynamically
  building a map. Limiting the range to something useful (e.g. 5m)
  prevents spurious erroneous points far away from the robot.
- `~sensor_model/[hit|miss]` (`float`, default: `0.7 / 0.4`):
  Probabilities for hits and misses in the sensor model when dynamically
  building a map
- `~sensor_model/[min|max]` (`float`, default: `0.12 / 0.97`): Minimum
  and maximum probability for clamping when dynamically building a map
- `~latch` (`bool`, default:
  `True for a static map, false if no initial map is given`): Whether
  topics are published latched or only once per change. For maximum
  performance when building a map (with frequent updates), set to false.
  When set to true, on every map change all topics and visualizations
  will be created.
- `~filter_ground` (`bool`, default: `false`): Whether the ground plane
  should be detected and ignored from scan data when dynamically
  building a map, using pcl::SACMODEL_PERPENDICULAR_PLANE. This clears
  everything up to the ground, but will not insert the ground as
  obstacle in the map. If this is enabled, it can be further configured
  with the ground_filter/\... parameters.
- `~ground_filter/distance` (`float`, default: `0.04`): Distance
  threshold for points (in z direction) to be segmented to the ground
  plane
- `~ground_filter/angle` (`float`, default: `0.15`): Angular threshold
  of the detected plane from the horizontal plane to be detected as
  ground
- `~ground_filter/plane_distance` (`float`, default: `0.07`): Distance
  threshold from z=0 for a plane to be detected as ground (4th
  coefficient of the plane equation from PCL)
- `~pointcloud_[min|max]_z` (`float`, default: `-/+ infinity`): Minimum
  and maximum height of points to consider for insertion in the
  callback. Any point outside of this intervall will be discarded before
  running any insertion or ground plane filtering. You can do a rough
  filtering based on height with this, but if you enable the
  ground_filter this interval needs to include the ground plane.
- `~occupancy_[min|max]_z` (`float`, default: `-/+ infinity`): Minimum
  and maximum height of occupied cells to be consider in the final map.
  This ignores all occupied voxels outside of the interval when sending
  out visualizations and collision maps, but will not affect the actual
  octomap representation.

#### Required tf Transforms

- `sensor data frame` →
  `/map (static world frame, changeable with parameter frame_id)`:
  Required transform of sensor data into the global map frame if you do
  scan integration. This information needs to be available from an
  external SLAM or localization node.

### octomap_server_multilayer

An extension of octomap_server to publish multilayer projected 2D maps,
used by the [3d_navigation](3d_navigation) stack. Subscriptions and
publications are mostly identical to the octomap_server node.

### octomap_saver

A small command line tool to request octomap files from an
octomap_server, similar to
[map_server](https://index.ros.org/p/map_server). Just run
"octomap_saver mapfile.bt" to request a compressed binary octomap via
service call and save it to mapfile.bt. Run "octomap_saver -f
mapfile.ot" to request a full probability octomap instead (Requires
version 0.5 or later).

## Report a Bug

Use the octomap
[issue tracker](https://github.com/OctoMap/octomap_mapping/issues) to
report bugs or request features. For questions (and FAQ), check
[answers.ros.org](http://answers.ros.org/questions/scope:all/sort:activity-desc/tags:octomap/page:1/).
