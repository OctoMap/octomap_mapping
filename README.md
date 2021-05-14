octomap_edt_server ![CI](https://github.com/OctoMap/octomap_mapping/workflows/CI/badge.svg)
===============
Written by JBS.
## Installation 

### Depends on my git fork of dynamicEDT3D
```
sudo apt-get install ros-melodic-octomap-ros
git clone https://github.com/icsl-Jeon/octomap.git
cd dynamicEDT3D && mkdir build && cd build && cmake ..
sudo make install
```

### Build this package 
```
https://github.com/icsl-Jeon/octomap_mapping.git
catkin build octomap_server 
```

## Out-of-box launch for [octomap_edt_server](octomap_server/launch/octomap_edt_server.launch)

Download [bag_file](https://drive.google.com/file/d/1AwMVekkpmsFcpDjmPqnEeNCCN2xaXQPk/view?usp=sharing) (2.9GB) and
set correct path to it here:
```
<arg name="bag_file" value=""/>
```
Then launch the file:

```
roslaunch octomap_server octomap_edt_server.launch
```

## Features 
* [DynamicEDTServer](octomap_server/src/EdtOctomapServer.cpp) : incrementally build EDF (note NOT ESDF) 
using [dynamicEDT3D](https://github.com/OctoMap/octomap/tree/devel/dynamicEDT3D
  )
 * Real-time analysis on your terminal  as below pretty table: 
 * All parameterization to cater your usages.
```
**********************************************************************
cloud callback                                                        
**********************************************************************
proc.                     pnts      time [ms]
octomap           252672 (raw)         111 ms
edt               23906 (upd.)          26 ms
edt. vis           298 (query)           1 ms
======================================================================
```

* EDF visualization

![intro](octomap_server/img/intro.gif)


## Wiki

### EDF from dynamicEDT3D
* getDistance : if query is out of fixed bound of EDT, `-1` is returned.
