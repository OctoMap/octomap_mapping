octomap_mapping ![CI](https://github.com/OctoMap/octomap_mapping/workflows/CI/badge.svg)
===============

## Installation 

* Require my git fork of dynamicEDT3D
```
git clone https://github.com/icsl-Jeon/octomap.git
cd dynamicEDT3D && mkdir build && cd build && cmake ..
sudo make install
```

## Features 
* [DynamicEDTServer](octomap_server/src/EdtOctomapServer.cpp) : incrementally build EDF (note NOT ESDF) 
using [dynamicEDT3D](https://github.com/OctoMap/octomap/tree/devel/dynamicEDT3D
  )
 * Real-time analysis on your terminal  as below pretty table: 
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
