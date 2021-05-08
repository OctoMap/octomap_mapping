//
// Created by jbs on 21. 5. 7..
//

#ifndef OCTOMAP_SERVER_EDTOCTOMAPSERVER_H
#define OCTOMAP_SERVER_EDTOCTOMAPSERVER_H

#include "octomap_server/OctomapServer.h"
#include <dynamicEDT3D/dynamicEDTOctomap.h>
#include <pcl_ros/point_cloud.h>
#include <octomap_server/Utils.h>

#define MAX_SCALE 10
#define MIN_SCALE (-10)

namespace octomap_server{
    struct Param{
        //! Edt initialize
        float maxDist = 2.0;
        bool unknownAsOccupied = false;
        octomap::point3d minBound;
        octomap::point3d maxBound;

        //! visualization
        bool publishGlobal = false;
        double sliceLevel; // for global
        bool publishLocal = true;
        octomap::point3d minWindowBound;
        octomap::point3d maxWindowBound;
        double safeMargin = 1.4;

        //! Report
        bool verbose = false; //! report verbose

    };

    class EdtOctomapServer: public OctomapServer{
    public:
        EdtOctomapServer(const std::string& filename = "");
        virtual ~EdtOctomapServer() {};

        void publish();

    protected:
        Param param;
        DynamicEDTOctomap* edtPtr;
        ros::Publisher pubSlice;
        ros::Publisher pubSliceLocal;

        //! For global visualization at a slice height
        pcl::PointCloud<pcl::PointXYZI> sliceGlobal;
        //! For local visualization at a bounding volume
        pcl::PointCloud<pcl::PointXYZI> sliceLocal;

        //! Update visualization for entire boundary
        void updateSliceGlobal(float height);
        //! Update visualization for entire boundary
        void updateSliceWindow(octomap::point3d minWinBound, octomap::point3d maxWinBound, int* numQuery = NULL);
        //! Include edt update on pcl callback routine
        void insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud) override;

    };
}


#endif //OCTOMAP_SERVER_EDTOCTOMAPSERVER_H
