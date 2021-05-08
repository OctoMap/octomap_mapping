//
// Created by jbs on 21. 5. 7..
//

#include <octomap_server/EdtOctomapServer.h>


using namespace octomap;

namespace octomap_server {

    void EdtOctomapServer::init() {

        m_nh_private.getParam("max_distance",param.maxDist);
        m_nh_private.getParam("verbose",param.verbose);
        m_nh_private.getParam("unknown_is_occupied",param.unknownAsOccupied);
        /**
         * In dynamicEDT3d, this bound determines the size of gridmap considered.
         * That is, it seems not to adjust total map size incrementally ?
         */
        double minX = MIN_SCALE;
        double minY = MIN_SCALE;
        double minZ = MIN_SCALE;
        double maxX = MAX_SCALE;
        double maxY = MAX_SCALE;
        double maxZ = MAX_SCALE;

        m_nh_private.getParam("init_bound/max/x",maxX);
        m_nh_private.getParam("init_bound/max/y",maxY);
        m_nh_private.getParam("init_bound/max/z",maxZ);
        m_nh_private.getParam("init_bound/min/x",minX);
        m_nh_private.getParam("init_bound/min/y",minY);
        m_nh_private.getParam("init_bound/min/z",minZ);
        // WTF: point3d.x() did not correctly insert
        param.minBound = point3d(minX,minY,minZ);
        param.maxBound = point3d(maxX,maxY,maxZ);
        m_nh_private.getParam("publish_global_edf",param.publishGlobal);
        m_nh_private.getParam("slice_level",param.sliceLevel);


        m_nh_private.getParam("window_bound/max/x",maxX);
        m_nh_private.getParam("window_bound/max/y",maxY);
        m_nh_private.getParam("window_bound/max/z",maxZ);
        m_nh_private.getParam("window_bound/min/x",minX);
        m_nh_private.getParam("window_bound/min/y",minY);
        m_nh_private.getParam("window_bound/min/z",minZ);
        param.minWindowBound = point3d(minX,minY,minZ);
        param.maxWindowBound = point3d(maxX,maxY,maxZ);
        m_nh_private.getParam("publish_local_edf",param.publishLocal);
        m_nh_private.getParam("safe_margin",param.safeMargin);


        pubSlice = m_nh_private.advertise<pcl::PointCloud<pcl::PointXYZI>>("edf_slice_global",1);
        pubSliceLocal = m_nh_private.advertise<pcl::PointCloud<pcl::PointXYZI>>("edf_slice_local",1);

        edtPtr = new DynamicEDTOctomap(param.maxDist,dynamic_cast<octomap::OcTree*> (m_octree),
                                       param.minBound,param.maxBound,
                                       param.unknownAsOccupied);

    }

    EdtOctomapServer::EdtOctomapServer(const ros::NodeHandle &nh_private) : OctomapServer(nh_private) {
        init();
    }

    EdtOctomapServer::EdtOctomapServer(const std::string &filename)   {

        init();

        if (filename != "") {
            if (m_octree->readBinary(filename)) {
            //! this will delete the existing node. Thus, assigning new for edtPtr before this will not work if you have prior
                ROS_INFO("Octomap file %s loaded (%zu nodes).", filename.c_str(), m_octree->size());
                delete edtPtr;
                edtPtr = new DynamicEDTOctomap(param.maxDist,dynamic_cast<octomap::OcTree*> (m_octree),
                                               param.minBound,param.maxBound,
                                               param.unknownAsOccupied);

                m_treeDepth = m_octree->getTreeDepth();
                m_res = m_octree->getResolution();
                m_gridmap.info.resolution = m_res;
                edtPtr->update();
                ROS_INFO("EDT complete.");
                updateSliceGlobal(param.sliceLevel);

                publishAll(); //! publisher of OctomapServer
                publish();

            } else {
                ROS_ERROR("Could not open requested file %s, exiting.", filename.c_str());
                exit(-1);
            }
        }
    }


    void EdtOctomapServer::updateSliceGlobal(float height) {

        sliceGlobal.clear();
        double resolution = m_octree->getResolution();
        int bbx_nx = floor((param.maxBound.x() - param.minBound.x()) /resolution);
        int bbx_ny = floor((param.maxBound.y() - param.minBound.y()) / resolution);
        double xmin = param.minBound.x();
        double ymin = param.minBound.y();

        int n = 0;
        for (int nx = 0; nx <= bbx_nx; nx++)
            for (int ny = 0; ny <= bbx_ny; ny++) {
                octomap::point3d pnt(xmin + nx * resolution, ymin + ny *resolution, height);
                geometry_msgs::Point point;
                point.x = pnt.x();
                point.y = pnt.y();
                point.z = pnt.z();
                double dist =edtPtr->getDistance(pnt);
                if (dist > 0) {
                    pcl::PointXYZI pntPcl;
                    pntPcl.intensity = dist;
                    pntPcl.x = pnt.x();
                    pntPcl.y = pnt.y();
                    pntPcl.z = pnt.z();
                    sliceGlobal.points.push_back(pntPcl);
                    n++;
                }
            }

        std::cout << "updating edf slice for global visualization: " << n << " points"<<std::endl;

    }

    /**
     * Visualize EDF around sensor origin (tf lookuped at pcl callback)
     * @param minWinBound origin - minWindBound
     * @param maxWinBound origin + maxWindBound
     */
    void EdtOctomapServer::updateSliceWindow(octomap::point3d minWinBound, octomap::point3d maxWinBound,
                                             int* numQuery) {
        sliceLocal.header.frame_id = m_worldFrameId;
        sliceLocal.clear();
        double resolution = m_octree->getResolution();
        int bbx_nx = floor((maxWinBound.x() - minWinBound.x()) /resolution);
        int bbx_ny = floor((maxWinBound.y() - minWinBound.y()) / resolution);
        int bbx_nz = floor((maxWinBound.z() - minWinBound.z()) / resolution);

        int n = 0;
        for (int nx = 0; nx <= bbx_nx; nx++)
            for (int ny = 0; ny <= bbx_ny; ny++)
                for (int nz = 0; nz <= bbx_nz; nz++) {
                    float x = minWinBound.x() + resolution * nx + m_sensorToWorldTf.coeffRef(0,3);
                    float y = minWinBound.y() + resolution * ny + m_sensorToWorldTf.coeffRef(1,3);
                    float z = minWinBound.z() + resolution * nz + m_sensorToWorldTf.coeffRef(2,3);
                    pcl::PointXYZI pclPnt; pclPnt.x = x; pclPnt.y = y ; pclPnt.z = z;
                    octomap::point3d pnt(x,y,z);
                    double dist =edtPtr->getDistance(pnt);
                    pclPnt.intensity = dist;
                    if (dist > 0 and dist < param.safeMargin ) { //! for only in bound points and whose dist < safeMargin
                        n++;
                        sliceLocal.push_back(pclPnt);
                    }
                }
        *numQuery = n;
    }


    void EdtOctomapServer::publish() {
        sliceGlobal.header.stamp = pcl_conversions::toPCL(ros::Time::now());
        sliceGlobal.header.frame_id = m_worldFrameId;
        sliceLocal.header.stamp = pcl_conversions::toPCL(ros::Time::now());
        if (param.publishGlobal)
        pubSlice.publish(sliceGlobal);
        if (param.publishLocal)
        pubSliceLocal.publish(sliceLocal);
    }


    void EdtOctomapServer::insertCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud) {
        Timer timer;
        OctomapServer::insertCloudCallback(cloud);
        short  t0 = timer.stop();
        int numUpdatePnt;
        edtPtr->update(true,false,&numUpdatePnt);
        short t1 = timer.stop();
        short t2; int numQuery;
        if (param.publishGlobal)
            updateSliceGlobal(param.sliceLevel);
        if (param.publishLocal){
            Timer timerNew;
            updateSliceWindow(param.minWindowBound,param.maxWindowBound,&numQuery);
            t2 = timerNew.stop();
        }
        int tableWidth = 4;
        if (param.verbose){

            TOP_RULE_STAR(tableWidth);
            TITLE(tableWidth,"cloud callback")
            TOP_RULE_STAR(tableWidth);

            FILL_CELL_LEFT("proc.")
            FILL_CELL_RIGHT("pnts") FILL_CELL_RIGHT("time [ms]")
            NEW_LINE

            FILL_CELL_LEFT("octomap")
            FILL_CELL_RIGHT(std::to_string(cloud->width*cloud->height) + " (raw)") FILL_CELL_RIGHT(std::to_string(t0) + " ms")
            NEW_LINE

            FILL_CELL_LEFT("edt")
            FILL_CELL_RIGHT(std::to_string(numQuery) + " (upd.)")
            FILL_CELL_RIGHT(std::to_string(t1-t0) + " ms")
            NEW_LINE

            if (param.publishLocal) {
                FILL_CELL_LEFT("edt. vis")
                FILL_CELL_RIGHT(std::to_string(numUpdatePnt) + " (query)")
                FILL_CELL_RIGHT(std::to_string(t2) + " ms")
                NEW_LINE
            }


            BOTTOM_RULE_EQ(tableWidth);
        }



        publish();
    }
    float EdtOctomapServer::getDistance(octomap::point3d pnt) {
        return edtPtr->getDistance(pnt);
    }











}