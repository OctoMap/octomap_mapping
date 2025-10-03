// Copyright 2010-2013, A. Hornung, University of Freiburg. All rights reserved.
//
// [License text same as original...]

#include <octomap_server/octomap_server_lifecycle.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace
{
    template <typename T>
    bool update_param(const std::vector<rclcpp::Parameter> &p, const std::string &name, T &value)
    {
        auto it = std::find_if(
            p.cbegin(), p.cend(), [&name](const rclcpp::Parameter &parameter)
            { return parameter.get_name() == name; });
        if (it != p.cend())
        {
            value = it->template get_value<T>();
            return true;
        }
        return false;
    }
} // namespace

namespace octomap_server
{
    OctomapServerLifecycle::OctomapServerLifecycle(const rclcpp::NodeOptions &node_options)
        : LifecycleNode("octomap_server", node_options)
    {
        RCLCPP_INFO(get_logger(), "Creating OctomapServerLifecycle");

        // Declare all parameters in constructor (before configuration)
        world_frame_id_ = declare_parameter("frame_id", "map");
        base_frame_id_ = declare_parameter("base_frame_id", "base_footprint");
        use_height_map_ = declare_parameter("use_height_map", false);
        use_colored_map_ = declare_parameter("colored_map", false);
        color_factor_ = declare_parameter("color_factor", 0.8);

        point_cloud_min_x_ = declare_parameter("point_cloud_min_x", -std::numeric_limits<double>::max());
        point_cloud_max_x_ = declare_parameter("point_cloud_max_x", std::numeric_limits<double>::max());
        point_cloud_min_y_ = declare_parameter("point_cloud_min_y", -std::numeric_limits<double>::max());
        point_cloud_max_y_ = declare_parameter("point_cloud_max_y", std::numeric_limits<double>::max());
        point_cloud_min_z_ = declare_parameter("point_cloud_min_z", -100.0);
        point_cloud_max_z_ = declare_parameter("point_cloud_max_z", 100.0);
        occupancy_min_z_ = declare_parameter("occupancy_min_z", -100.0);
        occupancy_max_z_ = declare_parameter("occupancy_max_z", 100.0);
        min_x_size_ = declare_parameter("min_x_size", 0.0);
        min_y_size_ = declare_parameter("min_y_size", 0.0);

        filter_speckles_ = declare_parameter("filter_speckles", false);
        filter_ground_plane_ = declare_parameter("filter_ground_plane", false);
        ground_filter_distance_ = declare_parameter("ground_filter.distance", 0.04);
        ground_filter_angle_ = declare_parameter("ground_filter.angle", 0.15);
        ground_filter_plane_distance_ = declare_parameter("ground_filter.plane_distance", 0.07);
        max_range_ = declare_parameter("sensor_model.max_range", -1.0);

        res_ = declare_parameter("resolution", 0.05);
        auto filename_ = declare_parameter("octomap_path", "");

        const double prob_hit = declare_parameter("sensor_model.hit", 0.7);
        const double prob_miss = declare_parameter("sensor_model.miss", 0.4);
        const double thres_min = declare_parameter("sensor_model.min", 0.12);
        const double thres_max = declare_parameter("sensor_model.max", 0.97);

        compress_map_ = declare_parameter("compress_map", true);
        incremental_2D_projection_ = declare_parameter("incremental_2D_projection", false);

        color_.r = declare_parameter("color.r", 0.0);
        color_.g = declare_parameter("color.g", 0.0);
        color_.b = declare_parameter("color.b", 1.0);
        color_.a = declare_parameter("color.a", 1.0);

        color_free_.r = declare_parameter("color_free.r", 0.0);
        color_free_.g = declare_parameter("color_free.g", 1.0);
        color_free_.b = declare_parameter("color_free.b", 0.0);
        color_free_.a = declare_parameter("color_free.a", 1.0);

        publish_free_space_ = declare_parameter("publish_free_space", false);
        latched_topics_ = declare_parameter("latch", true);

        // Initialize octree with sensor model parameters
        octree_ = std::make_unique<OcTreeT>(res_);
        octree_->setProbHit(prob_hit);
        octree_->setProbMiss(prob_miss);
        octree_->setClampingThresMin(thres_min);
        octree_->setClampingThresMax(thres_max);
        tree_depth_ = octree_->getTreeDepth();
        max_tree_depth_ = tree_depth_;

        declare_parameter("max_depth", static_cast<int64_t>(max_tree_depth_));
        gridmap_.info.resolution = res_;
    }

    CallbackReturn OctomapServerLifecycle::on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Configuring OctomapServerLifecycle");

        using std::placeholders::_1;
        using std::placeholders::_2;

        // Create TF2 buffer and listener
        tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
            this->get_node_base_interface(),
            this->get_node_timers_interface());
        tf2_buffer_->setCreateTimerInterface(timer_interface);
        tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

        // Create lifecycle publishers
        auto qos = latched_topics_ ? rclcpp::QoS{1}.transient_local() : rclcpp::QoS{1};
        marker_pub_ = create_publisher<MarkerArray>("occupied_cells_vis_array", qos);
        binary_map_pub_ = create_publisher<Octomap>("octomap_binary", qos);
        full_map_pub_ = create_publisher<Octomap>("octomap_full", qos);
        point_cloud_pub_ = create_publisher<PointCloud2>("octomap_point_cloud_centers", qos);
        map_pub_ = create_publisher<OccupancyGrid>("projected_map", qos.keep_last(5));
        fmarker_pub_ = create_publisher<MarkerArray>("free_cells_vis_array", qos);

        // Create services
        octomap_binary_srv_ = create_service<OctomapSrv>(
            "octomap_binary", std::bind(&OctomapServerLifecycle::onOctomapBinarySrv, this, _1, _2));
        octomap_full_srv_ = create_service<OctomapSrv>(
            "octomap_full", std::bind(&OctomapServerLifecycle::onOctomapFullSrv, this, _1, _2));
        clear_bbox_srv_ = create_service<BBoxSrv>(
            "~/clear_bbox", std::bind(&OctomapServerLifecycle::clearBBoxSrv, this, _1, _2));
        reset_srv_ = create_service<ResetSrv>(
            "~/reset", std::bind(&OctomapServerLifecycle::resetSrv, this, _1, _2));

        // Setup message filter for point cloud subscription
        using std::chrono_literals::operator""s;
        point_cloud_sub_.subscribe(this, "cloud_in", rmw_qos_profile_sensor_data);
        tf_point_cloud_sub_ = std::make_shared<tf2_ros::MessageFilter<PointCloud2>>(
            point_cloud_sub_, *tf2_buffer_, world_frame_id_, 5, this->get_node_logging_interface(),
            this->get_node_clock_interface(), 5s);

        // Set parameter callback
        set_param_res_ = this->add_on_set_parameters_callback(
            std::bind(&OctomapServerLifecycle::onParameter, this, _1));

        // Try to load octomap file if specified
        const auto filename = get_parameter("octomap_path").as_string();
        if (!filename.empty() && !openFile(filename))
        {
            RCLCPP_WARN(get_logger(), "Could not open file %s", filename.c_str());
        }

        RCLCPP_INFO(get_logger(), "OctomapServerLifecycle configured successfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn OctomapServerLifecycle::on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Activating OctomapServerLifecycle");

        // Activate all lifecycle publishers
        marker_pub_->on_activate();
        binary_map_pub_->on_activate();
        full_map_pub_->on_activate();
        point_cloud_pub_->on_activate();
        map_pub_->on_activate();
        fmarker_pub_->on_activate();

        // Register the point cloud callback
        tf_point_cloud_sub_->registerCallback(
            &OctomapServerLifecycle::insertCloudCallback, this);

        // Publish initial map if loaded from file
        if (octree_->size() > 1)
        {
            publishAll(now());
        }

        RCLCPP_INFO(get_logger(), "OctomapServerLifecycle activated successfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn OctomapServerLifecycle::on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Deactivating OctomapServerLifecycle");

        // Unregister callback
        point_cloud_sub_.unsubscribe();
        // Deactivate all lifecycle publishers
        marker_pub_->on_deactivate();
        binary_map_pub_->on_deactivate();
        full_map_pub_->on_deactivate();
        point_cloud_pub_->on_deactivate();
        map_pub_->on_deactivate();
        fmarker_pub_->on_deactivate();

        RCLCPP_INFO(get_logger(), "OctomapServerLifecycle deactivated successfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn OctomapServerLifecycle::on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(get_logger(), "Cleaning up OctomapServerLifecycle");

        // Reset publishers
        marker_pub_.reset();
        binary_map_pub_.reset();
        full_map_pub_.reset();
        point_cloud_pub_.reset();
        map_pub_.reset();
        fmarker_pub_.reset();

        // Reset services
        octomap_binary_srv_.reset();
        octomap_full_srv_.reset();
        clear_bbox_srv_.reset();
        reset_srv_.reset();

        // Reset TF components
        tf_point_cloud_sub_.reset();
        tf2_listener_.reset();
        tf2_buffer_.reset();

        // Clear the octree
        if (octree_)
        {
            octree_->clear();
        }

        RCLCPP_INFO(get_logger(), "OctomapServerLifecycle cleaned up successfully");
        return CallbackReturn::SUCCESS;
    }

    CallbackReturn OctomapServerLifecycle::on_shutdown(const rclcpp_lifecycle::State &state)
    {
        RCLCPP_INFO(get_logger(), "Shutting down OctomapServerLifecycle");

        // Perform cleanup based on current state
        if (state.label() == "active")
        {
            on_deactivate(state);
        }
        if (state.label() == "inactive")
        {
            on_cleanup(state);
        }

        return CallbackReturn::SUCCESS;
    }

    // Service callbacks remain the same as original implementation
    bool OctomapServerLifecycle::onOctomapBinarySrv(
        [[maybe_unused]] const std::shared_ptr<OctomapSrv::Request> req,
        const std::shared_ptr<OctomapSrv::Response> res)
    {
        const auto start_time = rclcpp::Clock{}.now();
        RCLCPP_INFO(get_logger(), "Sending binary map data on service request");
        res->map.header.frame_id = world_frame_id_;
        res->map.header.stamp = now();
        if (!octomap_msgs::binaryMapToMsg(*octree_, res->map))
        {
            return false;
        }

        double total_elapsed = (rclcpp::Clock{}.now() - start_time).seconds();
        RCLCPP_INFO(get_logger(), "Binary octomap sent in %f sec", total_elapsed);
        return true;
    }

    bool OctomapServerLifecycle::onOctomapFullSrv(
        [[maybe_unused]] const std::shared_ptr<OctomapSrv::Request> req,
        const std::shared_ptr<OctomapSrv::Response> res)
    {
        RCLCPP_INFO(get_logger(), "Sending full map data on service request");
        res->map.header.frame_id = world_frame_id_;
        res->map.header.stamp = now();

        if (!octomap_msgs::fullMapToMsg(*octree_, res->map))
        {
            return false;
        }

        return true;
    }

    // Additional methods: insertCloudCallback, insertScan, publishAll, etc.
    // These remain largely the same as the original implementation
    // Just ensure they check if the node is active before publishing

    void OctomapServerLifecycle::insertCloudCallback(const PointCloud2::ConstSharedPtr cloud)
    {
        // Check if node is in active state
        if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 5000,
                "Received point cloud but node is not active. Ignoring.");
            return;
        }

        const auto start_time = rclcpp::Clock{}.now();

        PCLPointCloud pc;
        pcl::fromROSMsg(*cloud, pc);

        geometry_msgs::msg::TransformStamped sensor_to_world_transform_stamped;
        try
        {
            sensor_to_world_transform_stamped = tf2_buffer_->lookupTransform(
                world_frame_id_, cloud->header.frame_id, cloud->header.stamp,
                rclcpp::Duration::from_seconds(1.0));
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "%s", ex.what());
            return;
        }

        // set up filter for height range, also removes NANs:
        pcl::PassThrough<PCLPoint> pass_x;
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(point_cloud_min_x_, point_cloud_max_x_);
        pcl::PassThrough<PCLPoint> pass_y;
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(point_cloud_min_y_, point_cloud_max_y_);
        pcl::PassThrough<PCLPoint> pass_z;
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(point_cloud_min_z_, point_cloud_max_z_);

        PCLPointCloud pc_ground;    // segmented ground plane
        PCLPointCloud pc_nonground; // everything else

        if (filter_ground_plane_)
        {
            geometry_msgs::msg::TransformStamped sensor_to_base_transform_stamped;
            geometry_msgs::msg::TransformStamped base_to_world_transform_stamped;
            try
            {
                tf2_buffer_->canTransform(
                    base_frame_id_, cloud->header.frame_id, cloud->header.stamp,
                    rclcpp::Duration::from_seconds(0.2));
                sensor_to_base_transform_stamped = tf2_buffer_->lookupTransform(
                    base_frame_id_, cloud->header.frame_id, cloud->header.stamp,
                    rclcpp::Duration::from_seconds(1.0));
                base_to_world_transform_stamped = tf2_buffer_->lookupTransform(
                    world_frame_id_, base_frame_id_, cloud->header.stamp,
                    rclcpp::Duration::from_seconds(1.0));
            }
            catch (const tf2::TransformException &ex)
            {
                RCLCPP_ERROR_STREAM(
                    get_logger(),
                    "Transform error for ground plane filter: " << ex.what() << ", quitting callback.\n"
                                                                                "You need to set the base_frame_id or disable filter_ground.");
            }

            // transform pointcloud from sensor frame to fixed robot frame
            pcl_ros::transformPointCloud(pc, pc, sensor_to_base_transform_stamped);
            pass_x.setInputCloud(pc.makeShared());
            pass_x.filter(pc);
            pass_y.setInputCloud(pc.makeShared());
            pass_y.filter(pc);
            pass_z.setInputCloud(pc.makeShared());
            pass_z.filter(pc);
            filterGroundPlane(pc, pc_ground, pc_nonground);

            // transform clouds to world frame for insertion
            pcl_ros::transformPointCloud(pc_ground, pc_ground, base_to_world_transform_stamped);
            pcl_ros::transformPointCloud(pc_nonground, pc_nonground, base_to_world_transform_stamped);
        }
        else
        {
            // directly transform to map frame:
            pcl_ros::transformPointCloud(pc, pc, sensor_to_world_transform_stamped);

            // just filter height range:
            pass_x.setInputCloud(pc.makeShared());
            pass_x.filter(pc);
            pass_y.setInputCloud(pc.makeShared());
            pass_y.filter(pc);
            pass_z.setInputCloud(pc.makeShared());
            pass_z.filter(pc);

            pc_nonground = pc;
            // pc_nonground is empty without ground segmentation
            pc_ground.header = pc.header;
            pc_nonground.header = pc.header;
        }

        const auto &t = sensor_to_world_transform_stamped.transform.translation;
        tf2::Vector3 sensor_to_world_vec3{t.x, t.y, t.z};
        insertScan(sensor_to_world_vec3, pc_ground, pc_nonground);

        double total_elapsed = (rclcpp::Clock{}.now() - start_time).seconds();
        RCLCPP_DEBUG(
            get_logger(),
            "Pointcloud insertion in OctomapServer done (%zu+%zu pts, %f sec)",
            pc_ground.size(), pc_nonground.size(), total_elapsed);

        publishAll(cloud->header.stamp);
    }

    void OctomapServerLifecycle::insertScan(
        const tf2::Vector3 &sensor_origin_tf, const PCLPointCloud &ground,
        const PCLPointCloud &nonground)
    {
        const auto sensor_origin = octomap::pointTfToOctomap(sensor_origin_tf);

        if (!octree_->coordToKeyChecked(sensor_origin, update_bbox_min_) ||
            !octree_->coordToKeyChecked(sensor_origin, update_bbox_max_))
        {
            RCLCPP_ERROR_STREAM(get_logger(), "Could not generate Key for origin " << sensor_origin);
        }

        octomap::KeySet free_cells, occupied_cells;

        // Insert ground points only as free
        for (PCLPointCloud::const_iterator it = ground.begin(); it != ground.end(); ++it)
        {
            octomap::point3d point(it->x, it->y, it->z);
            if ((max_range_ > 0.0) && ((point - sensor_origin).norm() > max_range_))
            {
                point = sensor_origin + (point - sensor_origin).normalized() * max_range_;
            }

            if (octree_->computeRayKeys(sensor_origin, point, key_ray_))
            {
                free_cells.insert(key_ray_.begin(), key_ray_.end());
            }

            octomap::OcTreeKey end_key;
            if (octree_->coordToKeyChecked(point, end_key))
            {
                updateMinKey(end_key, update_bbox_min_);
                updateMaxKey(end_key, update_bbox_max_);
            }
            else
            {
                RCLCPP_ERROR_STREAM(get_logger(), "Could not generate Key for endpoint " << point);
            }
        }

        // All other points: free on ray, occupied on endpoint
        for (PCLPointCloud::const_iterator it = nonground.begin(); it != nonground.end(); ++it)
        {
            octomap::point3d point(it->x, it->y, it->z);
            if ((max_range_ < 0.0) || ((point - sensor_origin).norm() <= max_range_))
            {
                if (octree_->computeRayKeys(sensor_origin, point, key_ray_))
                {
                    free_cells.insert(key_ray_.begin(), key_ray_.end());
                }

                octomap::OcTreeKey key;
                if (octree_->coordToKeyChecked(point, key))
                {
                    occupied_cells.insert(key);
                    updateMinKey(key, update_bbox_min_);
                    updateMaxKey(key, update_bbox_max_);

#ifdef COLOR_OCTOMAP_SERVER
                    octree_->averageNodeColor(it->x, it->y, it->z, it->r, it->g, it->b);
#endif
                }
            }
            else
            {
                octomap::point3d new_end = sensor_origin + (point - sensor_origin).normalized() * max_range_;
                if (octree_->computeRayKeys(sensor_origin, new_end, key_ray_))
                {
                    free_cells.insert(key_ray_.begin(), key_ray_.end());

                    octomap::OcTreeKey end_key;
                    if (octree_->coordToKeyChecked(new_end, end_key))
                    {
                        free_cells.insert(end_key);
                        updateMinKey(end_key, update_bbox_min_);
                        updateMaxKey(end_key, update_bbox_max_);
                    }
                }
            }
        }

        // Mark free cells only if not seen occupied in this cloud
        for (auto it = free_cells.begin(), end = free_cells.end(); it != end; ++it)
        {
            if (occupied_cells.find(*it) == occupied_cells.end())
            {
                octree_->updateNode(*it, false);
            }
        }

        // Mark all occupied cells
        for (auto it = occupied_cells.begin(), end = occupied_cells.end(); it != end; it++)
        {
            octree_->updateNode(*it, true);
        }

        if (compress_map_)
        {
            octree_->prune();
        }
    }

    void OctomapServerLifecycle::publishAll(const rclcpp::Time &rostime)
    {
        // Check if node is active before publishing
        if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        {
            return;
        }

        const auto start_time = rclcpp::Clock{}.now();
        const size_t octomap_size = octree_->size();

        if (octomap_size <= 1)
        {
            RCLCPP_WARN(get_logger(), "Nothing to publish, octree is empty");
            return;
        }

        bool publish_free_marker_array_ = publish_free_space_ &&
                                          (latched_topics_ || fmarker_pub_->get_subscription_count() > 0);
        bool publish_marker_array = (latched_topics_ || marker_pub_->get_subscription_count() > 0);
        bool publish_point_cloud = (latched_topics_ || point_cloud_pub_->get_subscription_count() > 0);
        bool publish_binary_map = (latched_topics_ || binary_map_pub_->get_subscription_count() > 0);
        bool publish_full_map = (latched_topics_ || full_map_pub_->get_subscription_count() > 0);
        publish_2d_map_ = (latched_topics_ || map_pub_->get_subscription_count() > 0);

        MarkerArray free_nodes_vis;
        free_nodes_vis.markers.resize(tree_depth_ + 1);

        MarkerArray occupied_nodes_vis;
        occupied_nodes_vis.markers.resize(tree_depth_ + 1);

        pcl::PointCloud<PCLPoint> pcl_cloud;

        handlePreNodeTraversal(rostime);

        // Traverse all leafs in the tree
        for (OcTreeT::iterator it = octree_->begin(max_tree_depth_),
                               end = octree_->end();
             it != end; ++it)
        {
            bool in_update_bbox = isInUpdateBBX(it);

            handleNode(it);
            if (in_update_bbox)
            {
                handleNodeInBBX(it);
            }

            if (octree_->isNodeOccupied(*it))
            {
                double z = it.getZ();
                double half_size = it.getSize() / 2.0;
                if (z + half_size > occupancy_min_z_ && z - half_size < occupancy_max_z_)
                {
                    double x = it.getX();
                    double y = it.getY();

                    if (filter_speckles_ && (it.getDepth() == tree_depth_ + 1) && isSpeckleNode(it.getKey()))
                    {
                        RCLCPP_DEBUG(get_logger(), "Ignoring single speckle at (%f,%f,%f)", x, y, z);
                        continue;
                    }

                    handleOccupiedNode(it);
                    if (in_update_bbox)
                    {
                        handleOccupiedNodeInBBX(it);
                    }

                    if (publish_marker_array)
                    {
                        unsigned idx = it.getDepth();
                        geometry_msgs::msg::Point cube_center;
                        cube_center.x = x;
                        cube_center.y = y;
                        cube_center.z = z;
                        occupied_nodes_vis.markers[idx].points.push_back(cube_center);

                        if (use_height_map_)
                        {
                            double min_x, min_y, min_z, max_x, max_y, max_z;
                            octree_->getMetricMin(min_x, min_y, min_z);
                            octree_->getMetricMax(max_x, max_y, max_z);
                            double h = (1.0 - std::min(std::max((cube_center.z - min_z) / (max_z - min_z), 0.0), 1.0)) * color_factor_;
                            occupied_nodes_vis.markers[idx].colors.push_back(heightMapColor(h));
                        }
                    }

                    if (publish_point_cloud)
                    {
                        PCLPoint _point;
                        _point.x = x;
                        _point.y = y;
                        _point.z = z;
                        pcl_cloud.push_back(_point);
                    }
                }
            }
            else
            {
                double z = it.getZ();
                double half_size = it.getSize() / 2.0;
                if (z + half_size > occupancy_min_z_ && z - half_size < occupancy_max_z_)
                {
                    handleFreeNode(it);
                    if (in_update_bbox)
                    {
                        handleFreeNodeInBBX(it);
                    }

                    if (publish_free_space_)
                    {
                        double x = it.getX();
                        double y = it.getY();

                        if (publish_free_marker_array_)
                        {
                            unsigned idx = it.getDepth();
                            geometry_msgs::msg::Point cube_center;
                            cube_center.x = x;
                            cube_center.y = y;
                            cube_center.z = z;
                            free_nodes_vis.markers[idx].points.push_back(cube_center);
                        }
                    }
                }
            }
        }

        handlePostNodeTraversal(rostime);

        // Publish marker arrays
        if (publish_marker_array)
        {
            for (size_t i = 0; i < occupied_nodes_vis.markers.size(); ++i)
            {
                double size = octree_->getNodeSize(i);
                occupied_nodes_vis.markers[i].header.frame_id = world_frame_id_;
                occupied_nodes_vis.markers[i].header.stamp = rostime;
                occupied_nodes_vis.markers[i].ns = "map";
                occupied_nodes_vis.markers[i].id = i;
                occupied_nodes_vis.markers[i].type = visualization_msgs::msg::Marker::CUBE_LIST;
                occupied_nodes_vis.markers[i].scale.x = size;
                occupied_nodes_vis.markers[i].scale.y = size;
                occupied_nodes_vis.markers[i].scale.z = size;
                if (!use_colored_map_)
                {
                    occupied_nodes_vis.markers[i].color = color_;
                }
                occupied_nodes_vis.markers[i].action = occupied_nodes_vis.markers[i].points.size() > 0 ? visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
            }
            marker_pub_->publish(occupied_nodes_vis);
        }

        if (publish_free_marker_array_)
        {
            for (size_t i = 0; i < free_nodes_vis.markers.size(); ++i)
            {
                double size = octree_->getNodeSize(i);
                free_nodes_vis.markers[i].header.frame_id = world_frame_id_;
                free_nodes_vis.markers[i].header.stamp = rostime;
                free_nodes_vis.markers[i].ns = "map";
                free_nodes_vis.markers[i].id = i;
                free_nodes_vis.markers[i].type = visualization_msgs::msg::Marker::CUBE_LIST;
                free_nodes_vis.markers[i].scale.x = size;
                free_nodes_vis.markers[i].scale.y = size;
                free_nodes_vis.markers[i].scale.z = size;
                free_nodes_vis.markers[i].color = color_free_;
                free_nodes_vis.markers[i].action = free_nodes_vis.markers[i].points.size() > 0 ? visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
            }
            fmarker_pub_->publish(free_nodes_vis);
        }

        if (publish_point_cloud)
        {
            PointCloud2 cloud;
            pcl::toROSMsg(pcl_cloud, cloud);
            cloud.header.frame_id = world_frame_id_;
            cloud.header.stamp = rostime;
            point_cloud_pub_->publish(cloud);
        }

        if (publish_binary_map)
        {
            publishBinaryOctoMap(rostime);
        }

        if (publish_full_map)
        {
            publishFullOctoMap(rostime);
        }

        double total_elapsed = (rclcpp::Clock{}.now() - start_time).seconds();
        RCLCPP_DEBUG(get_logger(), "Map publishing took %f sec", total_elapsed);
    }

    void OctomapServerLifecycle::publishBinaryOctoMap(const rclcpp::Time &rostime) const
    {
        Octomap map;
        map.header.frame_id = world_frame_id_;
        map.header.stamp = rostime;
        if (octomap_msgs::binaryMapToMsg(*octree_, map))
        {
            binary_map_pub_->publish(map);
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Error serializing OctoMap");
        }
    }

    void OctomapServerLifecycle::publishFullOctoMap(const rclcpp::Time &rostime) const
    {
        Octomap map;
        map.header.frame_id = world_frame_id_;
        map.header.stamp = rostime;
        if (octomap_msgs::fullMapToMsg(*octree_, map))
        {
            full_map_pub_->publish(map);
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Error serializing OctoMap");
        }
    }

    bool OctomapServerLifecycle::clearBBoxSrv(
        const std::shared_ptr<BBoxSrv::Request> req,
        [[maybe_unused]] const std::shared_ptr<BBoxSrv::Response> resp)
    {
        const auto min = octomap::pointMsgToOctomap(req->min);
        const auto max = octomap::pointMsgToOctomap(req->max);

        const double thres_min = octree_->getClampingThresMin();
        for (auto it = octree_->begin_leafs_bbx(min, max),
                  end = octree_->end_leafs_bbx();
             it != end; ++it)
        {
            it->setLogOdds(octomap::logodds(thres_min));
        }
        octree_->updateInnerOccupancy();

        publishAll(now());
        return true;
    }

    bool OctomapServerLifecycle::resetSrv(
        [[maybe_unused]] const std::shared_ptr<ResetSrv::Request> req,
        [[maybe_unused]] const std::shared_ptr<ResetSrv::Response> resp)
    {
        const auto rostime = now();
        octree_->clear();

        gridmap_.data.clear();
        gridmap_.info.height = 0.0;
        gridmap_.info.width = 0.0;
        gridmap_.info.resolution = 0.0;
        gridmap_.info.origin.position.x = 0.0;
        gridmap_.info.origin.position.y = 0.0;

        RCLCPP_INFO(get_logger(), "Cleared octomap");
        publishAll(rostime);

        return true;
    }

    bool OctomapServerLifecycle::openFile(const std::string &filename)
    {
        if (filename.length() <= 3)
        {
            return false;
        }

        std::string suffix = filename.substr(filename.length() - 3, 3);
        if (suffix == ".bt")
        {
            if (!octree_->readBinary(filename))
            {
                return false;
            }
        }
        else if (suffix == ".ot")
        {
            std::unique_ptr<octomap::AbstractOcTree> tree{octomap::AbstractOcTree::read(filename)};
            if (!tree)
            {
                return false;
            }
            octree_ = std::unique_ptr<OcTreeT>(dynamic_cast<OcTreeT *>(tree.release()));
            if (!octree_)
            {
                RCLCPP_ERROR(get_logger(), "Could not read OcTree in file");
                return false;
            }
        }
        else
        {
            return false;
        }

        RCLCPP_INFO(get_logger(), "Octomap file %s loaded (%zu nodes).", filename.c_str(), octree_->size());

        tree_depth_ = octree_->getTreeDepth();
        max_tree_depth_ = tree_depth_;
        res_ = octree_->getResolution();
        gridmap_.info.resolution = res_;

        double min_x, min_y, min_z, max_x, max_y, max_z;
        octree_->getMetricMin(min_x, min_y, min_z);
        octree_->getMetricMax(max_x, max_y, max_z);

        update_bbox_min_[0] = octree_->coordToKey(min_x);
        update_bbox_min_[1] = octree_->coordToKey(min_y);
        update_bbox_min_[2] = octree_->coordToKey(min_z);

        update_bbox_max_[0] = octree_->coordToKey(max_x);
        update_bbox_max_[1] = octree_->coordToKey(max_y);
        update_bbox_max_[2] = octree_->coordToKey(max_z);

        return true;
    }

    void OctomapServerLifecycle::filterGroundPlane(
        const PCLPointCloud &pc, PCLPointCloud &ground,
        PCLPointCloud &nonground) const
    {
        ground.header = pc.header;
        nonground.header = pc.header;

        if (pc.size() < 50)
        {
            RCLCPP_WARN(get_logger(), "Pointcloud too small, skipping ground plane extraction");
            nonground = pc;
            return;
        }

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

        pcl::SACSegmentation<PCLPoint> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(200);
        seg.setDistanceThreshold(ground_filter_distance_);
        seg.setAxis(Eigen::Vector3f(0, 0, 1));
        seg.setEpsAngle(ground_filter_angle_);

        PCLPointCloud cloud_filtered(pc);
        pcl::ExtractIndices<PCLPoint> extract;
        bool ground_plane_found = false;

        while (cloud_filtered.size() > 10 && !ground_plane_found)
        {
            seg.setInputCloud(cloud_filtered.makeShared());
            seg.segment(*inliers, *coefficients);

            if (inliers->indices.size() == 0)
            {
                break;
            }

            extract.setInputCloud(cloud_filtered.makeShared());
            extract.setIndices(inliers);

            if (std::abs(coefficients->values.at(3)) < ground_filter_plane_distance_)
            {
                extract.setNegative(false);
                extract.filter(ground);

                if (inliers->indices.size() != cloud_filtered.size())
                {
                    extract.setNegative(true);
                    PCLPointCloud cloud_out;
                    extract.filter(cloud_out);
                    nonground += cloud_out;
                    cloud_filtered = cloud_out;
                }
                ground_plane_found = true;
            }
            else
            {
                pcl::PointCloud<PCLPoint> cloud_out;
                extract.setNegative(false);
                extract.filter(cloud_out);
                nonground += cloud_out;

                if (inliers->indices.size() != cloud_filtered.size())
                {
                    extract.setNegative(true);
                    cloud_out.points.clear();
                    extract.filter(cloud_out);
                    cloud_filtered = cloud_out;
                }
                else
                {
                    cloud_filtered.points.clear();
                }
            }
        }

        if (!ground_plane_found)
        {
            RCLCPP_WARN(get_logger(), "No ground plane found in scan");
            pcl::PassThrough<PCLPoint> second_pass;
            second_pass.setFilterFieldName("z");
            second_pass.setFilterLimits(-ground_filter_plane_distance_, ground_filter_plane_distance_);
            second_pass.setInputCloud(pc.makeShared());
            second_pass.filter(ground);
            second_pass.setNegative(true);
            second_pass.filter(nonground);
        }
    }

    // Implement remaining virtual hook methods
    void OctomapServerLifecycle::handlePreNodeTraversal(const rclcpp::Time &rostime)
    {
        if (!publish_2d_map_)
        {
            return;
        }

        gridmap_.header.frame_id = world_frame_id_;
        gridmap_.header.stamp = rostime;
        MapMetaData old_map_info = gridmap_.info;

        double min_x, min_y, min_z, max_x, max_y, max_z;
        octree_->getMetricMin(min_x, min_y, min_z);
        octree_->getMetricMax(max_x, max_y, max_z);

        octomap::point3d min_pt(min_x, min_y, min_z);
        octomap::point3d max_pt(max_x, max_y, max_z);

        double half_padded_x = 0.5 * min_x_size_;
        double half_padded_y = 0.5 * min_y_size_;
        min_x = std::min(min_x, -half_padded_x);
        max_x = std::max(max_x, half_padded_x);
        min_y = std::min(min_y, -half_padded_y);
        max_y = std::max(max_y, half_padded_y);
        min_pt = octomap::point3d(min_x, min_y, min_z);
        max_pt = octomap::point3d(max_x, max_y, max_z);

        octomap::OcTreeKey padded_max_key;
        if (!octree_->coordToKeyChecked(min_pt, max_tree_depth_, padded_min_key_) ||
            !octree_->coordToKeyChecked(max_pt, max_tree_depth_, padded_max_key))
        {
            RCLCPP_ERROR(get_logger(), "Could not create padded OcTree keys");
            return;
        }

        multires_2d_scale_ = 1 << (tree_depth_ - max_tree_depth_);
        gridmap_.info.width = (padded_max_key[0] - padded_min_key_[0]) / multires_2d_scale_ + 1;
        gridmap_.info.height = (padded_max_key[1] - padded_min_key_[1]) / multires_2d_scale_ + 1;

        octomap::point3d origin = octree_->keyToCoord(padded_min_key_, tree_depth_);
        double grid_res = octree_->getNodeSize(max_tree_depth_);
        project_complete_map_ = (!incremental_2D_projection_ || (std::abs(grid_res - gridmap_.info.resolution) > 1e-6));
        gridmap_.info.resolution = grid_res;
        gridmap_.info.origin.position.x = origin.x() - grid_res * 0.5;
        gridmap_.info.origin.position.y = origin.y() - grid_res * 0.5;

        if (project_complete_map_)
        {
            gridmap_.data.clear();
            gridmap_.data.resize(gridmap_.info.width * gridmap_.info.height, -1);
        }
        else if (mapChanged(old_map_info, gridmap_.info))
        {
            adjustMapData(gridmap_, old_map_info);
        }
    }

    void OctomapServerLifecycle::handlePostNodeTraversal(const rclcpp::Time &rostime)
    {
        if (publish_2d_map_)
        {
            map_pub_->publish(gridmap_);
        }
    }

    void OctomapServerLifecycle::handleOccupiedNode(const OcTreeT::iterator &it)
    {
        if (publish_2d_map_ && project_complete_map_)
        {
            update2DMap(it, true);
        }
    }

    void OctomapServerLifecycle::handleFreeNode(const OcTreeT::iterator &it)
    {
        if (publish_2d_map_ && project_complete_map_)
        {
            update2DMap(it, false);
        }
    }

    void OctomapServerLifecycle::handleOccupiedNodeInBBX(const OcTreeT::iterator &it)
    {
        if (publish_2d_map_ && !project_complete_map_)
        {
            update2DMap(it, true);
        }
    }

    void OctomapServerLifecycle::handleFreeNodeInBBX(const OcTreeT::iterator &it)
    {
        if (publish_2d_map_ && !project_complete_map_)
        {
            update2DMap(it, false);
        }
    }

    void OctomapServerLifecycle::update2DMap(const OcTreeT::iterator &it, bool occupied)
    {
        if (it.getDepth() == max_tree_depth_)
        {
            unsigned idx = mapIdx(it.getKey());
            if (occupied)
            {
                gridmap_.data[idx] = 100;
            }
            else if (gridmap_.data[idx] == -1)
            {
                gridmap_.data[idx] = 0;
            }
        }
        else
        {
            int int_size = 1 << (max_tree_depth_ - it.getDepth());
            octomap::OcTreeKey min_key = it.getIndexKey();
            for (int dx = 0; dx < int_size; dx++)
            {
                int i = (min_key[0] + dx - padded_min_key_[0]) / multires_2d_scale_;
                for (int dy = 0; dy < int_size; dy++)
                {
                    unsigned idx = mapIdx(i, (min_key[1] + dy - padded_min_key_[1]) / multires_2d_scale_);
                    if (occupied)
                    {
                        gridmap_.data[idx] = 100;
                    }
                    else if (gridmap_.data[idx] == -1)
                    {
                        gridmap_.data[idx] = 0;
                    }
                }
            }
        }
    }

    bool OctomapServerLifecycle::isSpeckleNode(const octomap::OcTreeKey &n_key) const
    {
        octomap::OcTreeKey key;
        bool neighbor_found = false;
        for (key[2] = n_key[2] - 1; !neighbor_found && key[2] <= n_key[2] + 1; ++key[2])
        {
            for (key[1] = n_key[1] - 1; !neighbor_found && key[1] <= n_key[1] + 1; ++key[1])
            {
                for (key[0] = n_key[0] - 1; !neighbor_found && key[0] <= n_key[0] + 1; ++key[0])
                {
                    if (key != n_key)
                    {
                        octomap::OcTreeNode *node = octree_->search(key);
                        if (node && octree_->isNodeOccupied(node))
                        {
                            neighbor_found = true;
                        }
                    }
                }
            }
        }
        return neighbor_found;
    }

    void OctomapServerLifecycle::adjustMapData(
        OccupancyGrid &map, const MapMetaData &old_map_info) const
    {
        if (map.info.resolution != old_map_info.resolution)
        {
            RCLCPP_ERROR(get_logger(), "Resolution of map changed, cannot be adjusted");
            return;
        }

        int i_off = static_cast<int>((old_map_info.origin.position.x - map.info.origin.position.x) /
                                         map.info.resolution +
                                     0.5);
        int j_off = static_cast<int>((old_map_info.origin.position.y - map.info.origin.position.y) /
                                         map.info.resolution +
                                     0.5);

        if (i_off < 0 || j_off < 0 ||
            old_map_info.width + i_off > map.info.width ||
            old_map_info.height + j_off > map.info.height)
        {
            RCLCPP_ERROR(get_logger(), "New 2D map does not contain old map area");
            return;
        }

        OccupancyGrid::_data_type old_map_data = map.data;
        map.data.clear();
        map.data.resize(map.info.width * map.info.height, -1);

        for (size_t j = 0; j < old_map_info.height; ++j)
        {
            auto from_start = old_map_data.begin() + j * old_map_info.width;
            auto from_end = from_start + old_map_info.width;
            auto to_start = map.data.begin() + ((j + j_off) * gridmap_.info.width + i_off);
            std::copy(from_start, from_end, to_start);
        }
    }

    ColorRGBA OctomapServerLifecycle::heightMapColor(double h)
    {
        ColorRGBA color;
        color.a = 1.0;

        double s = 1.0;
        double v = 1.0;

        h -= floor(h);
        h *= 6;
        int i = floor(h);
        double f = h - i;
        if (!(i & 1))
        {
            f = 1 - f;
        }
        double m = v * (1 - s);
        double n = v * (1 - s * f);

        switch (i)
        {
        case 6:
        case 0:
            color.r = v;
            color.g = n;
            color.b = m;
            break;
        case 1:
            color.r = n;
            color.g = v;
            color.b = m;
            break;
        case 2:
            color.r = m;
            color.g = v;
            color.b = n;
            break;
        case 3:
            color.r = m;
            color.g = n;
            color.b = v;
            break;
        case 4:
            color.r = n;
            color.g = m;
            color.b = v;
            break;
        case 5:
            color.r = v;
            color.g = m;
            color.b = n;
            break;
        default:
            color.r = 1;
            color.g = 0.5;
            color.b = 0.5;
            break;
        }

        return color;
    }

    rcl_interfaces::msg::SetParametersResult OctomapServerLifecycle::onParameter(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        int64_t max_tree_depth{get_parameter("max_depth").as_int()};
        update_param(parameters, "max_depth", max_tree_depth);
        max_tree_depth_ = static_cast<size_t>(max_tree_depth);
        update_param(parameters, "point_cloud_min_z", point_cloud_min_z_);
        update_param(parameters, "point_cloud_max_z", point_cloud_max_z_);
        update_param(parameters, "occupancy_min_z", occupancy_min_z_);
        update_param(parameters, "occupancy_max_z", occupancy_max_z_);
        update_param(parameters, "filter_speckles", filter_speckles_);
        update_param(parameters, "filter_ground_plane", filter_ground_plane_);
        update_param(parameters, "compress_map", compress_map_);
        update_param(parameters, "incremental_2D_projection", incremental_2D_projection_);
        update_param(parameters, "ground_filter_distance", ground_filter_distance_);
        update_param(parameters, "ground_filter_angle", ground_filter_angle_);
        update_param(parameters, "ground_filter_plane_distance", ground_filter_plane_distance_);
        update_param(parameters, "sensor_model.max_range", max_range_);

        double sensor_model_min{get_parameter("sensor_model.min").as_double()};
        update_param(parameters, "sensor_model.min", sensor_model_min);
        octree_->setClampingThresMin(sensor_model_min);

        double sensor_model_max{get_parameter("sensor_model.max").as_double()};
        update_param(parameters, "sensor_model.max", sensor_model_max);
        octree_->setClampingThresMax(sensor_model_max);

        double sensor_model_hit{get_parameter("sensor_model.hit").as_double()};
        update_param(parameters, "sensor_model.hit", sensor_model_hit);
        octree_->setProbHit(sensor_model_hit);

        double sensor_model_miss{get_parameter("sensor_model.miss").as_double()};
        update_param(parameters, "sensor_model.miss", sensor_model_miss);
        octree_->setProbMiss(sensor_model_miss);

        publishAll(now());

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        return result;
    }

} // namespace octomap_server

#include <rclcpp_components/register_node_macro.hpp>

// Register lifecycle node
RCLCPP_COMPONENTS_REGISTER_NODE(octomap_server::OctomapServerLifecycle)