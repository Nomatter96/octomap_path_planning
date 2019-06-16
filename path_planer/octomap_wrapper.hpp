#pragma once

#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>
#include <octomap/octomap_types.h>
#include <octomap/OcTreeKey.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sl_zed/Camera.hpp>

class octomap_wrapper
{
public:
    octomap_wrapper();
    ~octomap_wrapper() = default;

    void add_depth_map(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, sl::Pose pose);

    void oct_tree_update_callback(std::function<void(octomap::OcTree&)> callback);

    bool is_valid() const;

    octomap::OcTree get_oct_tree();

private:

    void update_keys(const octomap::OcTreeKey& end_key);
    
    octomap::OcTree oct_tree_;
    octomap::KeyRay oct_ray_key_;
    octomap::OcTreeKey bounding_box_min_;
    octomap::OcTreeKey bounding_box_max_;
    bool is_valid_ = false;
    std::function<void(octomap::OcTree&)> oct_tree_update_callback_;
};
