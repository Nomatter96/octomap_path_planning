#include "octomap_wrapper.hpp"

#include <pcl/common/transforms.h>

octomap_wrapper::octomap_wrapper() : oct_tree_(0.1)
{
    oct_tree_.setProbHit(0.55);
    oct_tree_.setProbMiss(0.4);
    oct_tree_.setClampingThresMin(0.12);
    oct_tree_.setClampingThresMax(0.97);
}

void octomap_wrapper::add_depth_map(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, sl::Pose pose)
{
    constexpr auto oct_max_range = -1.0;

    Eigen::Matrix4f world_pose;

    for (auto i = 0; i <= 3; ++i) {
        for (auto j = 0; j <= 3; ++j) {
            world_pose(i, j) = pose.pose_data(i, j);
        }
    }

    pcl::transformPointCloud(*cloud, *cloud, world_pose);
    const octomap::point3d origin_vector3d{ pose.pose_data(0, 3), pose.pose_data(1, 3), pose.pose_data(2, 3) };

    if (!oct_tree_.coordToKeyChecked(origin_vector3d, bounding_box_max_) || !oct_tree_.coordToKeyChecked(origin_vector3d, bounding_box_min_))
    {
        std::cerr << "Unable to generate key for origin" << std::endl;
        return;
    }

    octomap::KeySet free_cells;
    octomap::KeySet occupied_cells;

    for (auto& it : cloud->points)
    {
        octomap::point3d point3d(it.x, it.y, it.z);

        if ((oct_max_range < 0.0) || ((point3d - origin_vector3d).norm() <= oct_max_range))
        {
            if (oct_tree_.computeRayKeys(origin_vector3d, point3d, oct_ray_key_))
            {
                free_cells.insert(oct_ray_key_.begin(), oct_ray_key_.end());
            }
            octomap::OcTreeKey oct_key;
            if (oct_tree_.coordToKeyChecked(point3d, oct_key))
            {
                occupied_cells.insert(oct_key);
            }
        }
        else
        {
            auto new_end3d = origin_vector3d + (point3d - origin_vector3d).normalize() * oct_max_range;
            if (oct_tree_.computeRayKeys(origin_vector3d, new_end3d, oct_ray_key_))
            {
                free_cells.insert(oct_ray_key_.begin(), oct_ray_key_.end());
                octomap::OcTreeKey end_key;
                if (oct_tree_.coordToKeyChecked(new_end3d, end_key))
                {
                    free_cells.insert(end_key);
                    update_keys(end_key);
                }
                else
                {
                    std::cerr << "Unable to generate key for end point" << std::endl;
                }
            }
        }
    }

    for (auto& it : free_cells)
    {
        if (occupied_cells.find(it) == occupied_cells.end())
        {
            oct_tree_.updateNode(it, false);
        }
    }

    for (auto& it : occupied_cells)
    {
        oct_tree_.updateNode(it, true);
    }
    oct_tree_.prune();

    std::cout << "octomap updated" << std::endl;
    oct_tree_update_callback_(oct_tree_);
    is_valid_ = true;
}

void octomap_wrapper::oct_tree_update_callback(std::function<void(octomap::OcTree&)> callback)
{
    oct_tree_update_callback_ = std::move(callback);
}

bool octomap_wrapper::is_valid() const
{
    return is_valid_;
}

octomap::OcTree octomap_wrapper::get_oct_tree()
{
    return oct_tree_;
}

void octomap_wrapper::update_keys(const octomap::OcTreeKey& end_key)
{
    for (std::size_t i = 0; i < 3; ++i)
    {
        bounding_box_min_[i] = std::min(end_key[i], bounding_box_min_[i]);
        bounding_box_max_[i] = std::max(end_key[i], bounding_box_max_[i]);
    }
}
