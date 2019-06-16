#include "zed_camera.hpp"

#include <pcl/filters/voxel_grid.h>

#include <utility>

zed_camera::zed_camera() : thread_flag_(true) { 
}

zed_camera::~zed_camera()
{
    stop();
}

void zed_camera::start()
{
    setup();
    camera_thread_ = std::thread(&zed_camera::update_camera, this);
}

void zed_camera::stop()
{
    thread_flag_ = false;
    camera_thread_.join();
    zed_camera_.close();
}

void zed_camera::add_frame_callback(std::function<void(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, sl::Pose)> callback)
{
    frame_complete_callback_ = std::move(callback);
}

std::vector<double> zed_camera::get_pose()
{
    sl::Pose camera_pose;
    zed_camera_.getPosition(camera_pose, sl::REFERENCE_FRAME_WORLD);

    std::vector<double> pose;
    for (int i = 0; i < 3; i++) {
        pose.push_back(camera_pose.pose_data(i, 0));
    }
    return pose;
}

void zed_camera::setup()
{
    sl::InitParameters init_params;
    init_params.camera_resolution = sl::RESOLUTION_VGA;
    init_params.camera_fps = 30;
    init_params.coordinate_units = sl::UNIT_METER;
    init_params.coordinate_system = sl::COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP;
    init_params.depth_mode = sl::DEPTH_MODE_MEDIUM;

    auto err = zed_camera_.open(init_params);
    if (err != sl::SUCCESS) {
        zed_camera_.close();
        throw std::runtime_error("Unable to open ZED camera.");
    }

    sl::TrackingParameters tracking_params;
    tracking_params.initial_world_transform = sl::Transform::identity();
    tracking_params.enable_spatial_memory = true;

    err = zed_camera_.enableTracking(tracking_params);
    if (err != sl::SUCCESS) {
        zed_camera_.close();
        throw std::runtime_error("Unable to setup ZED tracking params.");
    }
}

float zed_camera::sl_cvt_color(float color)
{
    uint32_t color_uint = *reinterpret_cast<uint32_t*>(&color);
    auto* color_uchar = reinterpret_cast<unsigned char*>(&color_uint);
    color_uint = (static_cast<uint32_t>(color_uchar[0]) << 16 |
        static_cast<uint32_t>(color_uchar[1]) << 8 | static_cast<uint32_t>(color_uchar[2]));
    return *reinterpret_cast<float*> (&color_uint);
}

void zed_camera::update_camera()
{
    sl::Mat depth_map;
    sl::Pose camera_pose;
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
    voxel_grid.setLeafSize(0.1f, 0.1f, 0.1f);
    while (thread_flag_) {

        if (zed_camera_.grab(sl::SENSING_MODE_STANDARD) == sl::SUCCESS) {
            zed_camera_.retrieveMeasure(depth_map, sl::MEASURE_XYZRGBA);
            zed_camera_.getPosition(camera_pose, sl::REFERENCE_FRAME_WORLD);

            if (frame_complete_callback_ == nullptr) {
                continue;
            }
        }
        else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl_cloud->resize(zed_camera_.getResolution().area());

        const auto cloud_data = depth_map.getPtr<float>();
        std::size_t idx = 0;

        for (auto& it : pcl_cloud->points)
        {
            if (!std::isfinite(cloud_data[idx]))
            {
                it.x = it.y = it.z = it.rgb = 0;
            }
            else
            {
                it.x = cloud_data[idx];
                it.y = cloud_data[idx + 1];
                it.z = cloud_data[idx + 2];
                it.rgb = sl_cvt_color(cloud_data[idx + 3]);
            }
            idx += 4;
        }
        voxel_grid.setInputCloud(pcl_cloud);
        voxel_grid.filter(*pcl_cloud);

        frame_complete_callback_(pcl_cloud, camera_pose);
    }
}

