#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sl_zed/Camera.hpp>

#include <functional>
#include <thread>
#include <atomic>

class zed_camera {
public:
    zed_camera(zed_camera &&) = delete;
    zed_camera(zed_camera&) = delete;

    zed_camera& operator=(zed_camera& other) = delete;
    zed_camera& operator=(zed_camera&& other) = delete;

    zed_camera();
    ~zed_camera();

    void start();

    void stop();

    void add_frame_callback(std::function<void(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, sl::Pose)> callback);

    std::vector<double> get_pose();

private:
    void setup();
    static inline float sl_cvt_color(float color);

    void update_camera();
    
    std::atomic<bool> thread_flag_;
    std::thread camera_thread_;
    std::function<void(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, sl::Pose)> frame_complete_callback_;
    sl::Camera zed_camera_;
};
