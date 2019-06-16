#pragma once

#include "zed_camera.hpp"
#include "octomap_wrapper.hpp"
#include "path_planer.hpp"

class application {
public:
    application() = default;
    ~application() = default;

    void start();

    void stop();

private:
    zed_camera zed_camera_;
    octomap_wrapper octomap_;
    path_planer path_planer_;

};