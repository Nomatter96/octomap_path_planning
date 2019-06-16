#include "application.hpp"

void application::start()
{
    const auto octomap_callback = std::bind(&octomap_wrapper::add_depth_map, &octomap_, std::placeholders::_1, std::placeholders::_2);
    zed_camera_.add_frame_callback(octomap_callback);
    const auto path_planner_callback = std::bind(&path_planer::update_octomap, &path_planer_, std::placeholders::_1);
    octomap_.oct_tree_update_callback(path_planner_callback);
    zed_camera_.start();
    bool f;
    while (true) {
        if (octomap_.is_valid()) {
            std::vector<double> cam_pose = zed_camera_.get_pose();
            path_planer_.set_start(cam_pose[0], cam_pose[1], cam_pose[2]);
            path_planer_.set_goal(cam_pose[0], cam_pose[1], cam_pose[2] + 0.5);
            
            path_planer_.plan();
            auto path = path_planer_.get_smooth_path();

            if (f)
                continue;
            f = true;
            octomap::OcTreeKey key;
            octomap::KeySet keys;
            auto oct_tree = octomap_.get_oct_tree();
            for (auto& it : path) {
                octomap::point3d point(std::get<0>(it), std::get<1>(it), std::get<2>(it));
                oct_tree.coordToKeyChecked(point, key);
                keys.insert(key);
            }
            for (auto& it : keys) {
                oct_tree.updateNode(it, true);
            }
            oct_tree.prune();
            oct_tree.write("test.ot");
        }
    }
}

void application::stop()
{

}
