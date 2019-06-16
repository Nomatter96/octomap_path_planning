#pragma once

#include <octomap/octomap.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/PathSimplifier.h>


#include <ompl/config.h>
#include <iostream>

#include "fcl/config.h"
#include "fcl/geometry/octree/octree.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/broadphase/broadphase_bruteforce.h"
#include "fcl/broadphase/broadphase_spatialhash.h"
#include "fcl/broadphase/broadphase_SaP.h"
#include "fcl/broadphase/broadphase_SSaP.h"
#include "fcl/broadphase/broadphase_interval_tree.h"
#include "fcl/broadphase/broadphase_dynamic_AABB_tree.h"
#include "fcl/broadphase/broadphase_dynamic_AABB_tree_array.h"
#include "fcl/geometry/geometric_shape_to_BVH_model.h"
#include "fcl/math/bv/utility.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/detail/gjk_solver_indep.h"
#include "fcl/narrowphase/detail/gjk_solver_libccd.h"
#include "fcl/narrowphase/detail/traversal/collision_node.h"

class path_planer
{
public:
    path_planer();
    ~path_planer() = default;

    void set_goal(double x, double y, double z) const;

    void set_start(double x, double y, double z) const;

    void update_octomap(const octomap::OcTree& map);

    void plan();

    void replan();

    std::vector<std::tuple<double, double, double>> get_smooth_path() const;

private:

    bool is_state_valid(const ompl::base::State* state) const;
    static ompl::base::OptimizationObjectivePtr get_path_length_with_cost(const ompl::base::SpaceInformationPtr& information_ptr);

    ompl::base::StateSpacePtr state_space_;
    ompl::base::SpaceInformationPtr space_information_;
    ompl::base::ProblemDefinitionPtr problem_definition_ptr_;
    ompl::base::PlannerPtr planner_ptr_;
    ompl::geometric::PathGeometric* smooth_path_ = nullptr;

    std::shared_ptr<fcl::CollisionObject<double>> tree_object_;
    std::shared_ptr<fcl::CollisionObject<double>> water_craft_object_;

    bool replan_flag_ = true;

};

