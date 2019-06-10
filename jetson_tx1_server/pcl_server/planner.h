#pragma once

#include <vector>

#include <octomap/octomap.h>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/PathSimplifier.h>

#include <ompl/config.h>
#include <iostream>

#include <fcl/config.h>
#include <fcl/fcl.h>
#include <fcl/geometry/collision_geometry.h>
#include <fcl/geometry/octree/octree.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace server {

class Planner
{
public:
    Planner();
    bool SetStart(double aX, double aY, double aZ);
    bool SetGoal(double aX, double aY, double aZ);
    void UpdateMap(octomap::OcTree aTreeOct);
    void Plan();
    bool Replan();
    std::vector<std::tuple<double, double, double>> GetSmoothPath();

private:
    ob::StateSpacePtr mSpace;
    ob::SpaceInformationPtr mSi;
    ob::ProblemDefinitionPtr mPdef;
    ob::PlannerPtr mOPlan;
    og::PathGeometric* mPathSmooth = NULL;
    bool mReplanFlag = true;
    std::shared_ptr<fcl::CollisionObject<double>> mTreeObj;
    std::shared_ptr<fcl::CollisionObject<double>> mWaterCraftObject;

    bool IsStateValid(const ob::State *aState);
    ob::OptimizationObjectivePtr GetPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& aSi);
};

}
