#include  "path_planer.hpp"
#include "fcl/math/constants.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/narrowphase/collision_object.h"
#include <ompl/base/Path.h>

path_planer::path_planer()
{
    water_craft_object_ = std::make_shared<fcl::CollisionObject<double>>(std::make_shared<fcl::Box<double>>(1.5, 1.5, 1.5));
    state_space_ = (std::make_shared<ompl::base::RealVectorStateSpace>(3));

    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> start(state_space_);
    ompl::base::ScopedState<ompl::base::RealVectorStateSpace> goal(state_space_);

    ompl::base::RealVectorBounds bounds(3);

    bounds.setLow(0, -10);
    bounds.setHigh(0, 10);
    bounds.setLow(1, -10);
    bounds.setHigh(1, 10);
    bounds.setLow(2, -10);
    bounds.setHigh(2, 10);

    bounds.check();

    state_space_->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
    space_information_ = std::make_shared<ompl::base::SpaceInformation>(state_space_);

    start->values[0] = 0;
    start->values[1] = 0;
    start->values[2] = 0;

    goal->values[0] = 0;
    goal->values[1] = 0;
    goal->values[2] = 0;

    space_information_->setStateValidityChecker(std::bind(&path_planer::is_state_valid, this, std::placeholders::_1));
    problem_definition_ptr_ = std::make_shared<ompl::base::ProblemDefinition>(space_information_);
    problem_definition_ptr_->setStartAndGoalStates(start, goal);

    problem_definition_ptr_->setOptimizationObjective(path_planer::get_path_length_with_cost(space_information_));
    planner_ptr_ = ompl::base::PlannerPtr(new ompl::geometric::InformedRRTstar(space_information_));
    planner_ptr_->setProblemDefinition(problem_definition_ptr_);
    planner_ptr_->setup();

    std::cout << "Inited" << std::endl;
}

void path_planer::set_goal(const double x, const double y, const double z) const
{
    ompl::base::ScopedState <ompl::base::RealVectorStateSpace> goal(state_space_);
    goal->values[0] = x;
    goal->values[1] = y;
    goal->values[2] = z;

    auto state = state_space_->allocState();
    state->as<ompl::base::RealVectorStateSpace::StateType>()->values = goal->values;

    if (is_state_valid(state))
    {
        problem_definition_ptr_->clearGoal();
        problem_definition_ptr_->setGoalState(goal);
    }
    else
    {
        std::cerr << "Unable to set goal state" << std::endl;
    }
}

void path_planer::set_start(const double x, const double y, const double z) const
{
    ompl::base::ScopedState <ompl::base::RealVectorStateSpace> start(state_space_);
    start->values[0] = x;
    start->values[1] = y;
    start->values[2] = z;

    auto state = state_space_->allocState();
    state->as<ompl::base::RealVectorStateSpace::StateType>()->values = start->values;

    if (is_state_valid(state))
    {
        problem_definition_ptr_->clearStartStates();
        problem_definition_ptr_->addStartState(start);
    }
    else
    {
        std::cerr << "Unable to set start state" << std::endl;
    }

}

void path_planer::update_octomap(const octomap::OcTree& map)
{
    const auto tree = new fcl::OcTree<double>(std::make_shared<const octomap::OcTree>(map));
    auto tree_obj = std::shared_ptr<fcl::CollisionGeometry<double>>(tree);
    tree_object_ = std::make_shared<fcl::CollisionObject<double>>(tree_obj);
}

void path_planer::plan()
{
    planner_ptr_->setProblemDefinition(problem_definition_ptr_);
    planner_ptr_->setup();
    space_information_->printSettings(std::cout);
    const auto solved = planner_ptr_->solve(4);

    if (solved)
    {
        auto path = problem_definition_ptr_->getSolutionPath();
        const auto geometric_path = problem_definition_ptr_->getSolutionPath()->as<ompl::geometric::PathGeometric>();
        geometric_path->printAsMatrix(std::cout);

        auto path_spline = std::make_shared<ompl::geometric::PathSimplifier>(space_information_);
        smooth_path_ = new ompl::geometric::PathGeometric(dynamic_cast<const ompl::geometric::PathGeometric&>(*problem_definition_ptr_->getSolutionPath()));
        path_spline->smoothBSpline(*smooth_path_);

    }

}

std::vector<std::tuple<double, double, double>> path_planer::get_smooth_path() const
{
    std::vector<std::tuple<double, double, double>> path;
    for (std::size_t idx = 0; idx < smooth_path_->getStateCount(); idx++) {
        const ompl::base::RealVectorStateSpace::StateType* pos = smooth_path_->getState(idx)->as<ompl::base::RealVectorStateSpace::StateType>();
        path.emplace_back(pos->values[0], pos->values[1], pos->values[2]);
    }
    return path;
}

inline bool path_planer::is_state_valid(const ompl::base::State* state) const
{
    const auto pos = state->as<ompl::base::RealVectorStateSpace::StateType>();
    const fcl::Vector3<double> translation(pos->values[0], pos->values[1], pos->values[2]);
    water_craft_object_->setTranslation(translation);
    const fcl::CollisionRequest<double> request_type(1, false, 1, false);
    fcl::CollisionResult<double> collision_result;
    fcl::collide(water_craft_object_.get(), tree_object_.get(), request_type, collision_result);
    return (!collision_result.isCollision());
}

ompl::base::OptimizationObjectivePtr path_planer::get_path_length_with_cost(
    const ompl::base::SpaceInformationPtr& information_ptr)
{
    ompl::base::OptimizationObjectivePtr obj(new ompl::base::PathLengthOptimizationObjective(information_ptr));
    information_ptr->printSettings(std::cout);
    return obj;
}

void path_planer::replan()
{
    if (smooth_path_ == nullptr)
    {
        auto path = problem_definition_ptr_->getSolutionPath()->as<ompl::geometric::PathGeometric>();
        auto distance = 0.0;

        if (problem_definition_ptr_->hasApproximateSolution())
        {
            replan_flag_ = true;
        }
        else
        {
            for (std::size_t idx = 0; idx < path->getStateCount(); idx++) {
                if (!replan_flag_) {
                    replan_flag_ = !is_state_valid(path->getState(idx));
                }
                else
                {
                    break;
                }
            }
        }
    }
    if (replan_flag_) {
        problem_definition_ptr_->clearSolutionPaths();
        plan();
    }
}

