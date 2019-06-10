#include "planner.h"

namespace server {

Planner::Planner()
{
    mWaterCraftObject = std::make_shared<fcl::CollisionObject<double>>(std::shared_ptr<fcl::CollisionGeometry<double>>(new fcl::Box<double>(1.5, 1.5, 1.5)));

    mSpace = ob::StateSpacePtr(new ob::RealVectorStateSpace(3));
    ob::ScopedState<ob::RealVectorStateSpace> start(mSpace);
    ob::ScopedState<ob::RealVectorStateSpace> goal(mSpace);
    ob::RealVectorBounds bounds(3);
    
    bounds.setLow(0,-10);
	bounds.setHigh(0,10);
	bounds.setLow(1,-10);
	bounds.setHigh(1,10);
	bounds.setLow(2, -10);
	bounds.setHigh(2, 10);

    bounds.check();

    mSpace->as<ob::RealVectorStateSpace>()->setBounds(bounds);
    mSi = ob::SpaceInformationPtr(new ob::SpaceInformation(mSpace));

	start->values[0] = 0;
	start->values[1] = 0;
	start->values[2] = 0;

	goal->values[0] = 0;
	goal->values[1] = 0;
	goal->values[2] = 0;

    mSi->setStateValidityChecker(std::bind(&Planner::IsStateValid, this, std::placeholders::_1 ));
    mPdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(mSi));
    mPdef->setStartAndGoalStates(start, goal);
    mPdef->setOptimizationObjective(Planner::GetPathLengthObjWithCostToGo(mSi));
    mOPlan = ob::PlannerPtr(new og::InformedRRTstar(mSi));
    mOPlan->setProblemDefinition(mPdef);
    mOPlan->setup();
}

bool
Planner::SetStart(double aX, double aY, double aZ)
{
    std::cout << aX << " " << aY << " " << aZ << "\n";
    ob::ScopedState<ob::RealVectorStateSpace> start(mSpace);
    start->values[0] = aX;
    start->values[1] = aY;
    start->values[2] = aZ;
    ob::State *state = mSpace->allocState();
    state->as<ob::RealVectorStateSpace::StateType>()->values = start->values;
    mPdef->clearStartStates();
    mPdef->addStartState(start);
    if (IsStateValid(state)) {
        mPdef->clearStartStates();
        mPdef->addStartState(start);
        return true;
    } else {
        return false;
    }
}

bool
Planner::SetGoal(double aX, double aY, double aZ)
{
    ob::ScopedState<ob::RealVectorStateSpace> goal(mSpace);
    goal->values[0] = aX;
    goal->values[1] = aY;
    goal->values[2] = aZ;
    mPdef->clearGoal();
    mPdef->setGoalState(goal);
    ob::State *state = mSpace->allocState();
    state->as<ob::RealVectorStateSpace::StateType>()->values = goal->values;
    if (IsStateValid(state)) {
        return true;
    } else {
        return false;
    }
}

void
Planner::UpdateMap(octomap::OcTree aTreeOct)
{
    fcl::OcTree<double>* tree = new fcl::OcTree<double>(std::make_shared<const octomap::OcTree>(aTreeOct));
    std::shared_ptr<fcl::CollisionGeometry<double>> treeObj = std::shared_ptr<fcl::CollisionGeometry<double>>(tree);
    mTreeObj = std::make_shared<fcl::CollisionObject<double>>(treeObj);
}

bool
Planner::Replan()
{
    if (mPathSmooth != NULL) {
        og::PathGeometric* path = mPdef->getSolutionPath()->as<og::PathGeometric>();
        std::cout << path->getStateCount() << "\n";
        double distance;
        if (mPdef->hasApproximateSolution()) {
            mReplanFlag = true;
        } else {
            for (std::size_t idx = 0; idx < path->getStateCount(); idx++) {
                if (!mReplanFlag) {
                    mReplanFlag = !IsStateValid(path->getState(idx));
                } else {
                    break;
                }
            }
        }
    }
    if (mReplanFlag) {
        mPdef->clearSolutionPaths();
        Plan();
        return true;
    } else {
        return false;
    }
}

void
Planner::Plan()
{
    mOPlan->setProblemDefinition(mPdef);
    mOPlan->setup();
    mSi->printSettings(std::cout);
    std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << "\n";
    mPdef->print(std::cout);
    ob::PlannerStatus solved = mOPlan->solve(4);
    if (solved) {
        ob::PathPtr path = mPdef->getSolutionPath();
        og::PathGeometric* pth = mPdef->getSolutionPath()->as<og::PathGeometric>();
        pth->printAsMatrix(std::cout);

        og::PathSimplifier* pathBSpline = new og::PathSimplifier(mSi);
        mPathSmooth = new og::PathGeometric(dynamic_cast<const og::PathGeometric&>(*mPdef->getSolutionPath()));
        pathBSpline->smoothBSpline(*mPathSmooth);
        mReplanFlag = false;
    }
}

bool
Planner::IsStateValid(const ob::State *aState)
{
    const ob::RealVectorStateSpace::StateType *pos = aState->as<ob::RealVectorStateSpace::StateType>();
    fcl::Vector3<double> translation(pos->values[0], pos->values[1], pos->values[2]);
    mWaterCraftObject->setTranslation(translation);
    fcl::CollisionRequest<double> requestType(1, false, 1, false);
    fcl::CollisionResult<double> collisionResult;
    fcl::collide(mWaterCraftObject.get(), mTreeObj.get(), requestType, collisionResult);
    return (!collisionResult.isCollision());

}

ob::OptimizationObjectivePtr
Planner::GetPathLengthObjWithCostToGo(const ob::SpaceInformationPtr& aSi)
{
    ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(aSi));
    aSi->printSettings(std::cout);
    return obj;
}

std::vector<std::tuple<double, double, double>>
Planner::GetSmoothPath()
{
    std::vector<std::tuple<double, double, double>> path;
    for (std::size_t idx = 0; idx < mPathSmooth->getStateCount(); idx++) {
        const ob::RealVectorStateSpace::StateType *pos = mPathSmooth->getState(idx)->as<ob::RealVectorStateSpace::StateType>();
        path.push_back(std::tuple<double, double, double>(pos->values[0], pos->values[1], pos->values[2]));
    }
    return path;
}

}
