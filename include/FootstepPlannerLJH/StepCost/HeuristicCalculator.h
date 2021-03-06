
#pragma once
#include <cmath>
#include <FootstepPlannerLJH/Title.h>
#include <FootstepPlannerLJH/DataType.h>
#include <FootstepPlannerLJH/parameters.h>
#include <Heuclid/geometry/Pose3D.h>
#include <Heuclid/geometry/Pose2D.h>
#include <FootstepPlannerLJH/SimpleBodyPathPlanner/simple2DBodyPathHolder.h>
using ljh::mathlib::Pose3D;
_FOOTSTEP_PLANNER_BEGIN


class HeuristicCalculator
{
private:
    cost_t heuristicCost;
    Pose3D<double> goalPose;
    Pose3D<double> startPose;

    double desireHeading;
    parameters param;

    //Compute param
    Pose2D<double> midFootPose;
    double initialTurnDistance;
    double walkDistance;
    double finalTurnDistance;
    double pathDistance;
    PointFromPathInfo pfp;
    
public:
    Simple2DBodyPathHolder pathHolder;
    
    HeuristicCalculator():heuristicCost(cost_t(0)),goalPose(),startPose(),desireHeading(0.0),param(),midFootPose(),initialTurnDistance(0.0),walkDistance(0.0),finalTurnDistance(0.0),pathDistance(0.0),pathHolder(){};
    HeuristicCalculator(Pose3D<double> _goalPose, Pose3D<double> _startPose):heuristicCost(cost_t(0)),goalPose(_goalPose),startPose(_startPose),desireHeading(0.0),param(),midFootPose(),initialTurnDistance(0.0),walkDistance(0.0),finalTurnDistance(0.0),pathDistance(0.0),pathHolder(){};

    void initialize(Pose3D<double> _goalPose, Pose3D<double> _startPose);
    cost_t compute(FootstepGraphNode& node);
    cost_t computeFollowEllipsoidPath(FootstepGraphNode& node);
};




_FOOTSTEP_PLANNER_END