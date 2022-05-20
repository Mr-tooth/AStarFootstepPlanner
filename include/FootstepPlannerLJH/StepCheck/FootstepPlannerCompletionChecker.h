#pragma once

#include <vector>
#include <FootstepPlannerLJH/DataType.h>
#include <FootstepPlannerLJH/Title.h>
#include <FootstepPlannerLJH/StepCost/HeuristicCalculator.h>
#include <Heuclid/euclid/tools/HeuclidCoreTool.h>
using ljh::mathlib::HeuclidCoreTool;

#define GOAL_REACHED_SPECIFIC 2
#define GOAL_REACHED_PROXIMITY 1
#define GOAL_NO_REACHED 0
#define NEIGHBOR_EMPTY 3

#define YAW_SCALAR 3.0

_FOOTSTEP_PLANNER_BEGIN
class FootstepCompletionChecker
{
private:
    Pose2D<double> goalMidFootPose;
    double goalDistanceProximity;
    double goalYawProximity;

    Location startNode;
    Location endNode;

    cost_t endNodeCost;

    parameters param;
    // For ReachCheck
    Location stopStep;
    Pose2D<double> stopMidPose;
    // For static compare func public use for all object
    static Pose2D<double> squaredUpStep;
    static double posDisA;
    static double posDisB;
    static double yawScalar;
    static double yawDisA;
    static double yawDisB;
    // For set UpStep
    static double stepX;
    static double stepY;

public:
    FootstepCompletionChecker():goalMidFootPose(),goalDistanceProximity(),goalYawProximity(),
         startNode(),endNode(),endNodeCost(cost_t(0)),param(),stopStep(),stopMidPose(){};
    FootstepCompletionChecker(Pose2D<double> _goalMidFootPose,Location _startNode,double _goalDistanceProximity,double _goalYawProximity, HeuristicCalculator heuristic)
        :goalMidFootPose(_goalMidFootPose),goalDistanceProximity(_goalDistanceProximity),goalYawProximity(_goalYawProximity),
         startNode(_startNode),endNode(_startNode),endNodeCost(cost_t(0)),param(),stopStep(),stopMidPose()
        {
            endNodeCost = heuristic.compute(_startNode);
        };
    
    void initilize(Pose2D<double> _goalMidFootPose,Location _startNode,double _goalDistanceProximity,double _goalYawProximity, HeuristicCalculator heuristic);

    int checkIfGoalReached(Location current, std::vector<Location> neighbors, HeuclidCoreTool heuclidCoreTool, Location _goalL, Location _goalR);

    bool isProximityModeEnabled();
    static bool compareNode(Location a, Location b);
    void setIdealSquaredUpStep(DiscreteFootstep footstep, double IdealStepWidth);

    Location getEndNode() const {return this->endNode;};
    Location getStopNode() const {return this->stopStep;};

    

};




_FOOTSTEP_PLANNER_END