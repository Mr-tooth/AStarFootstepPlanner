#pragma once
#ifndef __PARAMETERS__
#define __PARAMETERS__

#define _FOOTSTEP_PLANNER_BEGIN namespace ljh{namespace path{namespace footstep_planner{
#define _FOOTSTEP_PLANNER_END }}}

#ifndef pi
#define pi 3.1415926535
#endif

#include <cmath>
_FOOTSTEP_PLANNER_BEGIN
// extern double _edgecost_w_d;
// extern double _edgecost_w_yaw;
// extern double _edgecost_w_h;
// extern double _edgecost_w_area;
// extern double _edgecost_w_static;

class parameters
{
public:
    // weight params for edgecost
    static const double edgecost_w_d;
    static const double edgecost_w_yaw;
    static const double edgecost_w_h;
    static const double edgecost_w_area;
    static const double edgecost_w_static;

    //StepNodeExpansionCheck
    static const double MaxStepReach;
    //NodeExpansion
    static const double MinStepLength;
    static const double MaxStepLength;
    static const double MinStepWidth;
    static const double MaxStepWidth;

    // Robot Param
    static const double IdealStepWidth;

    // Yaw control Param
    static const double MinStepYaw;
    static const double MaxStepYaw;
    static const double StepYawReductionFactorAtMaxReach;

    // don't know
    static const int MaxBranchFactor;

    // Heuristic cost
    static const double FinalTurnProximity;
    static const double AStarHeuristicWeight;
    static const double AStarHeuristicFinalWeight;
    
    // For Node Stop Check
    static const double goalDistanceProximity;
    static const double goalYawProximity;

    parameters(/* args */){};
    ~parameters(){};
};




_FOOTSTEP_PLANNER_END

#endif