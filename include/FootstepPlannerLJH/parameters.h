#pragma once
#ifndef __PARAMETERS__
#define __PARAMETERS__

#define _FOOTSTEP_PLANNER_BEGIN namespace ljh{namespace path{namespace footstep_planner{
#define _FOOTSTEP_PLANNER_END }}}

#ifndef pi
#define pi 3.1415926535
#endif

//#define CON
#ifdef CON
#define CONST const
#else
#define CONST
#endif


#include <cmath>
#include <Heuclid/geometry/ConvexPolygon2D.h>
using ljh::mathlib::Point2D;
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
    static CONST double edgecost_w_d;
    static CONST double edgecost_w_yaw;
    static CONST double edgecost_w_h;
    static CONST double edgecost_w_area;
    static CONST double edgecost_w_static;

    //StepNodeExpansionCheck
    static CONST double MaxStepReach;
    //NodeExpansion
    static CONST double MinStepLength;
    static CONST double MaxStepLength;
    static CONST double MinStepWidth;
    static CONST double MaxStepWidth;

    // Robot Param
    static CONST double IdealStepWidth;

    // Yaw control Param
    static CONST double MinStepYaw;
    static CONST double MaxStepYaw;
    static CONST double StepYawReductionFactorAtMaxReach;

    // don't know
    static CONST int MaxBranchFactor;

    // Heuristic cost
    static CONST double FinalTurnProximity;
    static CONST double AStarHeuristicWeight;
    static CONST double AStarHeuristicFinalWeight;
    
    // For Node Stop Check
    static CONST double goalDistanceProximity;
    static CONST double goalYawProximity;

    // choose whether to plot and cout middle outcome to debug
    static CONST bool debugFlag;

    // constraints for stair alignment// load in the step expansion
    static CONST bool isStairAlignMode;
    static CONST ljh::mathlib::ConvexPolygon2D stairPolygon;
    static CONST double footPolygonExtendedLength;
    

    parameters(/* args */){};
    ~parameters(){};

    // configure the set and get func for all parameters
    double getEdgeCostDistance(const parameters& param);
    double getEdgeCostYaw(const parameters& param);
    double getEdgeCostHeight(const parameters& param);
    double getEdgeCostArea(const parameters& param);
    double getEdgeCostStaticPerStep(const parameters& param);
    double getMaxStepReach(const parameters& param);
    double getMinStepLength(const parameters& param);
    double getMaxStepLength(const parameters& param);
    double getMinStepWidth(const parameters& param);
    double getMaxStepWidth(const parameters& param);
    double getIdealStepWidth(const parameters& param);
    double getMinStepYaw(const parameters& param);
    double getMaxStepYaw(const parameters& param);
    double getStepYawReductionFactorAtMaxReach(const parameters& param);
    int    getMaxBranchFactor(const parameters& param);
    double getFinalTurnProximity(const parameters& param);
    double getAStartHeuristicWeight(const parameters& param);
    double getAStartHeuristicFinalWeight(const parameters& param);
    double getGoalDistanceProximity(const parameters& param);
    double getGoalYawProximity(const parameters& param);
    bool   getDebugFlag(const parameters& param);
    bool   getStairAlignMode(const parameters& param);
    ljh::mathlib::ConvexPolygon2D getStairPolygon(const parameters& param);
    double getFootPolygonExtendedLength(const parameters& param);

    void SetEdgeCostDistance(parameters& param, const double& change);
    void SetEdgeCostYaw(parameters& param, const double& change);
    void SetEdgeCostHeight(parameters& param, const double& change);
    void SetEdgeCostArea(parameters& param, const double& change);
    void SetEdgeCostStaticPerStep(parameters& param, const double& change);
    void SetMaxStepReach(parameters& param, const double& change);
    void SetMinStepLength(parameters& param, const double& change);
    void SetMaxStepLength(parameters& param, const double& change);
    void SetMinStepWidth(parameters& param, const double& change);
    void SetMaxStepWidth(parameters& param, const double& change);
    void SetIdealStepWidth(parameters& param, const double& change);
    void SetMinStepYaw(parameters& param, const double& change);
    void SetMaxStepYaw(parameters& param, const double& change);
    void SetStepYawReductionFactorAtMaxReach(parameters& param, const double& change);
    void SetMaxBranchFactor(parameters& param, const int& change);
    void SetFinalTurnProximity(parameters& param, const double& change);
    void SetAStartHeuristicWeight(parameters& param, const double& change);
    void SetAStartHeuristicFinalWeight(parameters& param, const double& change);
    void SetGoalDistanceProximity(parameters& param, const double& change);
    void SetGoalYawProximity(parameters& param, const double& change);
    void SetDebugFlag(parameters& param, const bool& change);
    void SetStairAlignMode(parameters& param, const bool& change);
    void SetStairPolygon(parameters& param, std::vector<Point2D<double> > stairBuffer, int numOfVertices, bool clockwiseOrdered);
    void SetFootPolygonExtendedLength(const parameters& param, const double& change);
};




_FOOTSTEP_PLANNER_END

#endif