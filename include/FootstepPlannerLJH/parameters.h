// Copyright 2026 Junhang Li
// SPDX-License-Identifier: Apache-2.0

#pragma once
#ifndef __PARAMETERS__
#define __PARAMETERS__

#define _FOOTSTEP_PLANNER_BEGIN namespace ljh{namespace path{namespace footstep_planner{
#define _FOOTSTEP_PLANNER_END }}}

#ifndef pi
static constexpr double pi = 3.1415926535;
#endif

//#define CON
#ifdef CON
#define CONST const
#else
#define CONST
#endif


#include <cmath>
#include <Heuclid/geometry/ConvexPolygon2D.h>
using ljh::heuclid::Point2D;
_FOOTSTEP_PLANNER_BEGIN
// extern double _edgecost_w_d;
// extern double _edgecost_w_yaw;
// extern double _edgecost_w_h;
// extern double _edgecost_w_area;
// extern double _edgecost_w_static;

/**
 * @class parameters
 * @brief Configuration parameters for footstep planning.
 *
 * Contains all tunable parameters for the A* footstep planner including:
 * - Step expansion constraints (length, width, yaw limits)
 * - Cost function weights (distance, yaw, height, area)
 * - Heuristic weights for A* search
 * - Goal proximity thresholds
 * - Stair alignment mode configuration
 */
class parameters
{
public:
    // weight params for edgecost
    static CONST double edgecost_w_d;
    static CONST double edgecost_w_yaw;
    static CONST double edgecost_w_h;
    static CONST double edgecost_w_area;
    static CONST double edgecost_w_static;
    /** @brief Weight for body path deviation penalty in edge cost.
     *  Penalizes footsteps that stray from the ellipsoid body path.
     *  Default: 0.0 (disabled). Typical tuning: 3.0–15.0. */
    static CONST double edgecost_w_pathdev;

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

    static CONST double HWPOfWalkDistacne;
    static CONST double HWPOfInitialTurnDistacne;
    static CONST double HWPOfFinalTurnDistacne;
    static CONST double HWPOfPathDistance;

    static CONST double HWPOfFinalWalkDistacne;
    static CONST double HWPOfFinalFinalTurnDistacne;

    // For Node Stop Check
    static CONST double goalDistanceProximity;
    static CONST double goalYawProximity;

    // choose whether to plot and cout middle outcome to debug
    static CONST bool debugFlag;

    // constraints for stair alignment// load in the step expansion
    static CONST bool isStairAlignMode;
    /** @brief Enable ellipsoid body path following heuristic in A* search.
     *  When true, the planner uses computeFollowEllipsoidPath() to guide
     *  footsteps along the body path from start to goal.
     *  Independent of isStairAlignMode (no stair constraints applied).
     *  @note Requires Simple2DBodyPathHolder to be initialized via HeuristicCalculator. */
    static CONST bool followBodyPath;
    static CONST ljh::heuclid::ConvexPolygon2D stairPolygon;

    /** @brief Landing zone polygons — terrain patches where each foot must be fully contained.
     *  When useLandingZoneCheck is true, each candidate footstep is validated
     *  using isConvexPolygonContained: ALL foot vertices must be inside at least
     *  one of these polygons. If the foot straddles two patches or extends
     *  beyond the terrain boundary, the step is rejected.
     *
     *  Use cases:
     *  - Stair climbing: each step is a small terrain patch, foot must land entirely on it
     *  - Stepping stones: narrow terrain patches, foot must not overhang
     *  - Obstacle avoidance: combined with stairPolygon for forbidden zone rejection
     *
     *  Distinction from stairPolygon:
     *  - stairPolygon + isStairAlignMode = forbidden zone (foot vertices rejected if inside)
     *  - landingZonePolygon + useLandingZoneCheck = valid zone (foot must be fully inside)
     */
    static CONST std::vector<ljh::heuclid::ConvexPolygon2D> landingZonePolygons;
    static CONST bool useLandingZoneCheck;

    static CONST double footPolygonExtendedLength;
    

    parameters(/* args */){};
    ~parameters(){};

    // configure the set and get func for all parameters
    double getEdgeCostDistance(const parameters& param);
    double getEdgeCostYaw(const parameters& param);
    double getEdgeCostHeight(const parameters& param);
    double getEdgeCostArea(const parameters& param);
    double getEdgeCostStaticPerStep(const parameters& param);
    /** @brief Get body path deviation edge cost weight. */
    double getEdgeCostPathDev(const parameters& param);
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
    /** @brief Check if body path following heuristic is enabled. */
    bool   getFollowBodyPath(const parameters& param);
    ljh::heuclid::ConvexPolygon2D getStairPolygon(const parameters& param);
    const std::vector<ljh::heuclid::ConvexPolygon2D>& getLandingZonePolygons(const parameters& param);
    bool   getUseLandingZoneCheck(const parameters& param);
    double getFootPolygonExtendedLength(const parameters& param);
    double getHWPOfWalkDistacne(const parameters& param);
    double getHWPOfInitialTurnDistacne(const parameters& param);
    double getHWPOfFinalTurnDistacne(const parameters& param);
    double getHWPOfPathDistance(const parameters& param);
    double getHWPOfFinalWalkDistacne(const parameters& param);
    double getHWPOfFinalFinalTurnDistacne(const parameters& param);

    void SetEdgeCostDistance(parameters& param, const double& change);
    void SetEdgeCostYaw(parameters& param, const double& change);
    void SetEdgeCostHeight(parameters& param, const double& change);
    void SetEdgeCostArea(parameters& param, const double& change);
    void SetEdgeCostStaticPerStep(parameters& param, const double& change);
    /** @brief Set body path deviation edge cost weight. @param change Weight value (0.0 to disable). */
    void SetEdgeCostPathDev(parameters& param, const double& change);
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
    /** @brief Enable/disable body path following heuristic (independent of stair mode). */
    void SetFollowBodyPath(parameters& param, const bool& change);
    void SetStairPolygon(parameters& param, std::vector<Point2D<double> > stairBuffer, int numOfVertices, bool clockwiseOrdered);
    void SetLandingZonePolygons(parameters& param, const std::vector<ljh::heuclid::ConvexPolygon2D>& polygons);
    void SetUseLandingZoneCheck(parameters& param, const bool& change);
    void SetFootPolygonExtendedLength(const parameters& param, const double& change);
    void SetHWPOfWalkDistacne(const parameters& param, const double& change);
    void SetHWPOfInitialTurnDistacne(const parameters& param, const double& change);
    void SetHWPOfFinalTurnDistacne(const parameters& param, const double& change);
    void SetHWPOfPathDistance(const parameters& param, const double& change);
    void SetHWPOfFinalWalkDistacne(const parameters& param, const double& change);
    void SetHWPOfFinalFinalTurnDistacne(const parameters& param, const double& change);
};




_FOOTSTEP_PLANNER_END

#endif
