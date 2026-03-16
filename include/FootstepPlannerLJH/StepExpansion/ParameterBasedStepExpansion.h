// Copyright 2026 Junhang Li
// SPDX-License-Identifier: Apache-2.0

#pragma once


#define _FOOTSTEP_PLANNER_BEGIN namespace ljh{namespace path{namespace footstep_planner{
#define _FOOTSTEP_PLANNER_END }}}

#include <cmath>
#include <algorithm>
#include <vector>
#include <unordered_map>
#include <FootstepPlannerLJH/parameters.h>
#include <FootstepPlannerLJH/FootstepplannerBasic.h>
#include <Heuclid/euclid/tuple2D/Vector2D.h>
#include <FootstepPlannerLJH/StepExpansion/PartialExpansionManager.h>
#include <FootstepPlannerLJH/StepConstraints/StepConstraintCheck.h>

using ljh::heuclid::Vector2D;
_FOOTSTEP_PLANNER_BEGIN

/**
 * @class ParameterBasedStepExpansion
 * @brief Parameter-based step expansion strategy for footstep planning.
 *
 * Generates possible successor footsteps from a given stance by exploring
 * a parameterized action space defined by step length, width, and yaw ranges.
 * Supports both full and partial expansion modes for efficiency.
 */
class ParameterBasedStepExpansion
{
private:
    LatticePoint latPoForFunc;
    parameters param;
    std::vector<double> xOffsets;
    std::vector<double> yOffsets;
    std::vector<double> yawOffsets;

    std::vector<FootstepGraphNode> fullExpansion;
    bool partialExpansionEnabled;
    PartialExpansionManager Manager;
    std::unordered_map<FootstepGraphNode,PartialExpansionManager,FootNodeHash> expansionManager;

    StepConstraintCheck stepConstraintChecker;

    // middle param
    double midStepLength;
    double midStepWidth;
    double midStepYaw;
    DiscreteFootstep childStep;
    Vector2D<double> midXY;
    FootstepGraphNode childNode;


public:
    ParameterBasedStepExpansion(/* args */)
        :latPoForFunc(),param(),xOffsets(),yOffsets(),yawOffsets(),fullExpansion(),partialExpansionEnabled(false),
        Manager(),expansionManager(),stepConstraintChecker(),midStepLength(0.0),midStepWidth(0.0),midStepYaw(0.0),childStep(),midXY(),() {};
    //~ParameterBasedStepExpansion();

    /**
     * @brief Initialize the step expansion module with default parameters.
     */
    void initialize();

    /**
     * @brief Perform iterative step expansion from a stance.
     * @param stanceStep Current stance node
     * @param expansionToPack Output vector of expanded successor nodes
     * @return true if expansion completed, false if more iterations needed
     */
    bool doIterativeExpansion(FootstepGraphNode stanceStep,std::vector<FootstepGraphNode>& expansionToPack);

    /**
     * @brief Perform full step expansion (all possible successors at once).
     * @param nodeToExpand Current stance node
     * @param fullExpansionToPack Output vector of all expanded successor nodes
     */
    void doFullExpansion(FootstepGraphNode nodeToExpand,std::vector<FootstepGraphNode>& fullExpansionToPack);

    /**
     * @brief Construct a child footstep in the parent frame.
     * @param steplen Step length
     * @param stepwid Step width
     * @param stepyaw Step yaw angle
     * @param step Parent footstep
     * @return Constructed child footstep
     */
    DiscreteFootstep constructNodeInPreviousNodeFrame(double steplen, double stepwid, double stepyaw,DiscreteFootstep step);
};

// ParameterBasedStepExpansion::ParameterBasedStepExpansion(/* args */)
// {
// }

// ParameterBasedStepExpansion::~ParameterBasedStepExpansion()
// {
// }






_FOOTSTEP_PLANNER_END
