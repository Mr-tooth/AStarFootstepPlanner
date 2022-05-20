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

using ljh::mathlib::Vector2D;
_FOOTSTEP_PLANNER_BEGIN

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
        Manager(),expansionManager(),stepConstraintChecker(),midStepLength(0.0),midStepWidth(0.0),midStepYaw(0.0),childStep(),midXY(),childNode() {};
    //~ParameterBasedStepExpansion();
    void initialize();
    bool doIterativeExpansion(FootstepGraphNode stanceStep,std::vector<FootstepGraphNode>& expansionToPack);
    void doFullExpansion(FootstepGraphNode nodeToExpand,std::vector<FootstepGraphNode>& fullExpansionToPack);
    DiscreteFootstep constructNodeInPreviousNodeFrame(double steplen, double stepwid, double stepyaw,DiscreteFootstep step);
};

// ParameterBasedStepExpansion::ParameterBasedStepExpansion(/* args */)
// {
// }

// ParameterBasedStepExpansion::~ParameterBasedStepExpansion()
// {
// }






_FOOTSTEP_PLANNER_END

