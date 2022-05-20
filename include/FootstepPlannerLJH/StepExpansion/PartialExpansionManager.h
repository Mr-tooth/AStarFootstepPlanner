#pragma once

#define _FOOTSTEP_PLANNER_BEGIN namespace ljh{namespace path{namespace footstep_planner{
#define _FOOTSTEP_PLANNER_END }}}

#include <FootstepPlannerLJH/parameters.h>
#include <FootstepPlannerLJH/FootstepplannerBasic.h>
#include <vector>
_FOOTSTEP_PLANNER_BEGIN
class PartialExpansionManager
{
private:
    parameters param;
    std::vector<FootstepGraphNode> allChildNodes;
    int expansionCount; 
public:
    PartialExpansionManager():param(), allChildNodes(),expansionCount(0){};
    void initialize(std::vector<FootstepGraphNode> _allChildNodes);
    void packPartialExpansion(std::vector<FootstepGraphNode>& expansionToPack);
    bool finishedExpansion();
    void operator=(const PartialExpansionManager& other);
    inline void plusCount() {this->expansionCount++;};

    void initialize();

};






_FOOTSTEP_PLANNER_END