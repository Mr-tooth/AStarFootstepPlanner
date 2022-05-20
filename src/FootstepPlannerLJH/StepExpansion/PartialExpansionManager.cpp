
#include <FootstepPlannerLJH/StepExpansion/PartialExpansionManager.h>

_FOOTSTEP_PLANNER_BEGIN


void PartialExpansionManager::initialize(std::vector<FootstepGraphNode> _allChildNodes)
{
    //this->allChildNodes.clear();
    this->allChildNodes.swap(_allChildNodes);
    expansionCount = 0;
}

void PartialExpansionManager::packPartialExpansion(std::vector<FootstepGraphNode>& expansionToPack)
{
    expansionToPack.clear();

    if(this->finishedExpansion())
        return;
    
    int startIndex = this->param.MaxBranchFactor * this->expansionCount;
    int endIndex = min_ljh(startIndex+this->param.MaxBranchFactor,int(this->allChildNodes.size()));

    for(int i=startIndex;i<endIndex;i++)
        expansionToPack.push_back(this->allChildNodes.at(i));
    
    expansionCount++;
}

bool PartialExpansionManager::finishedExpansion()
{
    return this->expansionCount * param.MaxBranchFactor >= this->allChildNodes.size();
}

void PartialExpansionManager::operator=(const PartialExpansionManager& other)
{
    this->allChildNodes = other.allChildNodes;
    this->expansionCount = other.expansionCount;
}

void PartialExpansionManager::initialize()
{
    this->allChildNodes.clear();
    this->expansionCount = 0;
}

_FOOTSTEP_PLANNER_END

