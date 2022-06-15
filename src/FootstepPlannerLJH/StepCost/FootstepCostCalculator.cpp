
#include <FootstepPlannerLJH/StepCost/FootstepCostCalculator.h>

_FOOTSTEP_PLANNER_BEGIN


cost_t FootstepCostCalculator::computeTotalCost(Location candidateNode, Location stanceNode)
{
    this->edgeCost = this->computeEdgeCost(candidateNode,stanceNode);

    this->heuristicCost = this->computeHeuristicCost(candidateNode);

    this->totalCost = this->edgeCost + this->heuristicCost;
    return this->totalCost;
}


cost_t FootstepCostCalculator::computeEdgeCost(Location candidateNode, Location stanceNode)
{
    this->xOffset = std::abs(candidateNode.getSecondStep().getMidFootPoint().getX()-stanceNode.getSecondStep().getMidFootPoint().getX());
    this->yOffset = std::abs(candidateNode.getSecondStep().getMidFootPoint().getY()-stanceNode.getSecondStep().getMidFootPoint().getY());
    this->yawOffset = std::abs(candidateNode.getSecondStep().getYaw()-stanceNode.getSecondStep().getYaw());

    this->lenOffset = sqrt(this->xOffset*this->xOffset + this->yOffset*this->yOffset);

    this->edgeCost = this->lenOffset * this->param.edgecost_w_d 
                    +this->yawOffset * this->param.edgecost_w_yaw
                    //+this->zOffset   * this->param.edgecost_w_h
                    +this->param.edgecost_w_static;

    return this->edgeCost;
}


cost_t FootstepCostCalculator::computeHeuristicCost(Location candidateNode)
{
   this->heuristicCost = this->heuristicCalculator.compute(candidateNode);
   return this->heuristicCost;
}

cost_t FootstepCostCalculator::computeHeuristicCostWithEllipsiodPath(Location candidateNode)
{
    this->heuristicCost = this->heuristicCalculator.computeFollowEllipsoidPath(candidateNode);
    return this->heuristicCost;
}

void FootstepCostCalculator::initialize(Pose3D<double> _goalPose, Pose3D<double> _startPose)
{
    this->edgeCost = cost_t(0);
    this->totalCost = cost_t(0);
    this->heuristicCost = cost_t(0);

    this->xOffset = cost_t(0);
    this->yOffset = cost_t(0);
    this->yawOffset = cost_t(0);
    this->lenOffset = cost_t(0);

    this->heuristicCalculator.initialize(_goalPose,_startPose);
}

// cost_t FootstepCostCalculator::computeCost(DiscreteFootstep candidateStep, DiscreteFootstep stanceStep, DiscreteFootstep startOfSwing)
// {
//     this->xOffset = std::abs(candidateStep.getMidFootPoint().getX() - stanceStep.getMidFootPoint().getX());
//     this->yOffset = std::abs(candidateStep.getMidFootPoint().getY() - stanceStep.getMidFootPoint().getY());
//     this->yawOffset = std::abs(candidateStep.getYaw()-stanceStep.getYaw());
    
//     // this->zOffset = cost_t(0);
//     // this->pitchOffset = cost_t(0);
//     // this->rollOffset = cost_t(0);

//     this->lenOffset = sqrt(this->xOffset*this->xOffset + this->yOffset*this->yOffset);

//     this->edgeCost = this->lenOffset * this->param.edgecost_w_d 
//                     +this->yawOffset * this->param.edgecost_w_yaw
//                     //+this->zOffset   * this->param.edgecost_w_h
//                     +this->param.edgecost_w_static;


//     this->totalCost = this->edgeCost + this->heuristicCost;
//     return this->totalCost;
// }



_FOOTSTEP_PLANNER_END