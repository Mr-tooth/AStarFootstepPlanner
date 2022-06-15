#pragma once
#ifndef __FOOTSTEP__COST__CALCULATOR__
#define __FOOTSTEP__COST__CALCULATOR__



#include <FootstepPlannerLJH/parameters.h>
//#include <FootstepPlannerLJH/FootstepplannerBasic.h>
#include <FootstepPlannerLJH/Title.h>
#include <FootstepPlannerLJH/DataType.h>
#include <FootstepPlannerLJH/StepCost/HeuristicCalculator.h>

_FOOTSTEP_PLANNER_BEGIN

// Regular version of LJH cost calculation
class FootstepCostCalculator
{
private:
    parameters param;
    cost_t edgeCost;
    cost_t totalCost;
    cost_t heuristicCost;

    // edgecost
    cost_t xOffset;
    cost_t yOffset;
    //cost_t zOffset;
    cost_t yawOffset;
    //cost_t pitchOffset;
    //cost_t rollOffset;

    cost_t lenOffset;
    
    
public:
    HeuristicCalculator heuristicCalculator;
    FootstepCostCalculator()
        :param(),edgeCost(cost_t(0)),totalCost(cost_t(0)),heuristicCost(cost_t(0)),xOffset(cost_t(0)),yOffset(cost_t(0)),yawOffset(cost_t(0)),lenOffset(cost_t(0)),heuristicCalculator(){};
    
    FootstepCostCalculator(Pose3D<double> _goalPose, Pose3D<double> _startPose)
        :param(),edgeCost(cost_t(0)),totalCost(cost_t(0)),heuristicCost(cost_t(0)),xOffset(cost_t(0)),yOffset(cost_t(0)),yawOffset(cost_t(0)),lenOffset(cost_t(0)),
        heuristicCalculator(_goalPose,_startPose){};
    cost_t computeTotalCost(Location candidateNode, Location stanceNode);
    //cost_t computeCost(DiscreteFootstep candidateStep, DiscreteFootstep stanceStep, DiscreteFootstep startOfSwing);

    cost_t computeEdgeCost(Location candidateNode, Location stanceNode);
    cost_t computeHeuristicCost(Location candidateNode);
    cost_t computeHeuristicCostWithEllipsiodPath(Location candidateNode);
    void initialize(Pose3D<double> _goalPose, Pose3D<double> _startPose);

    cost_t getEdgeCost() const {return this->edgeCost;};
    cost_t getHeuristicCost() const {return this->heuristicCost;};

};



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

#endif