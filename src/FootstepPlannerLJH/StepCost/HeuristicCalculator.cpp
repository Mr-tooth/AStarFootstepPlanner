#include <FootstepPlannerLJH/StepCost/HeuristicCalculator.h>

_FOOTSTEP_PLANNER_BEGIN

void HeuristicCalculator::initialize(Pose3D<double> _goalPose,Pose3D<double> _startPose)
{
    this->heuristicCost = cost_t(0);

    this->goalPose = _goalPose;
    this->startPose = _startPose;

    this->desireHeading = 0.0;
    this->midFootPose = Pose2D<double>();
    this->initialTurnDistance = 0.0;
    this->walkDistance = 0.0;
    this->finalTurnDistance = 0.0;
}

// the function shiftproperyaw should be out of the class to use smoothly
cost_t HeuristicCalculator::compute(FootstepGraphNode& node)
{
    this->midFootPose = node.getOrComputeMidFootPose();
    this->walkDistance = sqrt(
        pow(this->midFootPose.getPosition().getX()-this->goalPose.getPosition().getX(),2) +
        pow(this->midFootPose.getPosition().getY()-this->goalPose.getPosition().getY(),2));
    
    if(this->walkDistance < this->param.FinalTurnProximity)
    {
        this->finalTurnDistance = std::abs(this->midFootPose.getOrientation().shiftProperYaw(
            this->midFootPose.getOrientation().getYaw()-this->goalPose.getOrientation().getYaw())) * 0.5 * PI * this->param.IdealStepWidth;
        
        //this->heuristicCost = cost_t(this->param.AStarHeuristicWeight * (this->finalTurnDistance+this->walkDistance));
        this->heuristicCost = cost_t(this->param.AStarHeuristicFinalWeight * (this->finalTurnDistance)
                                    +this->param.AStarHeuristicFinalWeight * this->param.HWPOfWalkDistacne * (this->walkDistance));
    }
    else
    {  // turn the Ideal Yaw Traj as Online input
        //this->desireHeading = this->midFootPose.getOrientation().shiftProperYaw(this->goalPose.getOrientation().getYaw()-this->startPose.getOrientation().getYaw());
        double y = (goalPose.getPosition().getY()-startPose.getPosition().getY());
        double x = (goalPose.getPosition().getX()-startPose.getPosition().getX());
        this->desireHeading = this->midFootPose.getOrientation().shiftProperYaw(atan2(y,x));
        
        // this->initialTurnDistance = std::abs(this->midFootPose.getOrientation().shiftProperYaw(
        //     this->midFootPose.getOrientation().getYaw()-this->desireHeading)) * 0.5 * PI * this->param.IdealStepWidth;

        this->initialTurnDistance = (std::abs(this->midFootPose.getOrientation().shiftProperYaw(node.getFirstStep().getYaw()-this->desireHeading))
                                    +std::abs(this->midFootPose.getOrientation().shiftProperYaw(node.getSecondStep().getYaw()-this->desireHeading))) 
                                    * 0.5 * PI * this->param.IdealStepWidth;

        //this->finalTurnDistance = 0.0;
        this->finalTurnDistance = std::abs(this->midFootPose.getOrientation().shiftProperYaw(
            this->desireHeading - this->goalPose.getOrientation().getYaw())) * 0.5 * PI * this->param.IdealStepWidth;

        this->heuristicCost = cost_t(this->param.AStarHeuristicWeight * (this->param.HWPOfInitialTurnDistacne * this->initialTurnDistance 
                                                                       + this->param.HWPOfWalkDistacne        * this->walkDistance 
                                                                       + this->param.HWPOfFinalTurnDistacne   * this->finalTurnDistance));
    }

    return this->heuristicCost;
}


_FOOTSTEP_PLANNER_END