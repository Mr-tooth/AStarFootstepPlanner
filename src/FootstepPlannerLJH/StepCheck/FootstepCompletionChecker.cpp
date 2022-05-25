
#include <cmath>
#include <algorithm>
#include <FootstepPlannerLJH/StepCheck/FootstepPlannerCompletionChecker.h>
#include <iostream>
_FOOTSTEP_PLANNER_BEGIN

Pose2D<double> FootstepCompletionChecker::squaredUpStep = Pose2D<double>();
double FootstepCompletionChecker::posDisA = 0.0;
double FootstepCompletionChecker::posDisB = 0.0;
double FootstepCompletionChecker::yawScalar = YAW_SCALAR;
double FootstepCompletionChecker::yawDisA = 0.0;
double FootstepCompletionChecker::yawDisB = 0.0;
double FootstepCompletionChecker::stepX = 0.0;
double FootstepCompletionChecker::stepY = 0.0;


void FootstepCompletionChecker::initilize(Pose2D<double> _goalMidFootPose,Location _startNode,double _goalDistanceProximity,double _goalYawProximity, HeuristicCalculator heuristic)
{
    this->goalMidFootPose = _goalMidFootPose;
    this->goalDistanceProximity = _goalDistanceProximity;
    this->goalYawProximity = _goalYawProximity;
    this->startNode = _startNode;
    this->endNode = _startNode;
    this->endNodeCost = heuristic.compute(_startNode);

    this->stopStep = Location();
    this->stopMidPose = Pose2D<double>();

    // static
    this->squaredUpStep = Pose2D<double>();
    this->posDisA = 0.0;
    this->posDisB = 0.0;
    this->yawScalar = YAW_SCALAR;
    this->yawDisA = 0.0;
    this->yawDisB = 0.0;
    
    this->stepX = 0.0;
    this->stepY = 0.0;

}



int FootstepCompletionChecker::checkIfGoalReached(Location current, std::vector<Location> neighbors, HeuclidCoreTool heuclidCoreTool, Location _goalL, Location _goalR )
{
    if(this->isProximityModeEnabled())
    {
        this->setIdealSquaredUpStep(current.getSecondStep(),this->param.IdealStepWidth);
        if(this->param.debugFlag)
            std::cout<<"Neighbor size before: " <<neighbors.size()<<std::endl;
        sort(neighbors.begin(),neighbors.end(),compareNode);
        //std::cout<<"Neighbor size: " <<neighbors.size()<<std::endl;
        if(!neighbors.empty())
            this->stopStep = neighbors.at(0);
        else
        {
            // neighbor is empty! Search Terminate!
            this->endNode = current;
            return NEIGHBOR_EMPTY;
        }
             
        this->stopMidPose = stopStep.getOrComputeMidFootPose();

        double xyDis = heuclidCoreTool.norm(this->stopMidPose.getPosition().getX()-this->goalMidFootPose.getPosition().getX(),
                                            this->stopMidPose.getPosition().getY()-this->goalMidFootPose.getPosition().getY());
        double yawDis = std::abs(this->stopMidPose.getOrientation().getYaw()-this->goalMidFootPose.getOrientation().getYaw());
        if(xyDis<this->goalDistanceProximity && yawDis<this->goalYawProximity)
        {
            this->endNode = this->stopStep;
            return GOAL_REACHED_PROXIMITY;
        }
        // else
        // {
        //     return GOAL_NO_REACHED;
        // }
        
    }
    else// specific mode stop at the stand endnode after stopnode
    {
        for(int i=0;i<neighbors.size();i++)
        {
            this->stopStep = neighbors.at(i);
            if(this->stopStep.getSecondStep().getRobotSide().getStepFlag()==stepL && this->stopStep.getSecondStep()==_goalL.getSecondStep())
            {
                this->endNode = _goalR;
                return GOAL_REACHED_SPECIFIC;
            }
            else if(this->stopStep.getSecondStep().getRobotSide().getStepFlag()==stepR && this->stopStep.getSecondStep()==_goalR.getSecondStep())
            {
                this->endNode = _goalL;
                return GOAL_REACHED_SPECIFIC;
            }
        }
    }

return GOAL_NO_REACHED;
}



bool FootstepCompletionChecker::isProximityModeEnabled()
{
    return (this->goalDistanceProximity>0.0||this->goalYawProximity>0.0);
}


bool FootstepCompletionChecker::compareNode(Location a, Location b)
{
    posDisA = sqrt(pow(a.getSecondStep().getX()-squaredUpStep.getPosition().getX(),2)+ pow(a.getSecondStep().getY()-squaredUpStep.getPosition().getY(),2));
    posDisB = sqrt(pow(b.getSecondStep().getX()-squaredUpStep.getPosition().getX(),2)+ pow(b.getSecondStep().getY()-squaredUpStep.getPosition().getY(),2));
    
    yawDisA = yawScalar * a.getStanceAngle();
    yawDisB = yawScalar * b.getStanceAngle();

    return (posDisA+yawDisA)<(posDisB+yawDisB) ;
} 

void FootstepCompletionChecker::setIdealSquaredUpStep(DiscreteFootstep footstep, double IdealStepWidth)
{
    this->stepX =  footstep.getRobotSide().negateIfRightSide(sin(footstep.getYaw()) * IdealStepWidth);
    this->stepY = -footstep.getRobotSide().negateIfRightSide(cos(footstep.getYaw()) * IdealStepWidth);

    this->squaredUpStep.setX(footstep.getX() + this->stepX);
    this->squaredUpStep.setY(footstep.getY() + this->stepY);
    this->squaredUpStep.setYaw(footstep.getYaw());
}
_FOOTSTEP_PLANNER_END