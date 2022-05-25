
#include <FootstepPlannerLJH/StepExpansion/ParameterBasedStepExpansion.h>
#include<iostream>
_FOOTSTEP_PLANNER_BEGIN

void ParameterBasedStepExpansion::initialize()
{
    this->xOffsets.clear();
    this->yOffsets.clear();
    this->yawOffsets.clear();

    double maxReachSquared = param.MaxStepReach * param.MaxStepReach;
    for (double x = param.MinStepLength;x<=param.MaxStepLength; x+=latPoForFunc.gridSizeXY)
    {
       for (double y = param.MinStepWidth;y<=param.MaxStepWidth; y+=latPoForFunc.gridSizeXY) 
       {
            double relativeY2Ideal = y - param.IdealStepWidth;
            double reachSquared = x * x + relativeY2Ideal * relativeY2Ideal;

            if (reachSquared > maxReachSquared)
                continue; // drop the Node out of leg reach circle region 
            
            double reachFraction = sqrt(reachSquared)/param.MaxStepReach;

            double minYawAtFullExtension = (1-param.StepYawReductionFactorAtMaxReach) * param.MinStepYaw;
            double maxYawAtFullExtension = (1-param.StepYawReductionFactorAtMaxReach) * param.MaxStepYaw;

            //  linearInterpolate- Larger steplength, smaller the yaw admissable region
            double minYaw =  (1-reachFraction) * param.MinStepYaw + reachFraction * minYawAtFullExtension; 
            double maxYaw =  (1-reachFraction) * param.MaxStepYaw + reachFraction * maxYawAtFullExtension;

            // 
            minYaw = round(minYaw/latPoForFunc.gridSizeYaw)*latPoForFunc.gridSizeYaw;
            maxYaw = round(maxYaw/latPoForFunc.gridSizeYaw)*latPoForFunc.gridSizeYaw;
            for(double yaw = minYaw;yaw<=maxYaw;yaw += latPoForFunc.gridSizeYaw)
            {
                // double distance = computeDistanceBetweenFootPolygons(DiscreteFootstep(0,0,0,RobotSide.Right),
                
                // need a stance clear region  condition                                                    //DiscreteFootstep(x,y,yaw,RobotSide.Left))
                // Reserve the node which is collided with stance clear region but is parallel to the stance foot in yaw
                // if(!(this->stepConstraintChecker.isTwoFootCollided(0.0,0.0,0.0,stepR, x,y,yaw,stepL) && std::abs(yaw)>1e-4 ))
                // {
                    this->xOffsets.push_back(x);
                    this->yOffsets.push_back(y);
                    this->yawOffsets.push_back(yaw);
                // }
                // else{
                //     std::cout<<" FootColliding! "<< "X: "<<x<<" Y: "<<y<<" Yaw: "<<yaw<<std::endl;
                // }
            }
       }
    }

    this->latPoForFunc = LatticePoint();
    this->fullExpansion.clear();
    this->partialExpansionEnabled = param.MaxBranchFactor>0;
    this->Manager.initialize();
    this->expansionManager.clear();

    this->stepConstraintChecker.initialize();

    this->midStepLength = 0.0;
    this->midStepWidth = 0.0;
    this->midStepYaw = 0.0;
    this->childStep = DiscreteFootstep();
    this->midXY = Vector2D<double>();
    this->childNode = FootstepGraphNode();

}

/**
    * Packs part of the full expansion. Successive calls to this method will pack different subsets of the full expansion
    * (in decreasing order of queue priority. in progress...)
    *
    * @return whether the stance node has more child nodes
    */
bool ParameterBasedStepExpansion::doIterativeExpansion(FootstepGraphNode stanceStep,std::vector<FootstepGraphNode>& expansionToPack)
{
    if(this->partialExpansionEnabled)
    {
        if(this->expansionManager.find(stanceStep) == this->expansionManager.end())
        {
            this->doFullExpansion(stanceStep,this->fullExpansion);
            this->Manager.initialize(this->fullExpansion);
            this->expansionManager.emplace(stanceStep,this->Manager);            
        }
        else
        {
            this->Manager = expansionManager.at(stanceStep);
        }
        // has to figure out the usage of expansionCount
        this->expansionManager.at(stanceStep).plusCount();
        // 需要测试是否真的需要把map中的count也加1，按照道理来说是需要的；
        // 因为不可能每次取扩展子集的时候每次都只取走第一部分，不符合函数说明；
        // 应该是针对同一个stancestep，如果取多次，则按顺序依次分块取走

        this->Manager.packPartialExpansion(expansionToPack);
        return !this->Manager.finishedExpansion();
    }
    else
    {
        this->doFullExpansion(stanceStep, expansionToPack);
        return false;
    }
}

void ParameterBasedStepExpansion::doFullExpansion(FootstepGraphNode nodeToExpand,std::vector<FootstepGraphNode>& fullExpansionToPack)
{

    fullExpansionToPack.clear();
    RobotSide stepside = nodeToExpand.getFirstStepSide();

    for(int i=0;i<this->xOffsets.size();i++)
    {
        midStepLength = this->xOffsets[i];
        midStepWidth = stepside.negateIfRightSide(this->yOffsets[i]);
        midStepYaw = stepside.negateIfRightSide(this->yawOffsets[i]);
        childStep = constructNodeInPreviousNodeFrame(midStepLength,midStepWidth,midStepYaw,nodeToExpand.getSecondStep());

        // add check  whether the foot is in stair polygon
        if(this->param.isStairAlignMode)
        {
            if(this->stepConstraintChecker.isAnyVertexOfFootInsideStairRegion(childStep,this->param.stairPolygon))
                continue; //drop the childstep if it's in stair region cause it's unrealiazbile
        }


        childNode.setNode(nodeToExpand.getSecondStep(),childStep);
        double yawDistance = (childNode.getSecondStep().getYaw()-childNode.getFirstStep().getYaw() );
        //std::cout<<"yaw distance is : "<<yawDistance-midStepYaw<<std::endl;
        if(this->stepConstraintChecker.isTwoFootCollided(childNode) && std::abs(yawDistance) >1e-4 )
            continue;
        
        if(std::find(fullExpansionToPack.begin(),fullExpansionToPack.end(),childNode) == fullExpansionToPack.end())
        {
            fullExpansionToPack.push_back(childNode);
        }

    }
}


DiscreteFootstep ParameterBasedStepExpansion:: constructNodeInPreviousNodeFrame(double steplen, double stepwid, double stepyaw,DiscreteFootstep step)
{
    // Not using AxisAngle as ihmc, just use rotate matrix directly in 2DPlanePlanning
    
    midXY.setX(( cos(step.getYaw()) * steplen - sin(step.getYaw()) * stepwid ));
    midXY.setY(( sin(step.getYaw()) * steplen + cos(step.getYaw()) * stepwid ));

    DiscreteFootstep mid = DiscreteFootstep(step.getX()+midXY.getX(), step.getY()+midXY.getY(), step.getYaw()+stepyaw, step.getRobotSide().getOppositeSide());
    return mid;

}

_FOOTSTEP_PLANNER_END