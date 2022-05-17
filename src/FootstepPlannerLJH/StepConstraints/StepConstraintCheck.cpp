
#include <FootstepPlannerLJH\StepConstraints\StepConstraintCheck.h>

_FOOTSTEP_PLANNER_BEGIN

bool StepConstraintCheck::isAnyVertexOfFootInsideStairRegion(double stepX, double stepY, double stepYaw, enum StepFlag stepFlag,
                                            std::vector<Point2D<double> > stairVertexBuffer, int numOfVertices, bool clockwiseOrdered)
{
    this->stepPose.setPosition(stepX,stepY);
    this->stepPose.setOrientation(stepYaw);
    getFootVertex2D(this->stepPose,stepFlag,this->vertexX,this->vertexY);
    for(int i=0;i<numOfVertices;i++)
    {
        if(this->polygonTools.isPoint2DInsideConvexPolygon2D(this->vertexX.at(i),this->vertexY.at(i),stairVertexBuffer,numOfVertices,clockwiseOrdered))
            return true;
    }

    return false;
}

bool StepConstraintCheck::isAnyVertexOfFootInsideStairRegion(DiscreteFootstep stepToCheck, ljh::mathlib::ConvexPolygon2D stairPolygon)
{
    return this->isAnyVertexOfFootInsideStairRegion(stepToCheck.getX(),stepToCheck.getY(),stepToCheck.getYaw(),stepToCheck.getRobotSide().getStepFlag(),
                                                    stairPolygon.getVertexBuffer(),stairPolygon.getNumOfVertices(),stairPolygon.getClockwiseOrder());
}

bool StepConstraintCheck::isTwoFootCollided(double stanceX, double stanceY, double stanceYaw, enum StepFlag stanceFlag,
                        double swingX,  double swingY,  double swingYaw,  enum StepFlag swingFlag)
{   
    //calculate and load the vertex2d(in clockwiseorder) of the stanceStep as polygon
    this->stepPose.setPosition(stanceX,stanceY);
    this->stepPose.setOrientation(stanceYaw);
    getFootVertex2D(this->stepPose,stanceFlag,this->vertexX,this->vertexY);

    this->stanceBuffer.resize(4);
    for(int i=0;i<4;i++)
    {
        this->vertex.setPoint2D(this->vertexX.at(i),this->vertexY.at(i));
        this->stanceBuffer[i] = this->vertex;
    }
    // calculate the vertex2d of the swingStep
    this->stepPose.setPosition(swingX,swingY);
    this->stepPose.setOrientation(swingYaw);
    getFootVertex2D(this->stepPose,swingFlag,this->vertexX,this->vertexY);

    // check each vertex of swingStep Whether in stanceStep polygon
    for(int i=0;i<4;i++)
    {
        // getFootVertex2D would make an clockwise order
        if(this->polygonTools.isPoint2DInsideConvexPolygon2D(this->vertexX.at(0),this->vertexY.at(0),this->stanceBuffer,4,1))
            return true;
    }

    return false;



}   

bool StepConstraintCheck::isTwoFootCollided(DiscreteFootstep stanceStep, DiscreteFootstep swingStep)
{
    return  this->isTwoFootCollided(stanceStep.getX(),stanceStep.getY(),stanceStep.getYaw(),stanceStep.getRobotSide().getStepFlag(),
                                    swingStep.getX() ,swingStep.getY() ,swingStep.getYaw() ,swingStep.getRobotSide().getStepFlag()); 
}

bool StepConstraintCheck::isTwoFootCollided(FootstepGraphNode nodeToCheck)
{
    return this->isTwoFootCollided(nodeToCheck.getFirstStep(),nodeToCheck.getSecondStep());
}

_FOOTSTEP_PLANNER_END