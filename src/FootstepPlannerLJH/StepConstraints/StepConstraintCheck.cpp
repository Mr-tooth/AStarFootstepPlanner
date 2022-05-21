
#include <FootstepPlannerLJH/StepConstraints/StepConstraintCheck.h>

_FOOTSTEP_PLANNER_BEGIN

void StepConstraintCheck::initialize()
{
    this->param = parameters();
    //this->polygonTools = ljh::mathlib::HeuclidGeometryPolygonTools();
    ljh::mathlib::Pose2D<double> Res;
    this->stepPose = Res;
    this->vertexX.clear();
    this->vertexY.clear();
    this->vertexX8.clear();
    this->vertexY8.clear();
    this->vertex.setPoint2D(0.0,0.0);
    this->stanceBuffer.clear();
}


bool StepConstraintCheck::isAnyVertexOfFootInsideStairRegion(double stepX, double stepY, double stepYaw, enum StepFlag stepFlag,
                                            std::vector<Point2D<double> > stairVertexBuffer, int numOfVertices, bool clockwiseOrdered)
{
    this->stepPose.setPosition(stepX,stepY);
    this->stepPose.setOrientation(stepYaw);
    getExtendedFootVertex2D(this->stepPose,stepFlag,this->vertexX,this->vertexY,0.0);
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
    getExtendedFootVertex2D(this->stepPose,stanceFlag,this->vertexX,this->vertexY,this->param.footPolygonExtendedLength);

    this->stanceBuffer.resize(4);
    for(int i=0;i<4;i++)
    {
        this->vertex.setPoint2D(this->vertexX.at(i),this->vertexY.at(i));
        this->stanceBuffer[i] = this->vertex;
    }
    // calculate the vertex2d of the swingStep
    this->stepPose.setPosition(swingX,swingY);
    this->stepPose.setOrientation(swingYaw);
    getExtendedFootVertex2D(this->stepPose,swingFlag,this->vertexX,this->vertexY,this->param.footPolygonExtendedLength);
    // Four Vertex is not enough, need Eight Vertices
    //this->vertexX8.clear(); this->vertexY8.clear();

    // check each vertex of swingStep Whether in stanceStep polygon
    for(int i=0;i<4;i++)
    {
        // getFootVertex2D would make an clockwise order
        if(this->polygonTools.isPoint2DInsideConvexPolygon2D(this->vertexX.at(i),this->vertexY.at(i),this->stanceBuffer,4,1))
            return true;
    }
    // check 4 middle points of edge
    for(int i=0;i<4;i++)
    {
        if(this->polygonTools.isPoint2DInsideConvexPolygon2D(0.5*(this->vertexX.at(i)+this->vertexX.at((i+1)%4)),0.5*(this->vertexY.at(i)+this->vertexY.at((i+1)%4)),this->stanceBuffer,4,1))
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