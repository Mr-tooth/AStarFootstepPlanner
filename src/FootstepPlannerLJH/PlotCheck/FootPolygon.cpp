
#include <FootstepPlannerLJH/PlotCheck/FootPolygon.h>
_FOOTSTEP_PLANNER_BEGIN
double MidVertexX[4] = {0.0};
double MidVertexY[4] = {0.0};
/**
 * Input:
 *  CenterPose2D: FootCenter Param X(m),Y(m),Yaw(rad)
 *  StepFlag: 0L 1R 
 * Output:
 *  Vertex of X (1*4) in World Frame
 *  Vertex of Y (1*4) in World Frame
 * 
 *  Local Frame of Foot
 * 
 *         / \ Y (Left)
 *          |  
 * 3——————————————————0
 * |        |_________|___\ X (Foward)
 * |                  |   / 
 * 2——————————————————1
 */
void getFootVertex2D(double *CenterPose2D, int stepflag, double *VertexX, double *VertexY)
{
    // load coordinates in local frame of foot
    MidVertexX[0] = ForwardLength;
    MidVertexX[1] = MidVertexX[0];
    MidVertexX[2] = - BackwardLength;
    MidVertexX[3] = MidVertexX[2];

    if(stepflag) //step is Right
    {
        MidVertexY[0] = NarrowWidth;
        MidVertexY[1] = - WideWidth;
        
    }
    else //step is Left
    {
        MidVertexY[0] = WideWidth;
        MidVertexY[1] = - NarrowWidth;
    }

    MidVertexY[2] = MidVertexY[1];
    MidVertexY[3] = MidVertexY[0];

 
    // GetRotate
    for(int i=0;i<4;i++)
    {
        VertexX[i] = CenterPose2D[0] + cos(CenterPose2D[2]) * MidVertexX[i] - sin(CenterPose2D[2]) * MidVertexY[i];
        VertexY[i] = CenterPose2D[1] + sin(CenterPose2D[2]) * MidVertexX[i] + cos(CenterPose2D[2]) * MidVertexY[i];
    }

    // Clear
    for(int i=0;i<4;i++)
    {
        MidVertexX[i]=0.0;
        MidVertexY[i]=0.0;
    }
    

    

}

void getFootVertex2D(ljh::mathlib::Pose2D<double> _footPose, enum StepFlag _stepflag, std::vector<double>& _vertexX,std::vector<double>& _vertexY)
{
    double centerPose[3] = {_footPose.getPosition().getX(), _footPose.getPosition().getY(), _footPose.getOrientation().getYaw()};
    double vertexX[4] = {0.0};
    double vertexY[4] = {0.0};
    getFootVertex2D(centerPose, _stepflag, vertexX, vertexY);

    // check the size of vectors
    //if(_vertexX.size() < 4)
        _vertexX.resize(4); 

    //if(_vertexY.size() < 4)
        _vertexY.resize(4); 

    // load
    std::copy(vertexX,vertexX+4,_vertexX.begin());
    std::copy(vertexY,vertexY+4,_vertexY.begin());

    _vertexX.push_back(_vertexX.at(0));
    _vertexY.push_back(_vertexY.at(0));
 
}

void getFootVertex2D(Location _footNode, std::vector<double>& _vertexX,std::vector<double>& _vertexY)
{
    ljh::mathlib::Pose2D<double> _footPose(_footNode.getSecondStep().getX(), _footNode.getSecondStep().getY(), _footNode.getSecondStep().getYaw());
    getFootVertex2D(_footPose,_footNode.getSecondStepSide().getStepFlag(),_vertexX, _vertexY);
}


void getExtendedFootVertex2D(double *CenterPose2D, int stepflag, double *VertexX, double *VertexY, double extendLength)
{
    // load coordinates in local frame of foot
    MidVertexX[0] = (ForwardLength+extendLength);
    MidVertexX[1] = MidVertexX[0];
    MidVertexX[2] = - (BackwardLength+extendLength);
    MidVertexX[3] = MidVertexX[2];

    if(stepflag) //step is Right
    {
        MidVertexY[0] = (NarrowWidth+extendLength);
        MidVertexY[1] = - (WideWidth+extendLength);
        
    }
    else //step is Left
    {
        MidVertexY[0] = (WideWidth+extendLength);
        MidVertexY[1] = - (NarrowWidth+extendLength);
    }

    MidVertexY[2] = MidVertexY[1];
    MidVertexY[3] = MidVertexY[0];

 
    // GetRotate
    for(int i=0;i<4;i++)
    {
        VertexX[i] = CenterPose2D[0] + cos(CenterPose2D[2]) * MidVertexX[i] - sin(CenterPose2D[2]) * MidVertexY[i];
        VertexY[i] = CenterPose2D[1] + sin(CenterPose2D[2]) * MidVertexX[i] + cos(CenterPose2D[2]) * MidVertexY[i];
    }

    // Clear
    for(int i=0;i<4;i++)
    {
        MidVertexX[i]=0.0;
        MidVertexY[i]=0.0;
    }
    
}


void getExtendedFootVertex2D(ljh::mathlib::Pose2D<double> _footPose, enum StepFlag _stepflag, std::vector<double>& _vertexX,std::vector<double>& _vertexY, double extendLength)
{
    double centerPose[3] = {_footPose.getPosition().getX(), _footPose.getPosition().getY(), _footPose.getOrientation().getYaw()};
    double vertexX[4] = {0.0};
    double vertexY[4] = {0.0};
    getExtendedFootVertex2D(centerPose, _stepflag, vertexX, vertexY, extendLength);

    // check the size of vectors
    //if(_vertexX.size() < 4)
        _vertexX.resize(4); 

    //if(_vertexY.size() < 4)
        _vertexY.resize(4); 

    // load
    std::copy(vertexX,vertexX+4,_vertexX.begin());
    std::copy(vertexY,vertexY+4,_vertexY.begin());

    _vertexX.push_back(_vertexX.at(0));
    _vertexY.push_back(_vertexY.at(0));
}

void getExtendedFootVertex2D(Location _footNode, std::vector<double>& _vertexX,std::vector<double>& _vertexY, double extendLength)
{
    ljh::mathlib::Pose2D<double> _footPose(_footNode.getSecondStep().getX(), _footNode.getSecondStep().getY(), _footNode.getSecondStep().getYaw());
    getExtendedFootVertex2D(_footPose,_footNode.getSecondStepSide().getStepFlag(),_vertexX, _vertexY, extendLength);
}



_FOOTSTEP_PLANNER_END