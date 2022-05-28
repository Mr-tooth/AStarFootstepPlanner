
#include <FootstepPlannerLJH/Block/Block.h>

_FOOTSTEP_PLANNER_BEGIN
namespace internal
{
    LatticePoint latticepoint;
    parameters param;

    std::string IP = "127.0.0.1";
    int Port = 3333;
    
}


int Block::init()
{
    // initialize the parameters of footstepPlanner
    internal::latticepoint.setGridSizeXY(internal::latticepoint,0.01);
    internal::latticepoint.setYawDivision(internal::latticepoint, 72);

    internal::param.SetEdgeCostDistance(internal::param,4.0);
    internal::param.SetEdgeCostYaw(internal::param, 4.0);
    internal::param.SetEdgeCostStaticPerStep(internal::param,1.4);
    internal::param.SetDebugFlag(internal::param,false);
    internal::param.SetMaxStepYaw(internal::param,pi/8);
    internal::param.SetMinStepYaw(internal::param,-pi/8);        

    internal::param.SetFinalTurnProximity(internal::param,0.3);
    internal::param.SetGoalDistanceProximity(internal::param,0.008);
    internal::param.SetGoalYawProximity(internal::param,2.0/180.0 * pi);
    internal::param.SetFootPolygonExtendedLength(internal::param,0.03);
    
    internal::param.SetStairAlignMode(internal::param,true);
    //param.SetStairPolygon(param,stairBuffer,4,1);
    return 0;
}

int Block::run()
{
    // only run once with doSearchFlag
    if(*this->DataInput.KeyPress== '3'&&this->simulateFlag&& !this->doSearchFlag)
    {   
        this->doSearchFlag = true;
        this->getStartPose();
        this->getPlaneData();
        this->getGoalPosefromPlane(this->DataInput.planeData->alignEdgeEndpoints);
        
        //this->transformWorldpose2Local();
        this->footstepPlanner.initialize(this->goalPose2D,this->goalPose3D,this->startPose3D);
        this->footstepPlanner.doAStarSearch();
        this->footstepPlanner.calFootstepSeries();
        this->DataOutput.FootholdNodeList = this->footstepPlanner.getFootstepSeries();

        if(this->plotFlag) 
            this->pltChecker.plotSearchOutcome2(this->DataOutput.FootholdNodeList,this->goalPose3D,this->startPose3D);
    }

    if(*this->DataInput.KeyPress=='z'||*this->DataInput.KeyPress=='Z')
        this->setDoSearchFlag(false);

    return 0;
}

int Block::print()
{
    auto &Step = this->DataOutput.FootholdNodeList;
    std::cout << "[FootstepPlanner]" << std::endl;
    for (int i = 0; i < Step.size(); i++)
    {
        std::cout <<"      "<< "("
                  << Step[i].getSecondStep().getRobotSide().getStepFlag() << ", "
                  << Step[i].getSecondStep().getX() << ", "
                  << Step[i].getSecondStep().getY() << ", "
                  << Step[i].getSecondStep().getYaw()*57.32<< "), ";
                  
        if ((i + 1) % 2 == 0)
            std::cout << std::endl
                      << "      ";
    }
    std::cout << std::endl;
    return 0;
}

int Block::clear()
{

    return 0;
}

//  lowb version getdata directly ,and once move once rewrite
void Block::getPlaneData()
{
    double lenXY[2];
    double centerPose[3];
    double MidVertexX[4] = {0.0};
    double MidVertexY[4] = {0.0};

    switch (this->simStairNum)
    {
    case Cuboid: //x 1m y 1m z 0.12m
        lenXY[0]=1;
        lenXY[1]=1;
        centerPose[0]=1.5637;
        centerPose[1]=0.38141;
        centerPose[2]=0.0;
        this->transformWorldStairCenter2Local(centerPose);
        for(int i=0;i<3;i++)
            this->DataInput.planeData->centerPose[i] = centerPose[i];
        // load coordinates in local frame of foot
        MidVertexX[0] = lenXY[0]/2.0;
        MidVertexX[1] = MidVertexX[0];
        MidVertexX[2] = -lenXY[0]/2.0;
        MidVertexX[3] = MidVertexX[2];

        MidVertexY[0] = lenXY[1]/2.0;
        MidVertexY[1] = -lenXY[1]/2.0;
        MidVertexY[2] = MidVertexY[1];
        MidVertexY[3] = MidVertexY[0];

        for(int i=0;i<4;i++)
        {
            this->DataInput.planeData->vertex[0][i] = centerPose[0] + cos(centerPose[2]) * MidVertexX[i] - sin(centerPose[2]) * MidVertexY[i];
            this->DataInput.planeData->vertex[1][i] = centerPose[1] + sin(centerPose[2]) * MidVertexX[i] + cos(centerPose[2]) * MidVertexY[i];
        }
        this->DataInput.planeData->numOfVertices = 4;
        this->DataInput.planeData->clockwiseOrdered = true;

        this->transformPlaneData2StairBuffer();

        this->DataInput.planeData->alignEdgeEndpoints[0] = 2;
        this->DataInput.planeData->alignEdgeEndpoints[1] = 3;

        break;
    
    default:
        break;
    }

    internal::param.SetStairPolygon(internal::param,this->stairBuffer,this->DataInput.planeData->numOfVertices,1);
}

void Block::getGoalPosefromPlane(int _endpointsNum[2])
{
    auto midPoints = this->DataInput.planeData->alignEdgeMidPoint;
    auto normVector = this->DataInput.planeData->unitNormalVector;
    auto center = this->DataInput.planeData->centerPose;

    double goalFromStairEdge = 0.16+0.005;

    for(int i=0;i<2;i++)
        midPoints[i] = (this->DataInput.planeData->vertex[i][_endpointsNum[0]] + this->DataInput.planeData->vertex[i][_endpointsNum[1]])/2.0;
    
    for(int i=0;i<2;i++)
        normVector[i] = midPoints[i]-center[i];
    
    double length = std::sqrt(std::pow(normVector[0],2) + std::pow(normVector[1],2));

    for(int i=0;i<2;i++)
        normVector[i] = normVector[i]/length;

    this->goalPose2D.setX(midPoints[0] + goalFromStairEdge * normVector[0]);
    this->goalPose2D.setY(midPoints[1] + goalFromStairEdge * normVector[1]);

    this->goalPose3D.setX(this->goalPose2D.getPosition().getX());
    this->goalPose3D.setY(this->goalPose2D.getPosition().getY());
    this->goalPose3D.setYawPitchRoll(center[2],0.0,0.0);

}

void Block::getStartPose()
{
    switch (this->simStairNum)
    {
    case Cuboid:
        this->startPose3D.setX(-0.38594);
        this->startPose3D.setY( 0.5);
        this->startPose3D.setZ( 0.0);
        this->startPose3D.setYawPitchRoll(0.0,0.0,0.0);
        break;
    
    default:
        break;
    }
}


void Block::transformPlaneData2StairBuffer()
{
    this->stairBuffer.clear();
    for(int i=0;i<this->DataInput.planeData->numOfVertices;i++)
    {
        this->stairBuffer.push_back({this->DataInput.planeData->vertex[0][i],
                                     this->DataInput.planeData->vertex[1][i]});
    }
}

// if in normal mode, turn the goalPose into start local frame
void Block::transformWorldpose2Local()
{
    using Eigen::Matrix4d;
    using Eigen::Matrix3d;
    Matrix4d W_T_Start = Matrix4d::Identity(); 
    Matrix4d W_T_Goal  = Matrix4d::Identity();
    Matrix4d Start_T_Goal = Matrix4d::Identity();

    double yaw = this->startPose3D.getOrientation().getYaw();
    double pitch = this->startPose3D.getOrientation().getPitch();
    double roll = this->startPose3D.getOrientation().getRoll();

    auto Rzyx=[](double _yaw, double _pitch,double _roll)
    {
        Matrix3d Rzyx = (Eigen::AngleAxisd(_yaw,Eigen::Vector3d::UnitZ())*
                        Eigen::AngleAxisd(_pitch,Eigen::Vector3d::UnitY())*
                        Eigen::AngleAxisd(_roll,Eigen::Vector3d::UnitX())).matrix();
        return Rzyx;
    };

    W_T_Start.block<3,3>(0,0)=Rzyx(this->startPose3D.getOrientation().getYaw(),
                                   this->startPose3D.getOrientation().getPitch(),
                                   this->startPose3D.getOrientation().getRoll());
    
    W_T_Start(0,3) = this->startPose3D.getPosition().getX();
    W_T_Start(1,3) = this->startPose3D.getPosition().getY();
    W_T_Start(2,3) = this->startPose3D.getPosition().getZ();

    W_T_Goal.block<3,3>(0,0) =Rzyx(this->goalPose3D.getOrientation().getYaw(),
                                   this->goalPose3D.getOrientation().getPitch(),
                                   this->goalPose3D.getOrientation().getRoll());

    W_T_Goal(0,3) = this->goalPose3D.getPosition().getX();
    W_T_Goal(1,3) = this->goalPose3D.getPosition().getY();
    W_T_Goal(2,3) = this->goalPose3D.getPosition().getZ();

    Start_T_Goal = W_T_Start.inverse()*W_T_Goal;

    this->startPose3D = Pose3D<double> ();

    // it's not that stable to use eulerAngles!
    Eigen::Vector3d eulerAngle_Start_T_Goal = Start_T_Goal.block<3,3>(0,0).eulerAngles(2,1,0);
    this->goalPose3D.setX(Start_T_Goal(0,3));
    this->goalPose3D.setY(Start_T_Goal(1,3));
    this->goalPose3D.setZ(Start_T_Goal(2,3));
    this->goalPose3D.setYawPitchRoll(eulerAngle_Start_T_Goal(0),eulerAngle_Start_T_Goal(1),eulerAngle_Start_T_Goal(2));

    this->goalPose2D.setX(Start_T_Goal(0,3));
    this->goalPose2D.setY(Start_T_Goal(1,3));
    this->goalPose2D.setYaw(eulerAngle_Start_T_Goal(0));
}


// if in StairAlignMode, turn the stairbuffer, goalpose into start local frame
void Block::transformWorldStairCenter2Local(double *centerPose)
{
    Eigen::Matrix4d W_T_Start = Eigen::Matrix4d::Identity(); 
    Eigen::Matrix4d W_T_StairCenter = Eigen::Matrix4d::Identity(); 
    Eigen::Matrix4d Start_T_StairCenter = Eigen::Matrix4d::Identity(); 
    Eigen::Vector3d out;

    auto Rzyx=[](double _yaw, double _pitch,double _roll)
    {
        Eigen::Matrix3d Rzyx = (Eigen::AngleAxisd(_yaw,Eigen::Vector3d::UnitZ())*
                                Eigen::AngleAxisd(_pitch,Eigen::Vector3d::UnitY())*
                                Eigen::AngleAxisd(_roll,Eigen::Vector3d::UnitX())).matrix();
        return Rzyx;
    };

    W_T_Start.block<3,3>(0,0)=Rzyx(this->startPose3D.getOrientation().getYaw(),
                                   this->startPose3D.getOrientation().getPitch(),
                                   this->startPose3D.getOrientation().getRoll());

    W_T_Start(0,3) = this->startPose3D.getPosition().getX();
    W_T_Start(1,3) = this->startPose3D.getPosition().getY();
    W_T_Start(2,3) = this->startPose3D.getPosition().getZ();

    W_T_StairCenter.block<3,3>(0,0)=Rzyx(centerPose[2],0.0,0.0);
    for(int i=0;i<2;i++)
        W_T_StairCenter(i,3) = centerPose[i];

    Start_T_StairCenter = W_T_Start.inverse()*W_T_StairCenter;
    for(int i=0;i<2;i++)
        centerPose[i] = Start_T_StairCenter(i,3) ;
    
    centerPose[2] = Start_T_StairCenter.block<3,3>(0,0).eulerAngles(2,1,0)[0];

    this->startPose3D = ljh::mathlib::Pose3D<double> ();
}

void Block::setIP(const char *_IP)
{
    internal::IP = _IP;
}

void Block::setPort(const int &_Port)
{
    internal::Port = _Port;
}
_FOOTSTEP_PLANNER_END