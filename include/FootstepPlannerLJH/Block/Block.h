
// using in ubuntu with Dr.Chao

#pragma once
// #ifndef __LJH__FOOTSTEP__PLANNER
// #define __LJH__FOOTSTEP__PLANNER
#include <LBlocks/LBlocks.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <FootstepPlannerLJH/Title.h>
#include <FootstepPlannerLJH/DataType.h>
#include <FootstepPlannerLJH/AStarFootstepPlanner.h>

#include <iostream>
#include <string>
#include <vector>

_FOOTSTEP_PLANNER_BEGIN

enum{Cuboid,Cuboid0,Cuboid1,Cuboid2,Cuboid3,Cuboid4,Cuboid5,Cuboid6,Cuboid7};


template<const int N = 4>
struct PlaneData{
    double vertex[2][N];
    bool clockwiseOrdered;
    int numOfVertices;

    // for midpoint
    int alignEdgeEndpoints[2];      
    double alignEdgeMidPoint[2];
    double centerPose[3];
    double unitNormalVector[2];
};

struct Input{
    const int *KeyPress;
    const double *BodyPos;
    const double *BodyAng;
    const double *FootPosL;
    const double *FootAngL;
    const double *FootPosR;
    const double *FootAngR;
    const int    *SupFlag;
    PlaneData<> *planeData;
};

struct Output{
    std::vector<Location> FootholdNodeList;
};

class Block:public lee::blocks::LBlock<Input,Output>
{
protected:
    AStarFootstepPlanner footstepPlanner;
    double stepTime;
    bool simulateFlag;
    ljh::mathlib::Pose2D<double> goalPose2D;
    ljh::mathlib::Pose3D<double> goalPose3D;
    ljh::mathlib::Pose3D<double> startPose3D;

    bool doSearchFlag;
    bool plotFlag;
    PlotChecker pltChecker;
    int simStairNum;
    std::vector<ljh::mathlib::Point2D<double> > stairBuffer;
    //parameters param;
public:
    Block():footstepPlanner(),stepTime(0.8),simulateFlag(true),goalPose2D(),goalPose3D(),startPose3D(),doSearchFlag(false),plotFlag(true), pltChecker(),simStairNum(0),stairBuffer()
        {std::cout<<"Create Block: FootstepPlanner"<<std::endl;};
    //~Block();
    int init();
    int run();
    int print();
    int clear();

    void setIP(const char *_IP);
    void setPort(const int &_Port);
    void setPlotFlag(const bool &_flag){this->plotFlag = _flag;};
    void setStepTime(const double &_stepTime){this->stepTime = _stepTime;};
    void setDoSearchFlag(const bool &_flag){this->doSearchFlag = _flag;};

    // for simulation first
    void getPlaneData(); 
    void getGoalPosefromPlane(int _alignEdgeEndpoints[2]);
    void getStartPose();

    //void checkAndResetPlaneData();
    
    void transformPlaneData2StairBuffer();
    void transformWorldpose2Local();
    void transformWorldStairCenter2Local(double *centerPose);


};


_FOOTSTEP_PLANNER_END





