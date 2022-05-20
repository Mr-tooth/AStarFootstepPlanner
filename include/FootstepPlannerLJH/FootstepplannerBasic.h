#pragma once
#ifndef __FOOTSTEP__PLANNER__BASIC__
#define __FOOTSTEP__PLANNER__BASIC__


#define _FOOTSTEP_PLANNER_BEGIN namespace ljh{namespace path{namespace footstep_planner{
#define _FOOTSTEP_PLANNER_END }}}

#define IDEAL_STEP_WIDTH 0.16
#define pi_f 3.1415926535
#ifndef min_ljh
#define min_ljh(a,b)  (((a)<(b))?(a):(b))
#endif

#include <FootstepPlannerLJH/EnumDef.h>
#include <Heuclid/euclid/tuple2D/Point2D.h>
#include <Heuclid/geometry/Pose2D.h>
#include <cmath>
using ljh::mathlib::Point2D;
using ljh::mathlib::Pose2D;
_FOOTSTEP_PLANNER_BEGIN


class LatticePoint
{
private:
    int xIndex;
    int yIndex;
    int yawIndex;

public:
    // const double gridSizeXY = 0.02;
    // const int yawDivision = 72;
    // const double gridSizeYaw = 2.0*pi_f/yawDivision;
    static double gridSizeXY;
    static int yawDivision;
    static double gridSizeYaw;
    LatticePoint():xIndex(0),yIndex(0),yawIndex(0){};
    LatticePoint(double x, double y, double yaw);
    LatticePoint(int _x, int _y, int _yaw);
    LatticePoint(const LatticePoint& _latticepoint);
    inline int getXIndex() const {return this->xIndex;};
    inline int getYIndex() const {return this->yIndex;};
    inline int getYawIndex() const {return this->yawIndex;};
    inline double getX() const {return this->gridSizeXY * this->xIndex;};
    inline double getY() const {return this->gridSizeXY * this->yIndex;};
    inline double getYaw() const {return this->gridSizeYaw * this->yawIndex;};
    bool operator==(const LatticePoint& cmp_point) const;
    void operator=(const LatticePoint& cmp_point);

    double getGridSizeXY(const LatticePoint& other);
    double getGridSizeYaw(const LatticePoint& other);
    void setGridSizeXY(LatticePoint& other, double sizeXY);
    void setYawDivision(LatticePoint& other, int yawDivision);
};



class RobotSide
{
private:
    enum StepFlag stepflag;
public:
    inline enum StepFlag getStepFlag() const {return this->stepflag;};
    inline RobotSide getOppositeSide() const
    {
        RobotSide roside;
        if(this->stepflag ==stepL)
            roside.stepflag = stepR;
        else
            roside.stepflag = stepL;    
        
        return roside;

    }

    inline double negateIfLeftSide(double value) const
    {
        if(this->stepflag == stepL)
            return -value;
        return value;
    }

    inline double negateIfRightSide(double value) const
    {
        if(this->stepflag == stepR)
            return -value;
        return value;
    }

    RobotSide():stepflag(stepFly){};
    RobotSide(const RobotSide& _robotside){this->stepflag = _robotside.stepflag;};
    RobotSide(const enum StepFlag& _stepflag){this->stepflag = _stepflag;};

    inline void operator=(const RobotSide& other) {this->stepflag = other.stepflag;};

    int hashcode();
};


class DiscreteFootstep
{
private:
    LatticePoint latticepoint;
    RobotSide robotside;
    Point2D<double> midFootPoint;
    int hashcode;
public:
    DiscreteFootstep():latticepoint(),robotside(),midFootPoint(){};
    DiscreteFootstep(double _x, double _y);
    // :latticepoint(_x,_y,0.0),robotside()
    //     {this->hashcode = this->coumputeHashCode();};

    DiscreteFootstep(double _x, double _y, double _yaw, RobotSide _robotside);
    // :latticepoint(_x,_y,_yaw),robotside(_robotside)
    //     {this->hashcode = this->coumputeHashCode();};

    DiscreteFootstep(int _xindex, int _yindex, int _yawindex, RobotSide _robotside);
    // :latticepoint(_xindex,_yindex,_yawindex),robotside(_robotside)
    //     {this->hashcode = this->coumputeHashCode();};

    DiscreteFootstep( const LatticePoint& _latticepoint, const RobotSide& _robotside);
    // :latticepoint(_latticepoint),robotside(_robotside)
    //     {this->hashcode = this->coumputeHashCode();};

    DiscreteFootstep(double _x, double _y, double _yaw, enum StepFlag _stepflag);
    DiscreteFootstep(int _xindex, int _yindex, int _yawindex, enum StepFlag _stepflag);


    inline int getXIndex() const {return this->latticepoint.getXIndex();};
    inline int getYIndex() const {return this->latticepoint.getYIndex();};
    inline int getYawIndex() const {return this->latticepoint.getYawIndex();};
    
    inline double getX() const {return this->latticepoint.getX();};
    inline double getY() const {return this->latticepoint.getY();};
    inline double getYaw() const {return this->latticepoint.getYaw();};

    inline LatticePoint getLatticePoint() const {return this->latticepoint;};
    inline RobotSide getRobotSide() const {return this->robotside;};
    inline Point2D<double> getMidFootPoint() const {return this->midFootPoint;};

    double euclideanPlaneDistanceSquared(const DiscreteFootstep& other) const;
    double euclideanPlaneDistance(const DiscreteFootstep& other) const;

    int computeYawIndexDistance(const DiscreteFootstep& other) const;
    int computeXYManhattanDistance(const DiscreteFootstep& other) const;
    int computeManhattanDistance(const DiscreteFootstep& other) const;

    inline bool equalPosition(const DiscreteFootstep& other) const 
    {return (getXIndex()==other.getXIndex()&&getYIndex()==other.getYIndex());};

    void computeMidFootPoint(const double& idealStepWidth);
    Point2D<double> getOrComputeMidFootPoint(const double& stepWidth);

    bool operator==(const DiscreteFootstep& other) const;
    void operator=(const DiscreteFootstep& other);

    int coumputeHashCode();
    int hashCode() const{return this->hashcode;};
};



/**
 * This object represents a node on the graph search by the footstep planner.
 * A node is a robot "stance", i.e. a left footstep and right footstep.
 * An edge represents a step, i.e. a transition between two stances.
 */
class FootstepGraphNode
{
    //we would record both the swing and supporting(stance) footstep
private:
    /**
    * This is the start-of-swing step when stepping at this node. "First" refers to the step's sequencing
    * Note that this node's parent's secondStep will also be this step
    */
    DiscreteFootstep firstStep;
    /**
    * This is the stance step when stepping at this node. "Second" refers to the step's sequencing
    * Note that this node's children's firstStep will also be this step
    */
    DiscreteFootstep secondStep;

    Pose2D<double> midFootPose;
public:
    FootstepGraphNode():firstStep(),secondStep(),midFootPose(){};
    FootstepGraphNode(DiscreteFootstep _firststep, DiscreteFootstep _secondstep):firstStep(_firststep),secondStep(_secondstep),midFootPose()
        {this->midFootPose = this->getOrComputeMidFootPose() ;};
    FootstepGraphNode(const FootstepGraphNode& other):firstStep(other.firstStep),secondStep(other.secondStep),midFootPose(other.midFootPose){};

    bool checkDifferentSide(DiscreteFootstep _firststep, DiscreteFootstep _secondstep);

    DiscreteFootstep getFirstStep() const {return this->firstStep;};
    DiscreteFootstep getSecondStep() const {return this->secondStep;};

    RobotSide getFirstStepSide() const {return this->firstStep.getRobotSide();};
    RobotSide getSecondStepSide() const {return this->secondStep.getRobotSide();};

    Pose2D<double> getOrComputeMidFootPose();
    double getStanceAngle();

    //void operator=();
    void setNode(DiscreteFootstep _firststep, DiscreteFootstep _secondstep);
    void operator=(const FootstepGraphNode& other);
    bool operator==(const FootstepGraphNode& other) const;

    int computeHashCode();
};

struct FootNodeHash
{
    size_t operator() (const FootstepGraphNode& f) const
    {
        int prime = 31;
        int res = 1;
        res = prime * res + f.getFirstStep().coumputeHashCode();
        res = prime * res + f.getSecondStep().coumputeHashCode();
        return size_t(res); 
        //res = 
    }
};




_FOOTSTEP_PLANNER_END
#endif