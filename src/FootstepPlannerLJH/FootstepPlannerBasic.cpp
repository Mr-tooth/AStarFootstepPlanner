
#include <FootstepPlannerLJH/FootstepplannerBasic.h>

_FOOTSTEP_PLANNER_BEGIN

double LatticePoint::gridSizeXY = 0.02 ; //m;
int LatticePoint::yawDivision = 72;
double LatticePoint::gridSizeYaw = 2.0*pi_f/LatticePoint::yawDivision;

LatticePoint::LatticePoint(double x, double y, double yaw)
{
    this->xIndex   = int(x/this->gridSizeXY);
    this->yIndex   = int(y/this->gridSizeXY);
    this->yawIndex = int(yaw/this->gridSizeYaw);
}

LatticePoint::LatticePoint(int _x, int _y, int _yaw)
{
    this->xIndex = _x;
    this->yIndex = _y;
    this->yawIndex = _yaw;
}

LatticePoint::LatticePoint(const LatticePoint& _latticepoint)
{
    this->xIndex = _latticepoint.xIndex;
    this->yIndex = _latticepoint.yIndex;
    this->yawIndex = _latticepoint.yawIndex;
}

bool LatticePoint::operator==(const LatticePoint& cmp_point) const
{
    return (cmp_point.xIndex == this->xIndex &&
            cmp_point.yIndex == this->yIndex && 
            cmp_point.yawIndex == this->yawIndex);
}

void LatticePoint::operator=(const LatticePoint& cmp_point)
{
    this->xIndex = cmp_point.xIndex;
    this->yIndex = cmp_point.yIndex;
    this->yawIndex = cmp_point.yawIndex;
}

int RobotSide::hashcode()
{
    int res = 0;
    switch (this->stepflag)
    {
    case stepFly:
        break;
    case stepL:
        res = 2;
        break;
    case stepR:
        res = 3;
        break;
    };

    return res;
}

double LatticePoint:: getGridSizeXY(const LatticePoint& other)
{
    return other.gridSizeXY;
}

double LatticePoint:: getGridSizeYaw(const LatticePoint& other)
{
    return other.gridSizeYaw;
}

void LatticePoint:: setGridSizeXY(LatticePoint& other, double sizeXY)
{
    other.gridSizeXY = sizeXY;
}

void LatticePoint:: setYawDivision(LatticePoint& other, int yawDivision)
{
    other.yawDivision = yawDivision;
    other.gridSizeYaw = 2.0*pi_f/yawDivision;
}

DiscreteFootstep::DiscreteFootstep(double _x, double _y)
{   
    this->latticepoint = LatticePoint(_x,_y,0.0);    
    this->hashcode = this->coumputeHashCode();
    this->computeMidFootPoint(IDEAL_STEP_WIDTH);
};

DiscreteFootstep::DiscreteFootstep(double _x, double _y, double _yaw, RobotSide _robotside)
{
    this->latticepoint = LatticePoint(_x,_y,_yaw);
    this->robotside = _robotside;
    this->hashcode = this->coumputeHashCode();
    this->computeMidFootPoint(IDEAL_STEP_WIDTH);
};

DiscreteFootstep::DiscreteFootstep(int _xindex, int _yindex, int _yawindex, RobotSide _robotside)
{
    this->latticepoint = LatticePoint(_xindex,_yindex,_yawindex);
    this->robotside = _robotside;
    this->hashcode = this->coumputeHashCode();
    this->computeMidFootPoint(IDEAL_STEP_WIDTH);
}

DiscreteFootstep::DiscreteFootstep( const LatticePoint& _latticepoint, const RobotSide& _robotside)
{
    this->latticepoint = _latticepoint;
    this->robotside = _robotside;
    this->hashcode = this->coumputeHashCode();
    this->computeMidFootPoint(IDEAL_STEP_WIDTH);
}

DiscreteFootstep::DiscreteFootstep(double _x, double _y, double _yaw, enum StepFlag _stepflag)
{
    this->latticepoint = LatticePoint(_x,_y,_yaw);
    this->robotside = RobotSide(_stepflag);
    this->hashcode = this->coumputeHashCode();
    this->computeMidFootPoint(IDEAL_STEP_WIDTH);
}

DiscreteFootstep::DiscreteFootstep(int _xindex, int _yindex, int _yawindex, enum StepFlag _stepflag)
{
    this->latticepoint = LatticePoint(_xindex,_yindex,_yawindex);
    this->robotside = RobotSide(_stepflag);
    this->hashcode = this->coumputeHashCode();
    this->computeMidFootPoint(IDEAL_STEP_WIDTH);
}


double DiscreteFootstep::euclideanPlaneDistanceSquared(const DiscreteFootstep& other) const
{
    double dx = this->getX()-other.getX();
    double dy = this->getY()-other.getY();
    return (dx*dx + dy*dy);
}

double DiscreteFootstep::euclideanPlaneDistance(const DiscreteFootstep& other) const
{
    return sqrt(this->euclideanPlaneDistanceSquared(other));
}

int DiscreteFootstep::computeYawIndexDistance(const DiscreteFootstep& other) const
{
    int dYaw = std::abs(this->latticepoint.getYawIndex()-other.getYawIndex());
    return min_ljh(dYaw,this->latticepoint.yawDivision-dYaw);
}

int DiscreteFootstep::computeXYManhattanDistance(const DiscreteFootstep& other) const
{
    int manhattandistance = std::abs(this->latticepoint.getXIndex()-other.latticepoint.getXIndex())
                           +std::abs(this->latticepoint.getYIndex()-other.latticepoint.getYIndex());
    
    return manhattandistance;
}

int DiscreteFootstep::computeManhattanDistance(const DiscreteFootstep& other) const
{
    return (this->computeXYManhattanDistance(other) + this->computeYawIndexDistance(other));
}

void DiscreteFootstep::computeMidFootPoint(const double& idealStepWidth)
{
    //double yaw = this->getLatticePoint().getYaw();
    double vx =  this->getRobotSide().negateIfRightSide(sin(this->getLatticePoint().getYaw())*idealStepWidth/2);
    double vy = -this->getRobotSide().negateIfRightSide(cos(this->getLatticePoint().getYaw())*idealStepWidth/2);

    this->midFootPoint.setX(this->latticepoint.getX() + vx);
    this->midFootPoint.setY(this->latticepoint.getY() + vy);
}

Point2D<double> DiscreteFootstep::getOrComputeMidFootPoint(const double& stepWidth)
{
    //if(this->midFootPoint.getX() == 0.0&&this->midFootPoint.getY() == 0.0)
        this->computeMidFootPoint(stepWidth);
    return this->midFootPoint;    
}

bool DiscreteFootstep::operator==(const DiscreteFootstep& other) const
{
    return (this->latticepoint==other.latticepoint&&
            this->robotside.getStepFlag()==other.robotside.getStepFlag()&&
            this->midFootPoint==other.midFootPoint);
}

void DiscreteFootstep::operator=(const DiscreteFootstep& other)
{
    this->latticepoint = other.latticepoint;
    this->robotside = other.robotside;
    this->midFootPoint = other.midFootPoint;
    this->hashcode = other.hashcode;
}

int DiscreteFootstep::coumputeHashCode()
{
    int prime = 31;
    int res = 1;
    res = prime * res + this->robotside.hashcode();
    res = prime * res + this->latticepoint.getXIndex();
    res = prime * res + this->latticepoint.getYIndex();
    res = prime * res + this->latticepoint.getYawIndex();

    return res;
}    





bool FootstepGraphNode::checkDifferentSide(DiscreteFootstep _firststep, DiscreteFootstep _secondstep)
{
    if(_firststep.getRobotSide().getStepFlag()==_secondstep.getRobotSide().getStepFlag())
        return false;
    return true;
}

Pose2D<double> FootstepGraphNode::getOrComputeMidFootPose()
{
    //if(this->midFootPose.epsilonZero(ZERO_TEST_EPSILON))
    //{
        this->midFootPose.setX(0.5*(firstStep.getX()+secondStep.getX()));
        this->midFootPose.setY(0.5*(firstStep.getY()+secondStep.getY()));
        this->midFootPose.setYaw(0.5*(firstStep.getYaw()+secondStep.getYaw()));
    //}

    return this->midFootPose ;
}

double FootstepGraphNode::getStanceAngle()
{   // return [-pi,pi]
    double difference = firstStep.getYaw()-secondStep.getYaw();
    difference = this->midFootPose.getOrientation().shiftProperYaw(difference);
    return std::abs(difference);
}

void FootstepGraphNode::setNode(DiscreteFootstep _firststep, DiscreteFootstep _secondstep)
{
    this->firstStep = _firststep;
    this->secondStep = _secondstep;
    this->midFootPose = this->getOrComputeMidFootPose();
}

void FootstepGraphNode::operator=(const FootstepGraphNode& other)
{
    this->firstStep = other.firstStep;
    this->secondStep = other.secondStep;
    this->midFootPose = other.midFootPose;
}

bool FootstepGraphNode::operator==(const FootstepGraphNode& other) const
{
    return(this->firstStep  == other.firstStep &&
           this->secondStep == other.secondStep&&
           this->midFootPose== other.midFootPose);

}


int FootstepGraphNode::computeHashCode()
{
    int prime = 31;
    int res = 1;
    res = prime * res + this->firstStep.hashCode();
    res = prime * res + this->secondStep.hashCode();
    return res; 
}

_FOOTSTEP_PLANNER_END