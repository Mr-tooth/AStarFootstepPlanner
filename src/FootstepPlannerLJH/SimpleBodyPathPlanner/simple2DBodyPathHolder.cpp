#include <FootstepPlannerLJH/SimpleBodyPathPlanner/simple2DBodyPathHolder.h>

_FOOTSTEP_PLANNER_BEGIN
void Simple2DBodyPathHolder::initialize(ljh::mathlib::Pose2D<double> _startPose,ljh::mathlib::Pose2D<double> _goalPose)                                                                
{
    this->startPose = _startPose;
    this->goalPose = _goalPose;

    this->x0 = this->startPose.getPosition().getX();
    this->y0 = this->startPose.getPosition().getY();

    this->calculatePathfromTwoPose();
    this->calculateWayPointsfromPath();
}

void Simple2DBodyPathHolder::calculatePathfromTwoPose()
{
    // ONLY consider two branch
    // forwardLeft and forwardRight
    // dx>0

    // get a and b from the relative location of goalpose respect to startpose
    this->a = (this->goalPose.getPosition().getX()-this->startPose.getPosition().getX());
    this->b = (this->goalPose.getPosition().getY()-this->startPose.getPosition().getY());

    // the sign of x/y in quarter ellipsoid
    if(a>0)
    {
        this->maxX = a;
        this->minX = 0.0;

        this->xc = this->startPose.getPosition().getX();
        this->yc = b;
    }
    else
    {
        this->maxX = 0.0;
        this->minX = a;
    }
    if(b>0)
    {
        this->maxY = b;
        this->minY = 0.0;
    }
    else
    {
        this->maxY = 0.0;
        this->minY = b;
    }
   

    this->a = std::abs(this->a);
    this->b = std::abs(this->b);
}

void Simple2DBodyPathHolder::calculateWayPointsfromPath()
{
    this->dense = 100;
    double dY;
    if(this->minY<-1e-5)// maxY->minY
        dY = (this->minY-this->maxY)/(this->dense*1.0);
    else// minY->maxY
        dY = (this->maxY-this->minY)/(this->dense*1.0);
    double X = this->x0;
    double Y = this->y0;  //we hope the order is from start to theleft/rightside
    double Yaw = 0.0;
    this->wayPointsAlongPath.clear();
    this->wayPointsAlongPath.push_back({X,Y,Yaw});
    for(int i=0;i<this->dense-1;i++)
    {
        Y += dY;
        X = this->xc + std::sqrt(std::pow(this->a,2)*(1-std::pow(Y-this->yc,2)/std::pow(this->b,2)));
        Yaw = std::atan(-std::pow(this->b,2)*(X-this->xc)/(std::pow(this->a,2)*(Y-this->yc)));
        this->wayPointsAlongPath.push_back({X,Y,Yaw});
    }
}

double Simple2DBodyPathHolder::getClosestdPointsYawfromPathToGivenPoint(double x, double y, PointFromPathInfo& pfp)
{
    int closestPointIndex = 0;
    double closestPointDistance = 1000.0;
    double X = 0.0;
    double Y = 0.0;
    this->distance = 0.0;
    for(int i=0;i<this->dense-1;i++)
    {
        X = this->wayPointsAlongPath.at(i).getPosition().getX();
        Y = this->wayPointsAlongPath.at(i).getPosition().getY();
        this->distance = (X-x) * (X-x) + (Y-y) * (Y-y);
        if(distance<closestPointDistance)
        {
           closestPointDistance = distance;
           closestPointIndex = i; 
        }  
    }

    pfp.distance = closestPointDistance;
    pfp.index = closestPointIndex;
    pfp.yaw = this->wayPointsAlongPath.at(closestPointIndex).getOrientation().getYaw();
    return pfp.yaw;   
}

double Simple2DBodyPathHolder::getClosestdPointsYawfromPathToGivenPoint(ljh::mathlib::Point2D<double> point, PointFromPathInfo& pfp)
{
    return this->getClosestdPointsYawfromPathToGivenPoint(point.getX(),point.getY(),pfp);
}
double Simple2DBodyPathHolder::getClosestdPointsYawfromPathToGivenPoint(FootstepGraphNode node, PointFromPathInfo& pfp)
{
    double x = node.getSecondStep().getX();
    double y = node.getSecondStep().getY();
    return this->getClosestdPointsYawfromPathToGivenPoint(x,y,pfp);
}


void Simple2DBodyPathHolder::plotEllipsoidPath()
{
    namespace plt = matplotlibcpp;
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> yaw;
    double arrowlength = 0.005;
    for(int i=0;i<this->wayPointsAlongPath.size();i++)
    {
        x.push_back(this->wayPointsAlongPath.at(i).getPosition().getX());
        y.push_back(this->wayPointsAlongPath.at(i).getPosition().getY());
        yaw.push_back(this->wayPointsAlongPath.at(i).getOrientation().getYaw());
    }
    plt::figure(1);
    plt::clf();
    plt::plot(x,y);
    for(int i=0;i<this->wayPointsAlongPath.size();i+=10)
    {
        plt::arrow(x.at(i),y.at(i),cos(yaw.at(i))*arrowlength,sin(yaw.at(i))*arrowlength,"r","k",0.002,0.001);
    }
    plt::set_aspect_equal();
    plt::show();
    
}


std::vector<ljh::mathlib::Pose2D<double> > Simple2DBodyPathHolder::getWayPointPath()
{
    return this->wayPointsAlongPath;
}
_FOOTSTEP_PLANNER_END