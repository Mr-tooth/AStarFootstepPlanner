#pragma once
#include <Heuclid/geometry/Pose2D.h>
#include <Heuclid/geometry/Pose3D.h>
#include <FootstepPlannerLJH/Title.h>
#include <FootstepPlannerLJH/FootstepplannerBasic.h>
#include <vector>
#include <cmath>
#include <matplotlibcpp.h>
_FOOTSTEP_PLANNER_BEGIN

struct PointFromPathInfo
{
    int index;
    double distance;
    double yaw;
};

/**
 * @brief 
 * 
 */
class Simple2DBodyPathHolder
{
private:
    ljh::mathlib::Pose2D<double> startPose;
    ljh::mathlib::Pose2D<double> goalPose;
    std::vector<ljh::mathlib::Pose2D<double> > wayPointsAlongPath;

    double distance;
    double x0,y0;
    // the center of ellipsoid path
    // equation is (x-xc)^2/a^2 + (y-yc)^2/b^2 = 1
    double xc,yc;
    double a,b;
    double maxX,maxY;
    double minX,minY;
    int dense;
    

public:
    Simple2DBodyPathHolder():startPose(),goalPose(),wayPointsAlongPath(),distance(0.0),x0(0.0),y0(0.0),xc(0.0),yc(0.0),a(0.0),b(0.0),maxX(0.0),maxY(0.0),minX(0.0),minY(0.0),dense(0){};
    Simple2DBodyPathHolder(ljh::mathlib::Pose2D<double> _startPose,ljh::mathlib::Pose2D<double> _goalPose):startPose(_startPose),goalPose(_goalPose),
                                                    wayPointsAlongPath(),distance(0.0),x0(0.0),y0(0.0),xc(0.0),yc(0.0),a(0.0),b(0.0),maxX(0.0),maxY(0.0),minX(0.0),minY(0.0),dense(0){};

    void initialize(ljh::mathlib::Pose2D<double> _startPose,ljh::mathlib::Pose2D<double> _goalPose);                                                                                    
    void calculatePathfromTwoPose();
    void calculateWayPointsfromPath();
    double getClosestdPointsYawfromPathToGivenPoint(double x, double y, PointFromPathInfo& pfp);
    double getClosestdPointsYawfromPathToGivenPoint(ljh::mathlib::Point2D<double> point, PointFromPathInfo& pfp);
    double getClosestdPointsYawfromPathToGivenPoint(FootstepGraphNode node, PointFromPathInfo& pfp);

    void plotEllipsoidPath();
    std::vector<ljh::mathlib::Pose2D<double> > getWayPointPath();
};



_FOOTSTEP_PLANNER_END

