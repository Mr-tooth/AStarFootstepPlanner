#pragma once
#include <FootstepPlannerLJH/Title.h>
#include <FootstepPlannerLJH/parameters.h>
#include <FootstepPlannerLJH/EnumDef.h>
#include <FootstepPlannerLJH/FootstepplannerBasic.h>
#include <FootstepPlannerLJH/PlotCheck/FootPolygon.h>
#include <Heuclid/geometry/ConvexPolygon2D.h>
#include <Heuclid/geometry/tools/HeuclidPolygonTools.h>
#include <Heuclid/geometry/Pose2D.h>
#include <matplotlibcpp.h>
#include <vector>
_FOOTSTEP_PLANNER_BEGIN
class StepConstraintCheck
{
private:
    parameters param;
    ljh::mathlib::HeuclidGeometryPolygonTools polygonTools;

    ljh::mathlib::Pose2D<double> stepPose;
    std::vector<double> vertexX;
    std::vector<double> vertexY;
    std::vector<double> vertexX8;
    std::vector<double> vertexY8;

    ljh::mathlib::Point2D<double> vertex;
    std::vector<Point2D<double> > stanceBuffer;
public:
    StepConstraintCheck():param(),polygonTools(),stepPose(),vertexX(),vertexY(),vertexX8(),vertexY8(),vertex(),stanceBuffer(){};
    void initialize();
    bool isAnyVertexOfFootInsideStairRegion(double stepX, double stepY, double stepYaw, enum StepFlag stepFlag,
                                            std::vector<Point2D<double> > stairVertexBuffer, int numOfVertices, bool clockwiseOrdered);

    bool isAnyVertexOfFootInsideStairRegion(DiscreteFootstep stepToCheck, ljh::mathlib::ConvexPolygon2D stairPolygon);


    bool isTwoFootCollided(double stanceX, double stanceY, double stanceYaw, enum StepFlag stanceFlag,
                           double swingX,  double swingY,  double swingYaw,  enum StepFlag swingFlag);
    bool isTwoFootCollided(DiscreteFootstep stanceStep, DiscreteFootstep swingStep);
    bool isTwoFootCollided(FootstepGraphNode nodeToCheck);


    bool isTwoFootCollidedAndPlot(double stanceX, double stanceY, double stanceYaw, enum StepFlag stanceFlag,
                           double swingX,  double swingY,  double swingYaw,  enum StepFlag swingFlag);
    bool isTwoFootCollidedAndPlot(DiscreteFootstep stanceStep, DiscreteFootstep swingStep);
    bool isTwoFootCollidedAndPlot(FootstepGraphNode nodeToCheck);

    bool isGoalPoseCollidedWithStairRegion(ljh::mathlib::Pose3D<double> _goalPose,ljh::mathlib::ConvexPolygon2D stairPolygon);
};





_FOOTSTEP_PLANNER_END