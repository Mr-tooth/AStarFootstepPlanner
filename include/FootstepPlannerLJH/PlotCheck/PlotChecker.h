
#pragma once
#include <vector>
#include <FootstepPlannerLJH/Title.h>
#include <FootstepPlannerLJH/DataType.h>
#include <FootstepPlannerLJH/AStarSearch.h>
#include <FootstepPlannerLJH/PlotCheck/FootPolygon.h>
#include <Heuclid/geometry/Pose3D.h>
#include <matplotlibcpp.h>
#include <FootstepPlannerLJH/SimpleBodyPathPlanner/simple2DBodyPathHolder.h>
namespace plt = matplotlibcpp;
_FOOTSTEP_PLANNER_BEGIN
class PlotChecker
{
private:
    parameters param;
    std::vector<double> expanX;
    std::vector<double> expanY;
    std::vector<double> expanYaw;

    std::vector<double> vertexX;
    std::vector<double> vertexY;
    std::vector<double> vertexXs;
    std::vector<double> vertexYs;
public:
    PlotChecker():param(),expanX(),expanY(),expanYaw(),vertexX(),vertexY(),vertexXs(),vertexYs(){};
    void plotExpansion(FootstepGraphNode nodeToExpand,std::vector<FootstepGraphNode>& fullExpansionToPack);
    void plotFrontier(PriorityQueue<Location,cost_t> _frontier);
    void plotGoalposeAndStair(ljh::mathlib::Pose3D<double> _goalPose);
    void plotSearchOutcome(std::vector<Location> _outcome,ljh::mathlib::Pose3D<double> _goalPose,ljh::mathlib::Pose3D<double> _startPose);
    void plotSearchOutcome2(std::vector<Location> _outcome,ljh::mathlib::Pose3D<double> _goalPose,ljh::mathlib::Pose3D<double> _startPose);
    void plotAccurateSearchOutcome(std::vector<AccurateFootstep> _outcome,ljh::mathlib::Pose3D<double> _goalPose,ljh::mathlib::Pose3D<double> _startPose);
    void plotAccurateSearchOutcome2(std::vector<AccurateFootstep> _outcome,ljh::mathlib::Pose3D<double> _goalPose,ljh::mathlib::Pose3D<double> _startPose,ljh::path::footstep_planner::Simple2DBodyPathHolder pathHolder);
};



_FOOTSTEP_PLANNER_END