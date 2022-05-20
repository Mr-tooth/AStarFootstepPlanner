
#pragma once
#include <vector>
#include <FootstepPlannerLJH/Title.h>
#include <FootstepPlannerLJH/DataType.h>
#include <FootstepPlannerLJH/AStarSearch.h>
#include <FootstepPlannerLJH/PlotCheck/FootPolygon.h>
#include <Heuclid/geometry/Pose3D.h>
#include <matplotlibcpp.h>
namespace plt = matplotlibcpp;
_FOOTSTEP_PLANNER_BEGIN
class PlotChecker
{
private:
    parameters param;
public:
    void plotExpansion(FootstepGraphNode nodeToExpand,std::vector<FootstepGraphNode>& fullExpansionToPack);
    void plotFrontier(PriorityQueue<Location,cost_t> _frontier);
    void plotSearchOutcome(std::vector<Location> _outcome,ljh::mathlib::Pose3D<double> _goalPose,ljh::mathlib::Pose3D<double> _startPose);
};



_FOOTSTEP_PLANNER_END