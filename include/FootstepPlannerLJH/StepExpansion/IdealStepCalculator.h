#pragma once
#ifndef __IDEAL__STEP__CALCULATOR__
#define __IDEAL__STEP__CALCULATOR__

#define _FOOTSTEP_PLANNER_BEGIN namespace ljh{namespace path{namespace footstep_planner{
#define _FOOTSTEP_PLANNER_END }}}


#include <FootstepPlannerLJH/parameters.h>
#include <Heuclid/geometry/Pose2D.h>
using ljh::mathlib::Pose2D;
_FOOTSTEP_PLANNER_BEGIN

class IdealstepCalculator
{
    enum IdealStepMode {GOAL, ON_PATH, TOWARD_PATH};

private:
    parameters param;
    double correctiveDistanceX;
    double correctiveDistanceY;
    double correctiveYaw;
    double idealYaw;
    enum IdealStepMode yawMode;
    enum IdealStepMode stepMode;

    Pose2D<double> goalMidFootPose;
    Pose2D<double> stanceFootPose;
    Pose2D<double> idealStep;
    Pose2D<double> idealMidFootPose;



public:
    IdealstepCalculator();

};




_FOOTSTEP_PLANNER_END

#endif