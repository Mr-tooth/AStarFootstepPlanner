#pragma once
#include <FootstepPlannerLJH\Title.h>
#include <FootstepPlannerLJH\parameters.h>
#include <vector>
_FOOTSTEP_PLANNER_BEGIN
class StepConstraintCheck
{
private:
    parameters param;
    
public:
    StepConstraintCheck(){};
    bool isInsideStairLine();

};





_FOOTSTEP_PLANNER_END