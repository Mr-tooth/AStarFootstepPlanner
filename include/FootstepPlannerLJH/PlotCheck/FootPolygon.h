#pragma once

#include <cmath>
#include <vector>
#include <algorithm>
#include <Heuclid/geometry/Pose2D.h>
#include <FootstepPlannerLJH/EnumDef.h>
#include <FootstepPlannerLJH/Title.h>
#include <FootstepPlannerLJH/DataType.h>



#ifndef _7P3_Foot
#define _7P3_Foot
#endif


#ifdef _7P3_Foot

    #define ForwardLength   0.16
    #define BackwardLength  0.11
    #define WideWidth       0.085
    #define NarrowWidth     0.065

#endif

_FOOTSTEP_PLANNER_BEGIN


void getFootVertex2D(double *CenterPose2D, int stepflag, double *VertexX, double *VertexY);

void getFootVertex2D(ljh::mathlib::Pose2D<double> _footPose, enum StepFlag _stepflag, std::vector<double>& _vertexX,std::vector<double>& _vertexY);

void getFootVertex2D(Location _footNode, std::vector<double>& _vertexX,std::vector<double>& _vertexY);

void getExtendedFootVertex2D(double *CenterPose2D, int stepflag, double *VertexX, double *VertexY, double extendLength);
void getExtendedFootVertex2D(ljh::mathlib::Pose2D<double> _footPose, enum StepFlag _stepflag, std::vector<double>& _vertexX,std::vector<double>& _vertexY, double extendLength);
void getExtendedFootVertex2D(Location _footNode, std::vector<double>& _vertexX,std::vector<double>& _vertexY, double extendLength);

//void getFootVertex2D(ljh::mathlib::Pose2D<double> _footPose, enum StepFlag _stepflag, std::vector<Point2D<double> >& stepBuffer);
_FOOTSTEP_PLANNER_END