
#include <FootstepPlannerLJH\parameters.h>
_FOOTSTEP_PLANNER_BEGIN


//double 




// edgecost weight param
const double parameters:: edgecost_w_d = 4;
const double parameters:: edgecost_w_yaw    = 4;
const double parameters:: edgecost_w_h      = 1;
const double parameters:: edgecost_w_area   = 1;
const double parameters:: edgecost_w_static = 1;

// NodeCheck
const double parameters:: MaxStepReach = sqrt(0.26*0.26 + 0.2*0.2);//0.26;//(m)

// NodeExpansion
const double parameters:: MinStepLength = -0.2;//-0.2;
const double parameters:: MaxStepLength = 0.2;//0.2;
const double parameters:: MinStepWidth = 0.16;
const double parameters:: MaxStepWidth = 0.26;//0.26;

//Robot Param
const double parameters:: IdealStepWidth = 0.16;

//Yaw Param
const double parameters:: MinStepYaw = -pi/6;
const double parameters:: MaxStepYaw =  pi/6;
const double parameters:: StepYawReductionFactorAtMaxReach = 0.2;

// don't know
// IterativeExpansion the branchsize of the childset
const int parameters:: MaxBranchFactor = 0; //zero means doFullExpansion
const double parameters:: FinalTurnProximity = 0.5;//0.2;
const double parameters:: AStarHeuristicWeight = 30;
const double parameters:: AStarHeuristicFinalWeight = 35;

const double parameters:: goalDistanceProximity = 0.03;
const double parameters:: goalYawProximity = 5*pi/180;


// parameters::parameters(/* args */)
// {}

// parameters::~parameters() {}

_FOOTSTEP_PLANNER_END