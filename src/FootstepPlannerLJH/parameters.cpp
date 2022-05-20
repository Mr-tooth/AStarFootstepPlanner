
#include <FootstepPlannerLJH/parameters.h>
_FOOTSTEP_PLANNER_BEGIN


//double 




// edgecost weight param
CONST double parameters:: edgecost_w_d = 4;
CONST double parameters:: edgecost_w_yaw    = 4;
CONST double parameters:: edgecost_w_h      = 1;
CONST double parameters:: edgecost_w_area   = 1;
CONST double parameters:: edgecost_w_static = 1;

// NodeCheck
CONST double parameters:: MaxStepReach = sqrt(0.26*0.26 + 0.2*0.2);//0.26;//(m)

// NodeExpansion
CONST double parameters:: MinStepLength = -0.2;//-0.2;
CONST double parameters:: MaxStepLength = 0.2;//0.2;
CONST double parameters:: MinStepWidth = 0.16;
CONST double parameters:: MaxStepWidth = 0.26;//0.26;

//Robot Param
CONST double parameters:: IdealStepWidth = 0.16;

//Yaw Param
CONST double parameters:: MinStepYaw = -pi/6;
CONST double parameters:: MaxStepYaw =  pi/6;
CONST double parameters:: StepYawReductionFactorAtMaxReach = 0.2;

// don't know
// IterativeExpansion the branchsize of the childset
CONST int parameters:: MaxBranchFactor = 0; //zero means doFullExpansion
CONST double parameters:: FinalTurnProximity = 0.5;//0.2;
CONST double parameters:: AStarHeuristicWeight = 30;
CONST double parameters:: AStarHeuristicFinalWeight = 35;

CONST double parameters:: goalDistanceProximity = 0.03;
CONST double parameters:: goalYawProximity = 5*pi/180;

CONST bool parameters:: debugFlag = true;
CONST bool parameters:: isStairAlignMode = false;
CONST ljh::mathlib::ConvexPolygon2D parameters:: stairPolygon;
CONST double parameters:: footPolygonExtendedLength = 0.0;

// parameters::parameters(/* args */)
// {}

// parameters::~sparameters() {}
double parameters:: getEdgeCostDistance(const parameters& param){ return param.edgecost_w_d;}
double parameters:: getEdgeCostHeight(const parameters& param){ return param.edgecost_w_h;}
double parameters:: getEdgeCostYaw(const parameters& param){ return param.edgecost_w_d;} 
double parameters:: getEdgeCostArea(const parameters& param){ return param.edgecost_w_area; }
double parameters:: getEdgeCostStaticPerStep(const parameters& param){ return param.edgecost_w_static; }
double parameters:: getMaxStepReach(const parameters& param){ return param.MaxStepReach; }
double parameters:: getMinStepLength(const parameters& param){ return param.MinStepLength; }
double parameters:: getMaxStepLength(const parameters& param){ return param.MaxStepLength; }
double parameters:: getMinStepWidth(const parameters& param){ return param.MinStepWidth; }
double parameters:: getMaxStepWidth(const parameters& param){ return param.MaxStepWidth; }
double parameters:: getIdealStepWidth(const parameters& param){ return param.IdealStepWidth; }
double parameters:: getMinStepYaw(const parameters& param){ return param.MinStepYaw; }
double parameters:: getMaxStepYaw(const parameters& param){ return param.MaxStepYaw; }
double parameters:: getStepYawReductionFactorAtMaxReach(const parameters& param){ return param.StepYawReductionFactorAtMaxReach; }
int    parameters:: getMaxBranchFactor(const parameters& param){ return param.MaxBranchFactor; }
double parameters:: getFinalTurnProximity(const parameters& param){ return param.FinalTurnProximity; }
double parameters:: getAStartHeuristicWeight(const parameters& param){ return param.AStarHeuristicWeight; }
double parameters:: getAStartHeuristicFinalWeight(const parameters& param){ return param.AStarHeuristicFinalWeight; }
double parameters:: getGoalDistanceProximity(const parameters& param){ return param.goalDistanceProximity; }
double parameters:: getGoalYawProximity(const parameters& param){ return param.goalYawProximity; }
bool   parameters:: getDebugFlag(const parameters& param){ return param.debugFlag;}
bool   parameters:: getStairAlignMode(const parameters& param){return param.isStairAlignMode; }
ljh::mathlib::ConvexPolygon2D parameters:: getStairPolygon(const parameters& param){return param.stairPolygon;}
double parameters:: getFootPolygonExtendedLength(const parameters& param){return param.footPolygonExtendedLength;}


void parameters::SetEdgeCostDistance(parameters& param, const double& change){ param.edgecost_w_d = change;}
void parameters::SetEdgeCostYaw(parameters& param, const double& change){ param.edgecost_w_yaw = change; }
void parameters::SetEdgeCostHeight(parameters& param, const double& change){ param.edgecost_w_h = change;}
void parameters::SetEdgeCostArea(parameters& param, const double& change){ param.edgecost_w_area = change;}
void parameters::SetEdgeCostStaticPerStep(parameters& param, const double& change){ param.edgecost_w_static = change;}
void parameters::SetMaxStepReach(parameters& param, const double& change){ param.MaxStepReach = change;}
void parameters::SetMinStepLength(parameters& param, const double& change){ param.MinStepLength = change;}
void parameters::SetMaxStepLength(parameters& param, const double& change){ param.MaxStepLength = change;}
void parameters::SetMinStepWidth(parameters& param, const double& change){ param.MinStepWidth = change;}
void parameters::SetMaxStepWidth(parameters& param, const double& change){ param.MaxStepWidth = change;}
void parameters::SetIdealStepWidth(parameters& param, const double& change){ param.IdealStepWidth = change;}
void parameters::SetMinStepYaw(parameters& param, const double& change){ param.MinStepYaw = change; }
void parameters::SetMaxStepYaw(parameters& param, const double& change){ param.MaxStepYaw = change;}
void parameters::SetStepYawReductionFactorAtMaxReach(parameters& param, const double& change){ param.StepYawReductionFactorAtMaxReach = change;};
void parameters::SetMaxBranchFactor(parameters& param, const int& change){ param.MaxBranchFactor = change;}
void parameters::SetFinalTurnProximity(parameters& param, const double& change){ param.FinalTurnProximity = change;}
void parameters::SetAStartHeuristicWeight(parameters& param, const double& change){ param.AStarHeuristicWeight = change;}
void parameters::SetAStartHeuristicFinalWeight(parameters& param, const double& change){ param.AStarHeuristicFinalWeight = change;}
void parameters::SetGoalDistanceProximity(parameters& param, const double& change){ param.goalDistanceProximity = change;}
void parameters::SetGoalYawProximity(parameters& param, const double& change){ param.goalYawProximity = change;}
void parameters::SetDebugFlag(parameters& param, const bool& change){ param.debugFlag = change;}
void parameters::SetStairAlignMode(parameters& param, const bool& change){param.isStairAlignMode = change;}
void parameters::SetStairPolygon(parameters& param, std::vector<Point2D<double> > stairBuffer, int numOfVertices, bool clockwiseOrdered)
{
    param.stairPolygon.setVertexBuffer(stairBuffer);
    param.stairPolygon.setNumOfVertices(numOfVertices);
    param.stairPolygon.setClockwiseOrder(clockwiseOrdered);
}
void parameters::SetFootPolygonExtendedLength(const parameters& param, const double& change)
{
    param.footPolygonExtendedLength = change;
}
_FOOTSTEP_PLANNER_END