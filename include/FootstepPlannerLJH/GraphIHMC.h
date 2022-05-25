// things should be clarified in Graph:
// the class of [node]: Location
// the datatype of [edge cost]: cost_t
// the method defining the [Node Expansion rules]: getneighbor(CurrentNode)
// the method defining the [edgecost calulation bewteen parent and child node]: edgecost(current, next)
// heuristic cost can vary so make it a template parameter to the a_star_search(can be inlined this way).
#pragma once
#define _FOOTSTEP_PLANNER_BEGIN namespace ljh{namespace path{namespace footstep_planner{
#define _FOOTSTEP_PLANNER_END }}}
#include <cmath>
#include <FootstepPlannerLJH/DataType.h>
#include <FootstepPlannerLJH/Title.h>
_FOOTSTEP_PLANNER_BEGIN


class graph_test_ihmc
{


public:
    // typedef double cost_t;
    // typedef FootstepGraphNode Location;

    cost_t edgecost(const Location& current, const Location& next)
    {
        // // [cos sin; -sin cos]*[x;y]
        // this->MidStanceX =  cos(next.PosYaw)    * next.PosX    + sin(next.PosYaw)    * next.PosY
        //                    -cos(current.PosYaw) * current.PosX - sin(current.PosYaw) * current.PosY;
        // this->MidStanceY = -sin(next.PosYaw)    * next.PosX    + cos(next.PosYaw)    * next.PosY
        //                    +sin(current.PosYaw) * current.PosX - cos(current.PosYaw) * current.PosY;
        // this->MidStance = sqrt(pow(this->MidStanceX,2)+pow(this->MidStanceY,2));
        // this->DeltaYaw = std::abs(next.PosYaw-current.PosYaw);
        // this->DeltaHeight = std::abs(next.PosZ-current.PosZ);
        // //this->CoverFraction = getfracwithplane(int planenum, planedata(vertex, normal));
        // this->g_cost = cost_t(this->param.edgecost_w_d * this->MidStance +  this->param.edgecost_w_yaw * this->DeltaYaw + this->param.edgecost_w_h * this->DeltaHeight 
        //               +this->param.edgecost_w_area * this->CoverFraction + this->param.edgecost_w_static * this->StaticStepCost);
        
        return g_cost; 
    }

    void getneighbors(Location current, std::vector<Location> neighbors)
    {
        // only delete the data but keep the memory
        neighbors.clear();
    }
private:
    // weight params for edgecost
    parameters param;
    cost_t g_cost;
    //for edge-cost-cal
    double MidStanceX;
    double MidStanceY;
    double MidStance;
    double DeltaYaw;
    double DeltaHeight;
    double CoverFraction;
    double StaticStepCost;
    
    // param for 
};

template<typename Cost_t, typename Location_h>
Cost_t heuristic_ihmc(Location_h current, Location_h Goal)
{

}


_FOOTSTEP_PLANNER_END
