// Copyright 2026 Junhang Li
// SPDX-License-Identifier: Apache-2.0
//
// Stair demo: "shoot first, aim later"
// 1. Run planner WITHOUT landing zone to get ideal footsteps
// 2. Build terrain patches CENTERED on those footsteps
// 3. Re-run WITH landing zone to verify
//
// Output: CSV files for demo_visualize.py

#include <FootstepPlannerLJH/AStarFootstepPlanner.h>
#include <FootstepPlannerLJH/parameters.h>
#include <Heuclid/geometry/ConvexPolygon2D.h>
#include <Heuclid/euclid/tuple2D/Point2D.h>
#include <Heuclid/geometry/Pose2D.h>
#include <Heuclid/geometry/Pose3D.h>
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>

using namespace ljh::heuclid;
using namespace ljh::path::footstep_planner;

#ifndef PI
#define PI 3.1415926535
#endif

static ConvexPolygon2D makeRect(double cx, double cy, double hw, double hh)
{
    std::vector<Point2D<double>> v(4);
    v[0].setPoint2D(cx-hw, cy-hh);
    v[1].setPoint2D(cx+hw, cy-hh);
    v[2].setPoint2D(cx+hw, cy+hh);
    v[3].setPoint2D(cx-hw, cy+hh);
    ConvexPolygon2D poly(4);
    poly.setVertexBuffer(v);
    poly.setClockwiseOrder(false);
    return poly;
}

// Run planner and get footsteps
static std::vector<AccurateFootstep> planSteps(parameters& param,
    double startX, double startY, double startYaw,
    double goalX, double goalY, double goalYaw)
{
    Pose2D<double> goalPose2D(goalX, goalY, goalYaw);
    Pose3D<double> goalPose(goalX, goalY, 0.0, goalYaw, 0.0, 0.0);
    Pose3D<double> startPose(startX, startY, 0.0, startYaw, 0.0, 0.0);

    AStarFootstepPlanner planner;
    planner.initialize(goalPose2D, goalPose, startPose);
    planner.doAStarSearch();
    planner.calFootstepSeries();
    return planner.getOrCalAccurateFootstepSeries();
}

// Configure common parameters
static void setupBaseParams(parameters& param)
{
    param.SetEdgeCostDistance(param, 4.0);
    param.SetEdgeCostYaw(param, 4.0);
    param.SetEdgeCostStaticPerStep(param, 1.4);
    param.SetDebugFlag(param, false);
    param.SetMaxStepYaw(param, PI/12);
    param.SetMinStepYaw(param, -PI/12);
    param.SetFinalTurnProximity(param, 0.3);
    param.SetGoalDistanceProximity(param, 0.04);
    param.SetGoalYawProximity(param, 4.0/180.0 * PI);
    param.SetFootPolygonExtendedLength(param, 0.01);
    param.SetHWPOfWalkDistacne(param, 1.30);
    param.SetFollowBodyPath(param, true);
    param.SetHWPOfPathDistance(param, 1.0);
    param.SetEdgeCostPathDev(param, 0.0);
    param.SetHWPOfInitialTurnDistacne(param, 1.0);
    param.SetHWPOfFinalTurnDistacne(param, 1.30);
    param.SetHWPOfFinalWalkDistacne(param, 1.30);
    param.SetMaxStepLength(param, 0.10);
    param.SetMinStepLength(param, -0.10);
    param.SetMaxStepWidth(param, 0.22);
    param.SetMinStepWidth(param, 0.14);
    param.SetMaxStepReach(param, sqrt(pow(0.22-0.14,2)+0.10*0.10));
}

int main()
{
    double startX = 0.0, startY = 0.0, startYaw = 0.0;
    double goalX = 0.9, goalY = -0.9, goalYaw = -PI/2;

    // ============================================================
    // Step 1: Run WITHOUT constraints to get ideal footsteps
    // ============================================================
    parameters param1;
    setupBaseParams(param1);
    param1.SetUseLandingZoneCheck(param1, false);
    param1.SetStairAlignMode(param1, false);

    auto steps = planSteps(param1, startX, startY, startYaw, goalX, goalY, goalYaw);
    std::cout << "Unconstrained search: " << steps.size() << " steps" << std::endl;

    if(steps.size() < 4)
    {
        std::cerr << "ERROR: Too few steps from unconstrained planner!" << std::endl;
        return 1;
    }

    // ============================================================
    // Step 2: Build terrain patches centered on each footstep
    // Each patch is 0.24×0.24m — generous for foot (0.14×0.22 + buffer)
    // ============================================================
    double patchHW = 0.20;  // half-width: 0.24m patch
    std::vector<ConvexPolygon2D> landingZones;
    for(const auto& s : steps)
    {
        landingZones.push_back(makeRect(s.getX(), s.getY(), patchHW, patchHW));
    }
    std::cout << "Created " << landingZones.size() << " terrain patches (centered on footsteps)" << std::endl;

    // ============================================================
    // Step 3: Verify with landing zone check
    // ============================================================
    parameters param2;
    setupBaseParams(param2);
    param2.SetLandingZonePolygons(param2, landingZones);
    param2.SetUseLandingZoneCheck(param2, true);
    param2.SetStairAlignMode(param2, false);  // No obstacle for now

    auto verified = planSteps(param2, startX, startY, startYaw, goalX, goalY, goalYaw);
    std::cout << "Landing zone search: " << verified.size() << " steps" << std::endl;

    // Use whichever has more steps
    auto& finalSteps = (verified.size() >= 4) ? verified : steps;
    bool usedFallback = (verified.size() < 4);
    if(usedFallback)
        std::cout << "WARNING: Landing zone search failed, using unconstrained result" << std::endl;

    // ============================================================
    // Step 4: Export everything
    // ============================================================

    // Terrain patches
    for(size_t i = 0; i < landingZones.size(); i++)
    {
        std::string fname = "demo/terrain_" + std::to_string(i) + ".csv";
        std::ofstream fout(fname);
        fout << "x,y" << std::endl;
        for(const auto& p : landingZones[i].getVertexBuffer())
            fout << p.getX() << "," << p.getY() << std::endl;
        fout.close();
    }
    std::cout << "Wrote " << landingZones.size() << " terrain patches" << std::endl;

    // Body path
    {
        std::ofstream fout("demo/body_path.csv");
        fout << "x,y" << std::endl;
        double dx = goalX - startX;
        double dy = goalY - startY;
        for(int i = 0; i < 100; i++)
        {
            double t = (double)i / 99.0 * (PI / 2.0);
            fout << startX + dx * sin(t) << "," << startY - dy * (1.0 - cos(t)) << std::endl;
        }
        fout.close();
    }

    // Start/goal
    {
        std::ofstream fout("demo/start_goal.csv");
        fout << "x,y,type" << std::endl;
        fout << startX << "," << startY << ",start" << std::endl;
        fout << goalX << "," << goalY << ",goal" << std::endl;
        fout.close();
    }

    // Footsteps
    {
        std::ofstream fout("demo/footsteps.csv");
        fout << "step,x,y,yaw,side" << std::endl;
        int idx = 0;
        for(const auto& s : finalSteps)
        {
            fout << idx++ << ","
                 << s.getX() << "," << s.getY() << ","
                 << s.getYaw() << ","
                 << (s.getStepFlag() == stepL ? "L" : "R") << std::endl;
        }
        fout.close();
    }

    // Foot polygons
    {
        std::ofstream fout("demo/foot_polygons.csv");
        fout << "step,x1,y1,x2,y2,x3,y3,x4,y4" << std::endl;
        int idx = 0;
        for(const auto& s : finalSteps)
        {
            std::vector<double> vx, vy;
            Pose2D<double> pose(s.getX(), s.getY(), s.getYaw());
            getExtendedFootVertex2D(pose, s.getStepFlag(), vx, vy, 0.025);
            fout << idx++ << ",";
            for(int i = 0; i < 4; i++)
                fout << vx[i] << "," << vy[i] << (i < 3 ? "," : "");
            fout << std::endl;
        }
        fout.close();
    }

    std::cout << "All data exported. Fallback=" << (usedFallback ? "yes" : "no") << std::endl;
    return 0;
}
