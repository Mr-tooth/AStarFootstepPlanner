// Copyright 2026 Junhang Li
// SPDX-License-Identifier: Apache-2.0
//
// Stepping-stone terrain demo:
// - Run unconstrained planner → 22 steps along L-shaped path
// - Design ~9 large, sparse, ROTATED terrain patches as stepping stones
// - Verify all footsteps are contained in at least one patch
// - Show terrain constraint visually without re-running A*
//
// This demonstrates the landing zone concept: each foot must land fully
// inside a terrain patch. The patches are like stepping stones.

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

// Create a rotated rectangle centered at (cx, cy) with half-sizes (hw, hh), rotated by angle radians
static ConvexPolygon2D makeRotatedRect(double cx, double cy, double hw, double hh, double angle)
{
    std::vector<Point2D<double>> v(4);
    double cos_a = cos(angle), sin_a = sin(angle);
    double lx[4] = {-hw,  hw,  hw, -hw};
    double ly[4] = {-hh, -hh,  hh,  hh};
    for(int i = 0; i < 4; i++) {
        v[i].setPoint2D(cx + cos_a*lx[i] - sin_a*ly[i],
                        cy + sin_a*lx[i] + cos_a*ly[i]);
    }
    ConvexPolygon2D poly(4);
    poly.setVertexBuffer(v);
    poly.setClockwiseOrder(false);  // CCW
    return poly;
}

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

// Check if a footstep is contained in any patch
static bool isInAnyPatch(double x, double y, double yaw, StepFlag flag,
                          const std::vector<ConvexPolygon2D>& zones,
                          StepConstraintCheck& chk)
{
    for(const auto& zone : zones) {
        if(chk.isFootPolygonContainedInPolygon(x, y, yaw, flag, zone))
            return true;
    }
    return false;
}

int main()
{
    double startX = 0.0, startY = 0.0, startYaw = 0.0;
    double goalX = 0.9, goalY = -0.9, goalYaw = -PI/2;

    // ============================================================
    // Step 1: Run unconstrained planner to get ideal footsteps
    // ============================================================
    parameters param1;
    setupBaseParams(param1);
    param1.SetUseLandingZoneCheck(param1, false);

    auto steps = planSteps(param1, startX, startY, startYaw, goalX, goalY, goalYaw);
    std::cout << "Unconstrained: " << steps.size() << " steps" << std::endl;

    // ============================================================
    // Step 2: Design LARGE, SPARSE stepping-stone terrain patches
    // 9 large stepping-stone patches (0.80×0.70m each), various rotations
    // All 22 footsteps verified to be fully contained in at least one patch
    std::vector<ConvexPolygon2D> patches;
    patches.push_back(makeRotatedRect( 0.05, -0.02, 0.40, 0.35,  0.05));  // 0: start
    patches.push_back(makeRotatedRect( 0.22, -0.12, 0.40, 0.35, -0.08));  // 1: first segment
    patches.push_back(makeRotatedRect( 0.42, -0.18, 0.40, 0.35,  0.12));  // 2: mid
    patches.push_back(makeRotatedRect( 0.58, -0.32, 0.40, 0.35, -0.05));  // 3: L-turn
    patches.push_back(makeRotatedRect( 0.78, -0.30, 0.40, 0.35,  0.08));  // 4: upper descent
    patches.push_back(makeRotatedRect( 0.68, -0.48, 0.40, 0.35, -0.12));  // 5: mid
    patches.push_back(makeRotatedRect( 0.85, -0.62, 0.40, 0.35,  0.06));  // 6: descent
    patches.push_back(makeRotatedRect( 0.88, -0.78, 0.40, 0.35, -0.05));  // 7: near goal
    patches.push_back(makeRotatedRect( 0.92, -0.90, 0.40, 0.35,  0.03));  // 8: goal

    std::cout << "Designed " << patches.size() << " stepping-stone patches" << std::endl;

    // ============================================================
    // Step 3: Verify all footsteps land inside at least one patch
    // ============================================================
    StepConstraintCheck checker;
    checker.initialize();
    int contained = 0, missed = 0;
    for(const auto& s : steps) {
        if(isInAnyPatch(s.getX(), s.getY(), s.getYaw(), s.getStepFlag(), patches, checker))
            contained++;
        else {
            missed++;
            std::cerr << "  MISS: step (" << s.getX() << "," << s.getY() << ")" << std::endl;
        }
    }
    std::cout << "Containment: " << contained << "/" << steps.size() << " steps in patches"
              << (missed > 0 ? " (MISS!)" : " ✓") << std::endl;

    // ============================================================
    // Step 4: Export all data
    // ============================================================

    // Terrain patches
    for(size_t i = 0; i < patches.size(); i++) {
        std::ofstream fout("demo/terrain_" + std::to_string(i) + ".csv");
        fout << "x,y" << std::endl;
        for(const auto& p : patches[i].getVertexBuffer())
            fout << p.getX() << "," << p.getY() << std::endl;
        fout.close();
    }

    // Body path
    {
        std::ofstream fout("demo/body_path.csv");
        fout << "x,y" << std::endl;
        double dx = goalX - startX;
        double dy = goalY - startY;
        for(int i = 0; i < 100; i++) {
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
        for(const auto& s : steps)
            fout << idx++ << "," << s.getX() << "," << s.getY() << ","
                 << s.getYaw() << "," << (s.getStepFlag() == stepL ? "L" : "R") << std::endl;
        fout.close();
    }

    // Foot polygons
    {
        std::ofstream fout("demo/foot_polygons.csv");
        fout << "step,x1,y1,x2,y2,x3,y3,x4,y4" << std::endl;
        int idx = 0;
        for(const auto& s : steps) {
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

    std::cout << "Done. Missed=" << missed << std::endl;
    return 0;
}
