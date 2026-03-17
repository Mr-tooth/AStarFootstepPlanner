// Copyright 2026 Junhang Li
// SPDX-License-Identifier: Apache-2.0
//
// Stepping-stone terrain demo — NOT "shoot first, aim later"
// 1. Define random terrain patches (each unique size/rotation)
// 2. Run constrained planner — A* finds the path through patches
// 3. If planner fails, adjust seed until a valid path is found

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
#include <cstdlib>

using namespace ljh::heuclid;
using namespace ljh::path::footstep_planner;

#ifndef PI
#define PI 3.1415926535
#endif

// Simple LCG for reproducible randomness
static unsigned int rng_seed = 42;
static float frand() { rng_seed = rng_seed * 1103515245 + 12345; return (rng_seed & 0x7fffffff) / (float)0x7fffffff; }
static float frand_range(float lo, float hi) { return lo + frand() * (hi - lo); }

// Create a rotated rectangle
static ConvexPolygon2D makeRect(double cx, double cy, double hw, double hh, double angle)
{
    std::vector<Point2D<double>> v(4);
    double cos_a = cos(angle), sin_a = sin(angle);
    double lx[4] = {-hw,  hw,  hw, -hw};
    double ly[4] = {-hh, -hh,  hh,  hh};
    for(int i = 0; i < 4; i++)
        v[i].setPoint2D(cx + cos_a*lx[i] - sin_a*ly[i],
                        cy + sin_a*lx[i] + cos_a*ly[i]);
    ConvexPolygon2D poly(4);
    poly.setVertexBuffer(v);
    poly.setClockwiseOrder(false);
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

// Generate patches along a path with random sizes
// Each patch has DIFFERENT hw, hh, angle, and position
static std::vector<ConvexPolygon2D> generateRandomPatches(
    double startX, double startY, double goalX, double goalY, int seed)
{
    rng_seed = seed;
    std::vector<ConvexPolygon2D> patches;

    // Create 10-12 patches along the straight line from start to goal,
    // then add a few offset patches for lateral coverage
    double dx = goalX - startX;
    double dy = goalY - startY;
    double pathLen = sqrt(dx*dx + dy*dy);
    double pathAngle = atan2(dy, dx);

    int nAlong = 10;  // patches along the main path
    for(int i = 0; i < nAlong; i++)
    {
        double t = (double)i / (nAlong - 1);  // 0.0 to 1.0
        // Add some randomness to position (±0.08m)
        double cx = startX + dx * t + frand_range(-0.08, 0.08);
        double cy = startY + dy * t + frand_range(-0.08, 0.08);

        // Each patch has UNIQUE dimensions
        double hw = frand_range(0.18, 0.35);  // 0.36-0.70m wide
        double hh = frand_range(0.15, 0.30);  // 0.30-0.60m tall

        // Rotation: mostly aligned with path, some randomness
        double angle = pathAngle + frand_range(-0.3, 0.3);

        patches.push_back(makeRect(cx, cy, hw, hh, angle));
    }

    // Add 2-3 wider offset patches for lateral steps
    for(int i = 0; i < 3; i++)
    {
        double t = 0.15 + 0.35 * i + frand_range(-0.1, 0.1);
        double perpAngle = pathAngle + PI/2;
        double offset = frand_range(0.05, 0.15) * (i % 2 == 0 ? 1 : -1);
        double cx = startX + dx * t + cos(perpAngle) * offset;
        double cy = startY + dy * t + sin(perpAngle) * offset;
        double hw = frand_range(0.20, 0.30);
        double hh = frand_range(0.18, 0.28);
        double angle = pathAngle + frand_range(-0.4, 0.4);
        patches.push_back(makeRect(cx, cy, hw, hh, angle));
    }

    return patches;
}

// Export patch metadata for debugging
static void exportPatchInfo(const std::vector<ConvexPolygon2D>& patches, std::ostream& out)
{
    for(size_t i = 0; i < patches.size(); i++) {
        const auto& v = patches[i].getVertexBuffer();
        double cx = 0, cy = 0;
        for(const auto& p : v) { cx += p.getX(); cy += p.getY(); }
        cx /= v.size(); cy /= v.size();
        // Compute approximate hw, hh from vertex spread
        double maxDx = 0, maxDy = 0;
        for(const auto& p : v) {
            maxDx = fmax(maxDx, fabs(p.getX() - cx));
            maxDy = fmax(maxDy, fabs(p.getY() - cy));
        }
        out << "  P" << i << ": center=(" << cx << "," << cy << ") hw=" << maxDx << " hh=" << maxDy << std::endl;
    }
}

int main(int argc, char** argv)
{
    // New start/goal — different from other demos
    double startX = 0.05, startY = 0.02, startYaw = 0.0;
    double goalX = 1.05, goalY = -0.55, goalYaw = -PI/3;

    // Try different seeds until planner succeeds with >= 10 steps
    int bestSeed = -1;
    std::vector<AccurateFootstep> bestSteps;

    for(int seed : {42, 17, 73, 101, 256, 500, 777, 1000, 33, 99})
    {
        auto patches = generateRandomPatches(startX, startY, goalX, goalY, seed);

        parameters param;
        setupBaseParams(param);
        param.SetLandingZonePolygons(param, patches);
        param.SetUseLandingZoneCheck(param, true);
        param.SetStairAlignMode(param, false);

        auto steps = planSteps(param, startX, startY, startYaw, goalX, goalY, goalYaw);
        std::cout << "Seed " << seed << ": " << steps.size() << " steps" << std::endl;

        if((int)steps.size() >= 10)
        {
            bestSeed = seed;
            bestSteps = steps;
            std::cout << "  ✓ Found valid path with " << steps.size() << " steps" << std::endl;

            // Export patch info
            auto patches_final = generateRandomPatches(startX, startY, goalX, goalY, seed);
            exportPatchInfo(patches_final, std::cout);
            break;
        }
    }

    if(bestSteps.size() < 10)
    {
        std::cerr << "ERROR: No seed produced a valid path with >= 10 steps" << std::endl;
        std::cerr << "Falling back to unconstrained planner for visualization" << std::endl;

        parameters param;
        setupBaseParams(param);
        param.SetUseLandingZoneCheck(param, false);
        bestSteps = planSteps(param, startX, startY, startYaw, goalX, goalY, goalYaw);
        bestSeed = -1;  // marker: unconstrained fallback
    }

    // Generate final patches with winning seed
    auto patches = generateRandomPatches(startX, startY, goalX, goalY, bestSeed);

    std::cout << "Final: " << bestSteps.size() << " steps, seed=" << bestSeed << std::endl;

    // Export terrain patches
    for(size_t i = 0; i < patches.size(); i++) {
        std::ofstream fout("demo/terrain_" + std::to_string(i) + ".csv");
        fout << "x,y" << std::endl;
        for(const auto& p : patches[i].getVertexBuffer())
            fout << p.getX() << "," << p.getY() << std::endl;
        fout.close();
    }
    std::cout << "Wrote " << patches.size() << " terrain patches" << std::endl;

    // Body path (straight line from start to goal)
    {
        std::ofstream fout("demo/body_path.csv");
        fout << "x,y" << std::endl;
        for(int i = 0; i < 100; i++) {
            double t = (double)i / 99.0;
            fout << startX + (goalX-startX)*t << "," << startY + (goalY-startY)*t << std::endl;
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
        for(const auto& s : bestSteps)
            fout << idx++ << "," << s.getX() << "," << s.getY() << ","
                 << s.getYaw() << "," << (s.getStepFlag() == stepL ? "L" : "R") << std::endl;
        fout.close();
    }

    // Foot polygons
    {
        std::ofstream fout("demo/foot_polygons.csv");
        fout << "step,x1,y1,x2,y2,x3,y3,x4,y4" << std::endl;
        int idx = 0;
        for(const auto& s : bestSteps) {
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

    return 0;
}
