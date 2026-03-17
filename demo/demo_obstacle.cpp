/**
 * Demo: Obstacle avoidance footstep planning.
 * Uses the stair polygon mechanism as an obstacle region placed
 * between start and goal, forcing the planner to route around it.
 */
#include <FootstepPlannerLJH/AStarFootstepPlanner.h>
#include <FootstepPlannerLJH/parameters.h>
#include <FootstepPlannerLJH/SimpleBodyPathPlanner/simple2DBodyPathHolder.h>
#include <FootstepPlannerLJH/StepConstraints/StepConstraintCheck.h>
#include <FootstepPlannerLJH/PlotCheck/FootPolygon.h>

#include <iostream>
#include <fstream>
#include <cmath>

using namespace ljh::heuclid;
using namespace ljh::path::footstep_planner;

int main()
{
    // === Flat terrain with obstacle ===
    // Obstacle placed at center of path to force detour
    double startX = 0.015, startY = 0.0, startZ = 0.0, startYaw = 0.0;
    double goalX  = 0.815, goalY  = -0.8, goalZ = 0.0, goalYaw = -M_PI / 2.0;

    Pose2D<double> goalPose2D(goalX, goalY, goalYaw);
    Pose3D<double> goalPose(goalX, goalY, goalZ, goalYaw, 0.0, 0.0);
    Pose3D<double> startPose(startX, startY, startZ, startYaw, 0.0, 0.0);

    // === Body path (ellipsoid) ===
    Simple2DBodyPathHolder pathHolder;
    pathHolder.initialize({startX, startY, startYaw}, goalPose2D);
    auto waypoints = pathHolder.getWayPointPath();

    {
        std::ofstream fout("demo/body_path.csv");
        fout << "idx,x,y,yaw" << std::endl;
        for (size_t i = 0; i < waypoints.size(); i++)
        {
            fout << i << ","
                 << waypoints[i].getPosition().getX() << ","
                 << waypoints[i].getPosition().getY() << ","
                 << waypoints[i].getOrientation().getYaw() << std::endl;
        }
        fout.close();
    }

    // === Obstacle polygon (stair polygon used as obstacle) ===
    // A rectangle blocking the body path in the middle area
    // The obstacle is placed at the body path center to force detour
    double obsX = 0.35, obsY = -0.38;
    double obsW = 0.12, obsH = 0.12; // 12cm x 12cm obstacle
    std::vector<Point2D<double>> obstacle({
        {obsX - obsW/2, obsY - obsH/2},
        {obsX + obsW/2, obsY - obsH/2},
        {obsX + obsW/2, obsY + obsH/2},
        {obsX - obsW/2, obsY + obsH/2}
    });

    {
        std::ofstream fout("demo/obstacle.csv");
        fout << "x,y" << std::endl;
        for (auto& p : obstacle)
            fout << p.getX() << "," << p.getY() << std::endl;
        // close polygon
        fout << obstacle[0].getX() << "," << obstacle[0].getY() << std::endl;
        fout.close();
    }

    // === Run planner ===
    parameters param;
    param.SetEdgeCostDistance(param, 4.0);
    param.SetEdgeCostYaw(param, 4.0);
    param.SetEdgeCostStaticPerStep(param, 1.4);
    param.SetMaxStepYaw(param, pi / 12.0);
    param.SetMinStepYaw(param, -pi / 12.0);
    param.SetFinalTurnProximity(param, 0.3);
    param.SetGoalDistanceProximity(param, 0.04);
    param.SetGoalYawProximity(param, 4.0 / 180.0 * pi);
    param.SetFootPolygonExtendedLength(param, 0.025);

    // HWP weights — body path following
    param.SetHWPOfWalkDistacne(param, 1.30);
    param.SetHWPOfPathDistance(param, 1.0);
    param.SetHWPOfInitialTurnDistacne(param, 1.0);
    param.SetHWPOfFinalTurnDistacne(param, 1.30);
    param.SetHWPOfFinalWalkDistacne(param, 1.30);

    // Step size constraints
    param.SetMaxStepLength(param, 0.08);
    param.SetMinStepLength(param, -0.08);
    param.SetMaxStepWidth(param, 0.22);
    param.SetMinStepWidth(param, 0.16);
    param.SetMaxStepReach(param, sqrt(pow(0.22 - 0.16, 2) + 0.08 * 0.08));

    // Enable body path following
    param.SetFollowBodyPath(param, true);

    // Enable stair align mode to activate obstacle (stair polygon) blocking
    param.SetStairAlignMode(param, true);
    param.SetStairPolygon(param, obstacle, 4, 0);

    AStarFootstepPlanner planner;
    planner.initialize(goalPose2D, goalPose, startPose);
    planner.doAStarSearch();
    planner.calFootstepSeries();
    auto accurateSteps = planner.getOrCalAccurateFootstepSeries();

    std::cout << "Accurate footsteps: " << accurateSteps.size() << std::endl;

    // === Export footsteps ===
    {
        std::ofstream fout("demo/footsteps.csv");
        fout << "step,x,y,yaw,side" << std::endl;
        for (size_t i = 0; i < accurateSteps.size(); i++)
        {
            auto& s = accurateSteps[i];
            std::string side = (s.getStepFlag() == stepL) ? "L" : "R";
            fout << i << "," << s.getX() << "," << s.getY() << ","
                 << s.getYaw() << "," << side << std::endl;
        }
        fout.close();
    }

    // === Export foot polygons ===
    {
        std::ofstream fout("demo/foot_polygons.csv");
        fout << "step,vertex,x,y" << std::endl;
        for (size_t i = 0; i < accurateSteps.size(); i++)
        {
            auto& s = accurateSteps[i];
            Pose2D<double> pose;
            pose.setPosition(s.getX(), s.getY());
            pose.setOrientation(s.getYaw());

            std::vector<double> vx, vy;
            getFootVertex2D(pose, s.getStepFlag(), vx, vy);

            for (size_t j = 0; j < vx.size(); j++)
                fout << i << "," << j << "," << vx[j] << "," << vy[j] << std::endl;
        }
        fout.close();
    }

    // === Export start/goal ===
    {
        std::ofstream fout("demo/start_goal.csv");
        fout << "pose,x,y,yaw" << std::endl;
        fout << "start," << startX << "," << startY << "," << startYaw << std::endl;
        fout << "goal," << goalX << "," << goalY << "," << goalYaw << std::endl;
        fout.close();
    }

    std::cout << "All data exported." << std::endl;
    return 0;
}
