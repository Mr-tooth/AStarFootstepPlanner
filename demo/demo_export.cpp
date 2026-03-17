/**
 * Demo: Flat terrain footstep planning with body path.
 * Uses test7 tuned parameters for optimal body path following.
 * Exports real planner data to CSV for Python visualization.
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
    // === Flat terrain scenario (test7) ===
    double startX = 0.015, startY = 0.0, startZ = 0.0, startYaw = 0.0;
    double goalX  = 0.663, goalY  = -0.962, goalZ = 0.0, goalYaw = -1.554;

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
        std::cout << "Wrote " << waypoints.size() << " body path waypoints" << std::endl;
    }

    AStarFootstepPlanner planner;

    // Apply test7 tuned parameters
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

    // HWP weights — test7 tuned values
    param.SetHWPOfWalkDistacne(param, 1.30);
    param.SetHWPOfPathDistance(param, 2.50);      // Strong body path following
    param.SetHWPOfInitialTurnDistacne(param, 1.0);
    param.SetHWPOfFinalTurnDistacne(param, 1.30);
    param.SetHWPOfFinalWalkDistacne(param, 1.30);

    // Step size constraints
    param.SetMaxStepLength(param, 0.08);
    param.SetMinStepLength(param, -0.08);
    param.SetMaxStepWidth(param, 0.22);
    param.SetMinStepWidth(param, 0.16);
    param.SetMaxStepReach(param, sqrt(pow(0.22 - 0.16, 2) + 0.08 * 0.08));

    // Params are static members — set once, used everywhere

    planner.initialize(goalPose2D, goalPose, startPose);
    planner.doAStarSearch();
    planner.calFootstepSeries();
    auto accurateSteps = planner.getOrCalAccurateFootstepSeries();

    std::cout << "Accurate footsteps: " << accurateSteps.size() << std::endl;

    // === Export footstep center positions ===
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
        std::cout << "Wrote " << accurateSteps.size() << " footsteps" << std::endl;
    }

    // === Export foot polygon vertices ===
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
        std::cout << "Wrote foot polygons" << std::endl;
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
