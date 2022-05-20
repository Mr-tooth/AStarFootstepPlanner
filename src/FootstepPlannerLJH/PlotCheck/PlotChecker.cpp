#include <FootstepPlannerLJH/PlotCheck/PlotChecker.h>

_FOOTSTEP_PLANNER_BEGIN

void PlotChecker::plotExpansion(FootstepGraphNode nodeToExpand,std::vector<FootstepGraphNode>& fullExpansionToPack)
{
    std::vector<double> expanX,expanY,expanYaw;
    std::vector<double> nX,nY,nYaw;
    for(int i=0;i<fullExpansionToPack.size();i++)
    {
        expanX.push_back(fullExpansionToPack.at(i).getSecondStep().getX());
        expanY.push_back(fullExpansionToPack.at(i).getSecondStep().getY());
        expanYaw.push_back(fullExpansionToPack.at(i).getSecondStep().getYaw());

    }

    nX.push_back(nodeToExpand.getSecondStep().getX());
    nY.push_back(nodeToExpand.getSecondStep().getY());

    plt::clf();
    plt::plot(nX,nY,"ro");
    plt::scatter(expanX,expanY);
    
    
    
    //plt::show();

}

void PlotChecker::plotFrontier(PriorityQueue<Location,cost_t> _frontier)
{
    std::vector<double> expanX,expanY,expanYaw;
    for(int i=0;i<_frontier.elements.size();i++)
    {
        expanX.push_back(_frontier.elements.top().second.getSecondStep().getX());
        expanY.push_back(_frontier.elements.top().second.getSecondStep().getY());
        expanYaw.push_back(_frontier.elements.top().second.getSecondStep().getYaw());
        _frontier.elements.pop();
    }
    plt::scatter(expanX,expanY);

    plt::pause(0.01);
    
}

void PlotChecker::plotSearchOutcome(std::vector<Location> _outcome,ljh::mathlib::Pose3D<double> _goalPose,ljh::mathlib::Pose3D<double> _startPose)
{
    std::vector<double> vertexX(4);
    std::vector<double> vertexY(4);
    double x_num = 0.0;
    double y_num = 0.0;
    double x_start = 0.0;
    double y_start = 0.0;
    double x_goal = 0.0;
    double y_goal = 0.0;
    double s_yaw = 0.0;
    double g_yaw = 0.0;
    const double arrowLength = 0.05;

    x_start = _startPose.getPosition().getX();
    y_start = _startPose.getPosition().getY();
    x_goal  = _goalPose.getPosition().getX();
    y_goal  = _goalPose.getPosition().getY();
    s_yaw = _startPose.getOrientation().getYaw();
    g_yaw = _goalPose.getOrientation().getYaw();
    
    plt::close();
    plt::pause(0.01);
    plt::figure(2);
    plt::clf();
    for(int i=0;i<_outcome.size();i++)
    {
        getFootVertex2D(_outcome.at(i),vertexX,vertexY);
        if(_outcome.at(i).getSecondStepSide().getStepFlag() == stepL)
            plt::plot(vertexX,vertexY,"r");
        else
            plt::plot(vertexX,vertexY,"y");

    }
    vertexX.clear();
    vertexY.clear();

    vertexX.push_back(x_start);
    vertexX.push_back(x_goal);
    vertexY.push_back(y_start);
    vertexY.push_back(y_goal);
    plt::plot(vertexX,vertexY,"g");

    vertexX.clear();
    vertexY.clear();

    for(int i=0;i<_outcome.size();i++)
    {
        x_num = _outcome.at(i).getSecondStep().getX();
        y_num = _outcome.at(i).getSecondStep().getY();
        vertexX.push_back(x_num);
        vertexY.push_back(y_num);
        plt::annotate(std::to_string(i),x_num,y_num+0.02);
    }
    plt::scatter(vertexX,vertexY);
    plt::arrow(x_goal,y_goal,cos(g_yaw)*arrowLength,sin(g_yaw)*arrowLength,"r","k",0.02,0.01);
    //plt::arrow(0.8,-0.25,0.046985,-0.232899,"r","k",0.02,0.01);
    plt::arrow(x_start,y_start,cos(s_yaw)*arrowLength,sin(s_yaw)*arrowLength,"r","k",0.02,0.01);

    if(this->param.isStairAlignMode)
    {
        vertexX.clear();
        vertexY.clear();
        for(int i=0;i<4;i++)
        {
            vertexX.push_back(this->param.stairPolygon.getVertexBuffer().at(i).getX());
            vertexY.push_back(this->param.stairPolygon.getVertexBuffer().at(i).getY());
        }
        vertexX.push_back(this->param.stairPolygon.getVertexBuffer().at(0).getX());
        vertexY.push_back(this->param.stairPolygon.getVertexBuffer().at(0).getY());
        plt::plot(vertexX,vertexY,"c");


    }
    plt::set_aspect_equal();
    plt::show();
    
    //plt::pause(0.01);


}





_FOOTSTEP_PLANNER_END