#include <FootstepPlannerLJH/PlotCheck/PlotChecker.h>

_FOOTSTEP_PLANNER_BEGIN

void PlotChecker::plotExpansion(FootstepGraphNode nodeToExpand,std::vector<FootstepGraphNode>& fullExpansionToPack)
{
    //static std::vector<double> expanX,expanY,expanYaw;
    this->expanX.clear();this->expanY.clear();this->expanYaw.clear();
    std::vector<double> nX,nY,nYaw;
    for(int i=0;i<fullExpansionToPack.size();i++)
    {
        this->expanX.push_back(fullExpansionToPack.at(i).getSecondStep().getX());
        this->expanY.push_back(fullExpansionToPack.at(i).getSecondStep().getY());
        this->expanYaw.push_back(fullExpansionToPack.at(i).getSecondStep().getYaw());

    }

    nX.push_back(nodeToExpand.getSecondStep().getX());
    nY.push_back(nodeToExpand.getSecondStep().getY());

    plt::clf();
    plt::plot(nX,nY,"ro");
    plt::scatter(this->expanX,this->expanY);
    
    
    
    //plt::show();

}

void PlotChecker::plotFrontier(PriorityQueue<Location,cost_t> _frontier)
{
    //static std::vector<double> expanX,expanY,expanYaw;
    this->expanX.clear();this->expanY.clear();this->expanYaw.clear();
    for(int i=0;i<_frontier.elements.size();i++)
    {
        this->expanX.push_back(_frontier.elements.top().second.getSecondStep().getX());
        this->expanY.push_back(_frontier.elements.top().second.getSecondStep().getY());
        this->expanYaw.push_back(_frontier.elements.top().second.getSecondStep().getYaw());
        _frontier.elements.pop();
    }
    plt::scatter(this->expanX,this->expanY);

    plt::pause(0.01);
    
}

void PlotChecker::plotSearchOutcome(std::vector<Location> _outcome,ljh::mathlib::Pose3D<double> _goalPose,ljh::mathlib::Pose3D<double> _startPose)
{
    // static std::vector<double> vertexX(4);
    // static std::vector<double> vertexY(4);
    this->vertexX.clear();this->vertexY.clear();
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
    if(this->param.debugFlag)
        plt::close();
    plt::pause(0.01);
    plt::figure(2);
    plt::clf();
    for(int i=0;i<_outcome.size();i++)
    {
        getFootVertex2D(_outcome.at(i),this->vertexX,this->vertexY);
        if(_outcome.at(i).getSecondStepSide().getStepFlag() == stepL)
            plt::plot(this->vertexX,this->vertexY,"r");
        else
            plt::plot(this->vertexX,this->vertexY,"y");

    }
    this->vertexX.clear();
    this->vertexY.clear();

    this->vertexX.push_back(x_start);
    this->vertexX.push_back(x_goal);
    this->vertexY.push_back(y_start);
    this->vertexY.push_back(y_goal);
    plt::plot(this->vertexX,this->vertexY,"g");

    this->vertexX.clear();
    this->vertexY.clear();

    for(int i=0;i<_outcome.size();i++)
    {
        x_num = _outcome.at(i).getSecondStep().getX();
        y_num = _outcome.at(i).getSecondStep().getY();
        this->vertexX.push_back(x_num);
        this->vertexY.push_back(y_num);
        plt::annotate(std::to_string(i),x_num,y_num+0.02);
    }
    //plt::scatter(this->vertexX,this->vertexY,2.0);
    plt::arrow(x_goal,y_goal,cos(g_yaw)*arrowLength,sin(g_yaw)*arrowLength,"r","k",0.02,0.01);
    //plt::arrow(0.8,-0.25,0.046985,-0.232899,"r","k",0.02,0.01);
    plt::arrow(x_start,y_start,cos(s_yaw)*arrowLength,sin(s_yaw)*arrowLength,"r","k",0.02,0.01);

    if(this->param.isStairAlignMode)
    {
        this->vertexX.clear();
        this->vertexY.clear();
        for(int i=0;i<4;i++)
        {
            this->vertexX.push_back(this->param.stairPolygon.getVertexBuffer().at(i).getX());
            this->vertexY.push_back(this->param.stairPolygon.getVertexBuffer().at(i).getY());
        }
        this->vertexX.push_back(this->param.stairPolygon.getVertexBuffer().at(0).getX());
        this->vertexY.push_back(this->param.stairPolygon.getVertexBuffer().at(0).getY());
        plt::plot(this->vertexX,this->vertexY,"c");


    }
    plt::set_aspect_equal();
    plt::show();
    
    //plt::pause(0.01);


}


void PlotChecker::plotSearchOutcome2(std::vector<Location> _outcome,ljh::mathlib::Pose3D<double> _goalPose,ljh::mathlib::Pose3D<double> _startPose)
{
    // static std::vector<double> vertexX(4);
    // static std::vector<double> vertexY(4);
    this->vertexX.clear();this->vertexY.clear();
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
    if(this->param.debugFlag)
        plt::close();
    plt::pause(0.01);
    plt::figure(2);
    plt::clf();
    for(int i=0;i<_outcome.size();i++)
    {
        getFootVertex2D(_outcome.at(i),this->vertexX,this->vertexY);
        if(_outcome.at(i).getSecondStepSide().getStepFlag() == stepL)
            plt::plot(this->vertexX,this->vertexY,"r");
        else
            plt::plot(this->vertexX,this->vertexY,"y");

    }
    this->vertexX.clear();
    this->vertexY.clear();

    this->vertexX.push_back(x_start);
    this->vertexX.push_back(x_goal);
    this->vertexY.push_back(y_start);
    this->vertexY.push_back(y_goal);
    plt::plot(this->vertexX,this->vertexY,"g");

    this->vertexXs.clear();
    this->vertexYs.clear();

    for(int i=0;i<_outcome.size();i++)
    {
        x_num = _outcome.at(i).getSecondStep().getX();
        y_num = _outcome.at(i).getSecondStep().getY();
        this->vertexXs.push_back(x_num);
        this->vertexYs.push_back(y_num);
        plt::annotate(std::to_string(i),x_num,y_num+0.02);
    }
    plt::scatter(this->vertexXs,this->vertexYs,2.0);
    
    plt::arrow(x_goal,y_goal,cos(g_yaw)*arrowLength,sin(g_yaw)*arrowLength,"r","k",0.02,0.01);
    //plt::arrow(0.8,-0.25,0.046985,-0.232899,"r","k",0.02,0.01);
    plt::arrow(x_start,y_start,cos(s_yaw)*arrowLength,sin(s_yaw)*arrowLength,"r","k",0.02,0.01);

    if(this->param.isStairAlignMode)
    {
        this->vertexX.clear();
        this->vertexY.clear();
        for(int i=0;i<4;i++)
        {
            this->vertexX.push_back(this->param.stairPolygon.getVertexBuffer().at(i).getX());
            this->vertexY.push_back(this->param.stairPolygon.getVertexBuffer().at(i).getY());
        }
        this->vertexX.push_back(this->param.stairPolygon.getVertexBuffer().at(0).getX());
        this->vertexY.push_back(this->param.stairPolygon.getVertexBuffer().at(0).getY());
        plt::plot(this->vertexX,this->vertexY,"c");


    }
    plt::set_aspect_equal();
    plt::show();
    
    //plt::pause(0.01);


}


_FOOTSTEP_PLANNER_END