#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <pr2_collision_checker/body_pose.h>
#include <vector>
#include <pviz/pviz.h>
#include <sbpl_manipulation_components_pr2/pr2_kdl_robot_model.h>
#include <monolithic_pr2_planner_node/GetMobileArmPlan.h>

class ControlPlanner{
  public:
    ControlPlanner();
    ~ControlPlanner();

  private:
    void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

    FILE* fout;
    int test_num;

    PViz pviz;
    sbpl_arm_planner::PR2KDLRobotModel kdl_robot_model_;
    vector<double> start_angles0;
    vector<double> goal_angles0;
    vector<double> angles1;
    double torso_z;
    interactive_markers::InteractiveMarkerServer* int_marker_server;
    interactive_markers::MenuHandler menu_handler;
    ros::ServiceClient planner;
    ros::Publisher interrupt_pub;

    boost::mutex mutex;
    boost::thread* planner_thread;
    boost::condition_variable call_planner_cond;
    void callPlanner();
    monolithic_pr2_planner_node::GetMobileArmPlan::Request req;
    monolithic_pr2_planner_node::GetMobileArmPlan::Response res;

};
