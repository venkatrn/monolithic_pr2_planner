#include<ros/ros.h>
#include<std_srvs/Empty.h>

int main(int argc, char** argv){
  ros::init(argc,argv,"generateExperimentsNode");
  ros::NodeHandle nh;

  std_srvs::Empty::Request req;
  std_srvs::Empty::Response res;

  ROS_ERROR("\n\nwait for node to initialize!\n");
  sleep(5); //WHAT A HACK!
  ROS_ERROR("\n\nwait for node to initialize!\n");
  sleep(5); //WHAT A HACK!
  ROS_ERROR("\n\nwait for node to initialize!\n");
  sleep(5); //WHAT A HACK!
  ROS_ERROR("\n\nwait for node to initialize!\n");
  sleep(5); //WHAT A HACK!
  ROS_ERROR("\n\nwait for node to initialize!\n");
  sleep(5); //WHAT A HACK!
  ros::service::waitForService("/sbpl_planning/generate_experiments_file",10);
  ros::ServiceClient genExp = ros::NodeHandle().serviceClient<std_srvs::Empty>("/sbpl_planning/generate_experiments_file", true);
  sleep(1);
  ROS_INFO("ask node to generate experiments...");

  genExp.call(req,res);

  return 0;
}

