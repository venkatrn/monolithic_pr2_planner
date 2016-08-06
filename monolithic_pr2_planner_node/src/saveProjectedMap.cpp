#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include "gridmap_2d/GridMap2D.h"
#include <assert.h>

#include <opencv2/opencv.hpp>
#include <highgui.h>

#define MAX_KITCHEN_WIDTH 300 // For width 6m at 0.02m/pix
#define MAX_KITCHEN_LENGTH 500 // For length 10m at 0.02 m/pix

std::string path;

void chatterCallback(const nav_msgs::OccupancyGridPtr &map)
{
 ROS_INFO("I heard");
 gridmap_2d::GridMap2D converted_map(map);

 cv::Mat img(converted_map.binaryMap());
 //std::cout<<img.rows<<"\t"<<img.cols<<"\n";
 cv::Rect myROI(620, 624, MAX_KITCHEN_WIDTH , MAX_KITCHEN_LENGTH);
 cv::Mat croppedImage = img(myROI);
 cv::imwrite(path + "/projected_map.jpg", croppedImage);

 ROS_INFO("Saved the map as image");
}

int main(int argc, char **argv)
{
    assert(argc == 2);
    path = argv[1];
    std::cout<<path + "/projected_map.jpg";
    ros::init(argc, argv, "listener");

    ros::NodeHandle n;

    ROS_INFO("node created");
    ros::Subscriber sub = n.subscribe("/projected_map", 1, chatterCallback);

    ros::spin();

 return 0;
}
