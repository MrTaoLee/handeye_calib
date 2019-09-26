#include "opencv2/opencv.hpp"
#include "handeye_calib/takeSnapshot.h"
#include "ros/ros.h"
#include <vector>
#include <iostream>

using namespace std;
using namespace cv;

VideoCapture cap(1); // open the default camera

// Service callback function: detect charuco board and return pose  
bool takeSnapshot(handeye_calib::takeSnapshot::Request &req, handeye_calib::takeSnapshot::Response &res)
{
  // Get latest image from camera 
  cv::Mat image;
  cap >> image; // get a new frame from camera

  res.flag = 1;
  ROS_INFO("taking snapshot");
  
  // draw results
  imshow("out", image);
  //char key = (char)waitKey(3);
  
  waitKey(3);
  return true;
}

int main(int argc, char **argv)
{
  if(!cap.isOpened())  // check if we succeeded
    return -1;
   
  ros::init(argc, argv, "takeSnapshot_server");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("takeSnapshot", takeSnapshot);
  ROS_INFO("Ready to take snapshots with camera.");
  ros::spin();



  return 0;
}