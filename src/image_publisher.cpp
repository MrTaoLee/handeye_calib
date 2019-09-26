#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "ros/ros.h"
#include <vector>
#include <iostream>
#include "sensor_msgs/Image.h"

using namespace std;
using namespace cv;

int main( int argc, char ** argv )
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;

  ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("image", 1);
  cv::VideoCapture cap(1); // open the default camera
  ros::Rate rate(30);
  while( ros::ok() )
  {
    ROS_INFO_STREAM_ONCE("Publishing image.");
    // Get latest image from camera 
    cv::Mat image;
    cap >> image; // get a new frame from camera
    // draw results
    cv::imshow("out", image);
    waitKey(1);
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    image_pub.publish(msg);
    rate.sleep();
  }
}