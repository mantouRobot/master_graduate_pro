#include <iostream>
#include <boost/thread.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

using namespace std;

int mouse_x, mouse_y;

// 只有在鼠标移动情况下触发
void mouseCb(int event, int x, int y, int flags, void *obj) {
  mouse_x = x;
  mouse_y = y;
}

void threadCb() {
  cv::namedWindow("mouse");
  cv::resizeWindow("mouse", 640, 480);
  cv::setMouseCallback("mouse", mouseCb);
  cv::waitKey(0);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "test");
  ros::NodeHandle n;
  boost::thread *thread = new boost::thread(threadCb);
  ros::Rate r(1000);
  while(ros::ok()) {
    ROS_INFO("%d, %d", mouse_x, mouse_y);
    r.sleep();
  }
  ros::spin();
  return 0;
}
