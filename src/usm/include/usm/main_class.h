#ifndef MAIN_CLASS_H
#define MAIN_CLASS_H

#include <iostream>

#include <boost/thread.hpp>
#include <ros/ros.h>
#include <pangolin/pangolin.h>
#include <serial/serial.h>

#include "daq/compatibility.h"
#include "daq/bdaqctrl.h"

using namespace Automation::BDaq;

namespace main_class {
class MainClass {
 public:
  MainClass(ros::NodeHandle nh, ros::NodeHandle pnh);
  virtual ~MainClass();

  void startPangolin();
  void keyHook(const std::string& input);
  void drawWall(void);
  void drawBall(double ball_theta);
  void drawCircle();
  void drawSpring(double spring_press_radius);
  void hapticRender();
  GLuint loadTexture();
  void initGL();

  void initHardware();

  ros::NodeHandle nh_, pnh_;
  ros::Publisher debug_pub_;
  boost::thread *pangolin_thread_, *haptic_thread_;
  double ball_theta_;
  double spring_press_radius_;
  GLuint texture_;
  // 编码器
  UdCounterCtrl *daq_link_;
  UdCounterCtrl *daq_motor_;
  int32 ecd_link_, ecd_motor_;
  // 串口
  serial::Serial serial_;
  std::string motor_cmd_;
};
}
#endif // MAIN_CLASS_H
