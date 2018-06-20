#ifndef MAIN_CLASS_H
#define MAIN_CLASS_H

#include <mutex>
#include <iostream>
#include <condition_variable>

#include <boost/thread.hpp>
#include <ros/ros.h>
#include <pangolin/pangolin.h>
#include <serial/serial.h>

#include "daq/compatibility.h"
#include "daq/bdaqctrl.h"
#include "force_sensor.h"
#include "ae210_force_driver.h"

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
  void daqThread();
  void testThread();

  ros::NodeHandle nh_, pnh_;
  ros::Publisher debug_pub_;
  boost::thread *pangolin_thread_, *haptic_thread_, *daq_thread_, *test_thread_;
  double ball_theta_;
  double spring_press_radius_;
  GLuint texture_;
  int32 dpc_target_, motor_target_;
  // 编码器
  UdCounterCtrl *daq_link_;
  UdCounterCtrl *daq_motor_;
  int32 ecd_link_, ecd_motor_, ecd_link_last_, ecd_motor_last_, ecd_dpc_;
  double rpm_link_, rpm_motor_, rpm_link_last_, rpm_motor_last_;
  // 驱动器，串口
  serial::Serial motor_serial_;
  std::string motor_cmd_;
  // 力传感器
//  ForceSensor force_sensor_;
  AE210 ae210_;
  double force_z_;

  // 传动比
  double kTransRatio_;
  // 保护
  enum Control_Mode{MANUAL, AUTO} control_mode_;
  enum Work_Mode{STOP, TEST, FOLLOW, VIRTURE_WALL, VIRTURE_SPRING} work_mode_;

  // 线程同步条件，互斥锁，条件变量
  std::mutex mutex_;
  std::condition_variable cv_;
  bool test_ready_;
};
}
#endif // MAIN_CLASS_H
