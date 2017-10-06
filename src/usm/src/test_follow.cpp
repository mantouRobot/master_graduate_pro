// 实现机械不安装下跟随
// 1. 数据采集卡初始化，用于读取连杆编码器和电机编码器
// 2. 串口初始化，用于控制电机
// 3. 读取两个编码器值，计算目标角度下发

/************************
 test_step.cpp
1. 初始化驱动器串口连接控制
2. 初始化编码器采集卡
3. 发送驱动器阶跃信号
4. 保存编码器信号
 ***********************/
#include <boost/thread.hpp>

#include <ros/ros.h>

#include <serial/serial.h>

#include "daq/compatibility.h"
#include "daq/bdaqctrl.h"

#define CHECK(ret) {if(BioFailed(ret)) ROS_ERROR("DAQ init failed.");}

using namespace Automation::BDaq;

class Follow{
public:
  Follow(ros::NodeHandle nh, ros::NodeHandle pnh) :
    nh_(nh),
    pnh_(pnh),
    serial_("/dev/ttyS0", 115200, serial::Timeout::simpleTimeout(1000)),
    daq_link_(AdxUdCounterCtrlCreate()),
    daq_motor_(AdxUdCounterCtrlCreate())
  {
    init();
  }

  ~Follow() {
//    motor_cmd_ = "A1S#";
//    serial_.write(motor_cmd_);
//    serial_.close();

    daq_link_->Dispose();
    daq_motor_->Dispose();

    if(daq_thread_ != nullptr) {
      daq_thread_->interrupt();
      daq_thread_->join();
      daq_thread_ = nullptr;
    }

    if(follow_thread_ != nullptr) {
      follow_thread_->interrupt();
      follow_thread_->join();
      follow_thread_ = nullptr;
    }
  }

  void init() {
    // 串口打开初始化
    if(!serial_.isOpen()) {
      serial_.open();
      if(!serial_.isOpen())
        ROS_ERROR("Serial open failed!");
    }
    ROS_INFO("Serial init successed.");

    // 采集卡初始化
    DeviceInformation devInfo(L"PCI-1784,BID#0");
    ErrorCode ret = Success;
    ret = daq_link_->setSelectedDevice(devInfo);
    CHECK(ret);
    ret = daq_link_->setChannel(0); // 连杆是通道0
    CHECK(ret);
    ret = daq_link_->setCountingType(AbPhaseX4); // 4倍频
    CHECK(ret);
    ret= daq_link_->setEnabled(true);
    CHECK(ret);

    ret = daq_motor_->setSelectedDevice(devInfo);
    CHECK(ret);
    ret = daq_motor_->setChannel(1); // 电机是通道1
    CHECK(ret);
    ret = daq_motor_->setCountingType(AbPhaseX4); // 4倍频
    CHECK(ret);
    ret= daq_motor_->setEnabled(true);
    CHECK(ret);
    ROS_INFO("DAQ init successed.");

    // 开启跟踪线程
    follow_thread_ = new boost::thread(boost::bind(&Follow::followThread, this));
  }

  void followThread() {
    ros::Rate r(120);
    while(ros::ok()) {
      // 获取连杆当前角度
      ecd_link_ = daq_link_->getValue();
      // 获取电机当前角度
      ecd_motor_ = daq_motor_->getValue();
      // 根据电机当前角度计算差量
      int32 degree = ecd_link_ + ecd_motor_;
      if(abs(degree) < 20) degree = 0;
      ROS_INFO("ecd_link:%d, ecd_motor:%d, degree:%d", ecd_link_, ecd_motor_, degree);
      // 下发差量
      if(degree > 0) {
        motor_cmd_ = "A2,P41500," + std::to_string(abs(degree)) + "#";
        ROS_INFO_STREAM(motor_cmd_);
        serial_.write(motor_cmd_);
      } else {
        motor_cmd_ = "A2,N41500," + std::to_string(abs(degree)) + "#";
        ROS_INFO_STREAM(motor_cmd_);
        serial_.write(motor_cmd_);
      }
      r.sleep();
    }
  }

  void saveThread() {
//    ros::Time start_time = ros::Time::now();
//    ros::Rate r(1000);
//    while(ros::ok()) {
//      int32 value = daq_->getValue();
//      std::cout << ros::Time::now() - start_time << "\t" << value << std::endl;
//      if(ros::Duration(ros::Time::now() - start_time).toSec() > 0.1)
//        return;
//      r.sleep();
//    }
  }

  ros::NodeHandle nh_, pnh_;
  serial::Serial serial_;
  std::string motor_cmd_;
  UdCounterCtrl *daq_link_;
  UdCounterCtrl *daq_motor_;
  int32 ecd_link_, ecd_motor_;
  boost::thread *follow_thread_;
  boost::thread *daq_thread_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "step");
  ros::NodeHandle nh, pnh("~");
  Follow follow(nh, pnh);
  ros::spin();
  return 0;
}
