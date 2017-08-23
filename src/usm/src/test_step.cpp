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

class Step{
public:
  Step(ros::NodeHandle nh, ros::NodeHandle pnh) :
    nh_(nh),
    pnh_(pnh),
    serial_("/dev/ttyS0", 115200, serial::Timeout::simpleTimeout(1000)),
    daq_(AdxUdCounterCtrlCreate())
  {
    init();
  }

  ~Step() {
    motor_cmd_ = "A1S#";
    serial_.write(motor_cmd_);
    serial_.close();

    daq_->Dispose();

    if(daq_thread_ != nullptr) {
      daq_thread_->interrupt();
      daq_thread_->join();
      daq_thread_ = nullptr;
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

    // 驱动器初始化
    ros::Rate r(1);
    motor_cmd_ = "A1E41000P16384#"; // 使能
    serial_.write(motor_cmd_);
    r.sleep();
    motor_cmd_ = "A1CA42300D500T100#"; // 调整频率，跑得快：41.2；起的动：42.3
    serial_.write(motor_cmd_);
    r.sleep(); // 等待回传
    ROS_INFO("Motor Driver init successed.");

    // 采集卡初始化
    DeviceInformation devInfo(L"PCI-1784,BID#0");
    ErrorCode ret = Success;
    ret = daq_->setSelectedDevice(devInfo);
    CHECK(ret);
    ret = daq_->setChannel(0);
    CHECK(ret);
    ret = daq_->setCountingType(AbPhaseX4);
    CHECK(ret);
    ret= daq_->setEnabled(true);
    CHECK(ret);
    ROS_INFO("DAQ init successed.");

    // 发送信号，开启采集线程
    switch(2) {
    case 1:
      ROS_INFO("Step: 0~1000 cnt.");
      motor_cmd_ = "A1R41200P50#";
      serial_.write(motor_cmd_);
      daq_thread_ = new boost::thread(boost::bind(&Step::saveThread, this));
      break;
    case 2: {
      ROS_INFO("Step: -1000~1000 cnt");
      motor_cmd_ = "A1R41200P100#";
      serial_.write(motor_cmd_);
      daq_thread_ = new boost::thread(boost::bind(&Step::saveThread, this));
      ros::Rate r_100ms(1); // 最大速度为27cnts/ms，50*27=1500cnt
      r_100ms.sleep();
      motor_cmd_ = "A1R41200N200#";
      serial_.write(motor_cmd_);
      r_100ms.sleep();
      motor_cmd_ = "A1R41200P100#";
      serial_.write(motor_cmd_);
//      r_50ms.sleep();
//      motor_cmd_ = "A1R41200N1000#";
//      serial_.write(motor_cmd_);
      break;
    }
    default:
      break;
    }

  }

  void saveThread() {
    ros::Time start_time = ros::Time::now();
    ros::Rate r(1000);
    while(ros::ok()) {
      int32 value = daq_->getValue();
      std::cout << ros::Time::now() - start_time << "\t" << value << std::endl;
      if(ros::Duration(ros::Time::now() - start_time).toSec() > 4)
        return;
      r.sleep();
    }
  }

  ros::NodeHandle nh_, pnh_;
  serial::Serial serial_;
  std::string motor_cmd_;
  UdCounterCtrl* daq_;
  boost::thread *daq_thread_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "step");
  ros::NodeHandle nh, pnh("~");
  Step step(nh, pnh);
  ros::spin();
  return 0;
}
