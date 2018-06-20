/************************
 test_step.cpp
1. 初始化驱动器串口连接控制
2. 初始化编码器采集卡
3. 发送驱动器阶跃信号
4. 保存编码器信号
 ***********************/
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <std_msgs/Header.h>
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
    ecd_motor_(0),
    ecd_motor_last_(0),
    rpm_motor_(0),
    rpm_motor_last_(0),
    serial_("/dev/ttyS0", 115200, serial::Timeout::simpleTimeout(1000)),
    daq_(AdxUdCounterCtrlCreate())
  {
    pub_ = nh_.advertise<std_msgs::Header>("/send_data_pub", 5);
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
    //ROS_INFO("Serial init successed.");

    // 采集卡初始化
    DeviceInformation devInfo(L"PCI-1784,BID#0");
    ErrorCode ret = Success;
    ret = daq_->setSelectedDevice(devInfo);
    CHECK(ret);
    ret = daq_->setChannel(1);
    CHECK(ret);
    ret = daq_->setCountingType(AbPhaseX4);
    CHECK(ret);
    ret= daq_->setEnabled(true);
    CHECK(ret);
    //ROS_INFO("DAQ init successed.");

    // 发送信号，开启采集线程
    switch(4) {
    // 阶跃
    case 1: {
      ROS_INFO("Step: 0~1000 cnt.");
      ecd_target_ = 15;
      motor_cmd_ = "A2,P45500," + std::to_string(ecd_target_) + "#";
      serial_.write(motor_cmd_);
      daq_thread_ = new boost::thread(boost::bind(&Step::saveThread, this));
      break;
    }
    // 方波
    case 2: {
      ROS_INFO("Step: -1000~1000 cnt");
      motor_cmd_ = "A2,N41500,10000#";
      serial_.write(motor_cmd_);
      daq_thread_ = new boost::thread(boost::bind(&Step::saveThread, this));
      ros::Rate r_10ms(100); // 电机41.5KHz最大速度为27cnts/ms，人手最大16cnt/ms，人手常规3cnt/ms. 50*27=1500cnt
      r_10ms.sleep();
      int32 error = 10000 + ecd_motor_;
      motor_cmd_ = "A2,P41500," + std::to_string(error) + "#";
      serial_.write(motor_cmd_);
      r_10ms.sleep();
      error = 10000 - ecd_motor_;
      motor_cmd_ = "A2,N41500," + std::to_string(error) + "#";
      serial_.write(motor_cmd_);
      r_10ms.sleep();
      error = 10000 + ecd_motor_;
      motor_cmd_ = "A2,P41500," + std::to_string(error) + "#";
      serial_.write(motor_cmd_);
//      r_50ms.sleep();
//      motor_cmd_ = "A1R41200N1000#";
//      serial_.write(motor_cmd_);
      break;
      }
    // 走的时候发送停止
    case 3: {
      motor_cmd_ = "A2,N41500,30000#";
      serial_.write(motor_cmd_);
      daq_thread_ = new boost::thread(boost::bind(&Step::saveThread, this));
      ros::Rate r_20ms(50);
      r_20ms.sleep();
      motor_cmd_ = "A2,N41500,0#";
      serial_.write(motor_cmd_);
      break;
    }
    case 4: {
      daq_thread_ = new boost::thread(boost::bind(&Step::saveThread, this));
      ros::Rate r_50ms(20);
      int32 error;

      ecd_target_ = 0;
      motor_cmd_ = "A2,N42500,0#";
      serial_.write(motor_cmd_);     
      r_50ms.sleep();
      
      ecd_target_ = 50;
      error = ecd_target_ - ecd_motor_;
      motor_cmd_ = "A2,N42500," + std::to_string(abs(error)) + "#";
      serial_.write(motor_cmd_);     
      r_50ms.sleep();

      ecd_target_ = -50;
      error = ecd_target_ - ecd_motor_;
      motor_cmd_ = "A2,P42500," + std::to_string(abs(error)) + "#";
      serial_.write(motor_cmd_);     
      r_50ms.sleep();

      ecd_target_ = 100;
      error = ecd_target_ - ecd_motor_;
      motor_cmd_ = "A2,N42500," + std::to_string(abs(error)) + "#";
      serial_.write(motor_cmd_);     
      r_50ms.sleep();

      ecd_target_ = -100;
      error = ecd_target_ - ecd_motor_;
      motor_cmd_ = "A2,P42500," + std::to_string(abs(error)) + "#";
      serial_.write(motor_cmd_);     
      r_50ms.sleep();

      ecd_target_ = 150;
      error = ecd_target_ - ecd_motor_;
      motor_cmd_ = "A2,N42500," + std::to_string(abs(error)) + "#";
      serial_.write(motor_cmd_);     
      r_50ms.sleep();

      ecd_target_ = -150;
      error = ecd_target_ - ecd_motor_;
      motor_cmd_ = "A2,P42500," + std::to_string(abs(error)) + "#";
      serial_.write(motor_cmd_);     
      r_50ms.sleep();

      ecd_target_ = 200;
      error = ecd_target_ - ecd_motor_;
      motor_cmd_ = "A2,N42500," + std::to_string(abs(error)) + "#";
      serial_.write(motor_cmd_);     
      r_50ms.sleep();
      break;
    }
      case 5: { // 求取速度转换用时
      ros::Rate r_50ms(20);
      int32 error;

      ecd_target_ = 5000;
      error = ecd_target_ - ecd_motor_;
      motor_cmd_ = "A2,N43500," + std::to_string(abs(error)) + "#";
      serial_.write(motor_cmd_);
      daq_thread_ = new boost::thread(boost::bind(&Step::saveThread, this));
      r_50ms.sleep();

      ecd_target_ = 5000;
      error = ecd_target_ - ecd_motor_;
      motor_cmd_ = "A2,N41500," + std::to_string(abs(error)) + "#";
      serial_.write(motor_cmd_);
      r_50ms.sleep();

      ecd_target_ = -5000;
      error = ecd_target_ - ecd_motor_;
      motor_cmd_ = "A2,P41500," + std::to_string(abs(error)) + "#";
      serial_.write(motor_cmd_);
      r_50ms.sleep();

      ecd_target_ = 5000;
      error = ecd_target_ - ecd_motor_;
      motor_cmd_ = "A2,P43500," + std::to_string(abs(error)) + "#";
      serial_.write(motor_cmd_);
      r_50ms.sleep();
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
      ecd_motor_ = daq_->getValue();
      rpm_motor_ = (ecd_motor_ - ecd_motor_last_) / 16384.0 * 1000.0 * 60.0;
      rpm_motor_ = 0.5*rpm_motor_ + 0.5*rpm_motor_last_;
      ecd_motor_last_ = ecd_motor_;
      rpm_motor_last_ = rpm_motor_;
      std::cout << ros::Time::now() - start_time << "\t" << ecd_target_ << "\t" << ecd_motor_ << std::endl;
      if(ros::Duration(ros::Time::now() - start_time).toSec() > 0.25)
        return;
      r.sleep();
    }
  }

  ros::NodeHandle nh_, pnh_;
  ros::Publisher pub_;
  serial::Serial serial_;
  serial::Serial usb_serial_;
  std::string motor_cmd_;
  UdCounterCtrl* daq_;
  boost::thread *daq_thread_;
  int32 ecd_motor_, ecd_motor_last_;
  double rpm_motor_, rpm_motor_last_;
  int32 ecd_target_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "step");
  ros::NodeHandle nh, pnh("~");
  Step step(nh, pnh);
  ros::spin();
  return 0;
}










///************************
// test_step.cpp
//1. 初始化驱动器串口连接控制
//2. 初始化编码器采集卡
//3. 发送驱动器阶跃信号
//4. 保存编码器信号
// ***********************/
//#include <boost/thread.hpp>

//#include <ros/ros.h>
//#include <std_msgs/Header.h>
//#include <serial/serial.h>

//#include "daq/compatibility.h"
//#include "daq/bdaqctrl.h"

//#define CHECK(ret) {if(BioFailed(ret)) ROS_ERROR("DAQ init failed.");}

//using namespace Automation::BDaq;

//class Step{
//public:
//  Step(ros::NodeHandle nh, ros::NodeHandle pnh) :
//    nh_(nh),
//    pnh_(pnh),
//    cnt_send_(0),
//    cnt_rec_(0),
//    serial_("/dev/ttyS0", 115200, serial::Timeout::simpleTimeout(1000)),
//    usb_serial_("/dev/ttyUSB0", 115200, serial::Timeout::simpleTimeout(1000)),
//    daq_(AdxUdCounterCtrlCreate())
//  {
//    pub_ = nh_.advertise<std_msgs::Header>("/send_data_pub", 5);
//    init();
//  }

//  ~Step() {
//    motor_cmd_ = "A1S#";
//    serial_.write(motor_cmd_);
//    serial_.close();

//    daq_->Dispose();

//    if(daq_thread_ != nullptr) {
//      daq_thread_->interrupt();
//      daq_thread_->join();
//      daq_thread_ = nullptr;
//    }
//  }

//  void init() {
//    // 串口打开初始化
//    if(!serial_.isOpen()) {
//      serial_.open();
//      if(!serial_.isOpen())
//        ROS_ERROR("Serial open failed!");
//    }
//    if(!usb_serial_.isOpen()) {
//      usb_serial_.open();
//      if(!usb_serial_.isOpen())
//        ROS_ERROR("Serial open failed!");
//    }
//    ROS_INFO("Serial init successed.");

//    // 驱动器初始化
////    ros::Rate r(1);
////    motor_cmd_ = "A1E41000P16384#"; // 使能
////    serial_.write(motor_cmd_);
////    r.sleep();
////    motor_cmd_ = "A1CA42300D500T100#"; // 调整频率，跑得快：41.2；起的动：42.3
////    serial_.write(motor_cmd_);
////    r.sleep(); // 等待回传
////    ROS_INFO("Motor Driver init successed.");

//    // 采集卡初始化
//    DeviceInformation devInfo(L"PCI-1784,BID#0");
//    ErrorCode ret = Success;
//    ret = daq_->setSelectedDevice(devInfo);
//    CHECK(ret);
//    ret = daq_->setChannel(1);
//    CHECK(ret);
//    ret = daq_->setCountingType(AbPhaseX4);
//    CHECK(ret);
//    ret= daq_->setEnabled(true);
//    CHECK(ret);
//    ROS_INFO("DAQ init successed.");

//    // 发送信号，开启采集线程
//    switch(1) {
//    // 阶跃
//    case 1: {
//      ROS_INFO("Step: 0~1000 cnt.");
//      motor_cmd_ = "A2,N41500,100#";
//      ros::Time last = ros::Time::now();
////      motor_cmd_ = std::to_string(ros::Time::now().toSec());// + "\n";
//      std_msgs::Header header;
//      header.stamp = ros::Time::now();
//      header.frame_id = "send data.";
//      daq_thread_ = new boost::thread(boost::bind(&Step::saveThread, this));

////      serial_.write(motor_cmd_);


//      while(ros::ok()) {
//        pub_.publish(header);
//        serial_.write(motor_cmd_);
//        cnt_send_++;
//        ros::Rate r_10ms(100);
//        r_10ms.sleep();

//      }

////      daq_thread_ = new boost::thread(boost::bind(&Step::saveThread, this));
//      break;
//    }
//    // 方波
//    case 2: {
//      ROS_INFO("Step: -1000~1000 cnt");

//      serial_.write(motor_cmd_);
//      daq_thread_ = new boost::thread(boost::bind(&Step::saveThread, this));
//      ros::Rate r_10ms(50); // 最大速度为27cnts/ms，50*27=1500cnt
//      r_10ms.sleep();
//      int32 error = 50 + value_;
//      motor_cmd_ = "A2,P41500," + std::to_string(error) + "#";
//      serial_.write(motor_cmd_);
//      r_10ms.sleep();
//      error = 50 - value_;
//      motor_cmd_ = "A2,N41500," + std::to_string(error) + "#";
//      serial_.write(motor_cmd_);
//      r_10ms.sleep();
//      error = 50 + value_;
//      motor_cmd_ = "A2,P41500," + std::to_string(error) + "#";
//      serial_.write(motor_cmd_);
////      r_50ms.sleep();
////      motor_cmd_ = "A1R41200N1000#";
////      serial_.write(motor_cmd_);
//      break;
//      }
//    // 走的时候发送停止
//    case 3: {
//      motor_cmd_ = "A2,N41500,30000#";
//      serial_.write(motor_cmd_);
//      daq_thread_ = new boost::thread(boost::bind(&Step::saveThread, this));
//      ros::Rate r_20ms(50);
//      r_20ms.sleep();
//      motor_cmd_ = "A2,N41500,0#";
//      serial_.write(motor_cmd_);
//      break;
//    }
//    default:
//      break;
//    }

//  }

//  void saveThread() {
////    ros::Time start_time = ros::Time::now();
////    ros::Rate r(1000);
////    while(ros::ok()) {
////      value_ = daq_->getValue();
////      std::cout << ros::Time::now() - start_time << "\t" << value_ << std::endl;
////      if(ros::Duration(ros::Time::now() - start_time).toSec() > 0.05)
////        return;
////      r.sleep();
////    }
//    while(ros::ok()) {
//      std::string str = usb_serial_.read(14);
//      cnt_rec_++;
//      ROS_INFO("%s, %d, %d",  str.c_str(), cnt_send_, cnt_rec_);
//    }
//  }

//  ros::NodeHandle nh_, pnh_;
//  ros::Publisher pub_;
//  serial::Serial serial_;
//  serial::Serial usb_serial_;
//  std::string motor_cmd_;
//  UdCounterCtrl* daq_;
//  boost::thread *daq_thread_;
//  int32 value_;
//  int cnt_send_, cnt_rec_;
//};

//int main(int argc, char** argv) {
//  ros::init(argc, argv, "step");
//  ros::NodeHandle nh, pnh("~");
//  Step step(nh, pnh);
//  ros::spin();
//  return 0;
//}
