/************************
test_ms.cpp
1. 初始化编码器，串口连接控制
2. 根据时序发送M序列指令
3. 及时保存对应时间角度信息
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
   ecd_motor_(0),
   ecd_motor_last_(0),
   rpm_motor_(0),
   rpm_motor_last_(0),
   target_(0),
   finish_(false),
   serial_("/dev/ttyS0", 115200, serial::Timeout::simpleTimeout(1000)),
   daq_(AdxUdCounterCtrlCreate())
 {
//   pub_ = nh_.advertise<std_msgs::Header>("/send_data_pub", 5);
   init();
   work_thread_ = new boost::thread(boost::bind(&Step::workThread, this));
 }

 ~Step() {
   serial_.close();
   daq_->Dispose();

   if(work_thread_ != nullptr) {
     work_thread_->interrupt();
     work_thread_->join();
     work_thread_ = nullptr;
   }
   if(save_thread_ != nullptr) {
     save_thread_->interrupt();
     save_thread_->join();
     save_thread_ = nullptr;
   }
 }

 void init() {
   // 串口打开初始化
   if(!serial_.isOpen()) {
     serial_.open();
     if(!serial_.isOpen())
       ROS_ERROR("Serial open failed!");
   }
//   ROS_INFO("Serial init successed.");

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
//   ROS_INFO("DAQ init successed.");
}

 void workThread() {
   int amplitude = 200;
   std::vector<int> kMS{-1, 1, -1, -1, -1,
                       -1, 1, 1, 1, -1,
                       1, 1, -1, -1, 1};
   target_ = 0;
   ros::Rate r(50);
   save_thread_ = new boost::thread(boost::bind(&Step::saveThread, this)); // 开启记录

   int i = 4;
   while(i > 0) {
     i--;
     for(auto value : kMS) {
       r.sleep();
       target_ = value * amplitude;
       int degree = target_ - ecd_motor_;
       if(degree >= 0)
         motor_cmd_ = "A2,P41500," + std::to_string(abs(degree)) + "#";
       else
         motor_cmd_ = "A2,N41500," + std::to_string(abs(degree)) + "#";
       serial_.write(motor_cmd_);
     }
   }

   r.sleep();
   finish_ = true;
 }

 void saveThread() {
   ros::Time start_time = ros::Time::now();
   ros::Rate r(1000);
   while(ros::ok()) {
     ecd_motor_ = -daq_->getValue();
     std::cout << ros::Time::now() - start_time << "\t" << target_ << "\t" << ecd_motor_ << std::endl;
     if(ros::Duration(ros::Time::now() - start_time).toSec() > 1.4)
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
 boost::thread *work_thread_;
 boost::thread *save_thread_;
 int32 ecd_motor_, ecd_motor_last_;
 double rpm_motor_, rpm_motor_last_;
 int target_;
 bool finish_;
};

int main(int argc, char** argv) {
 ros::init(argc, argv, "step");
 ros::NodeHandle nh, pnh("~");
 Step step(nh, pnh);
 ros::spin();
 return 0;
}

