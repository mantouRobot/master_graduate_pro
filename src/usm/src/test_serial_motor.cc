/* ALL RIGHT RESERVED BY MANTOU */

#include <iostream>

#include <ros/ros.h>

#include <serial/serial.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_serial_motor");
  ros::NodeHandle nh, pnh("~");

  // 打开串口
  serial::Serial my_serial("/dev/ttyS0", 115200, serial::Timeout::simpleTimeout(1000));
  std::cout << "Is the serial port open?";
  if(my_serial.isOpen())
    std::cout << " Yes." << std::endl;
  else {
    std::cout << " No. Now trying open the serial port." << std::endl;
    my_serial.open();
    if(my_serial.isOpen())
      std::cout << "Open serial success." << std::endl;
    else {
      std::cout << "Can't open serial. Exit." << std::endl;
      return 1;
    }
  }

  std::string cmd;

  // 使能
  ros::Rate r(1);
  cmd = "A1E41000P16384#";
  my_serial.write(cmd);
  r.sleep(); // 等待回传

  // 调整频率
  cmd = "A1CA42300D500T100#"; //跑得快：41.2；起的动：42.3
  my_serial.write(cmd);
  r.sleep(); // 等待回传
//  cmd = "A1R42300P5000#";
//  my_serial.write(cmd);
//  r.sleep();

  ros::Rate rate(1);
  while(ros::ok()) {
    cmd = "A1R41200P20000#";
    my_serial.write(cmd);
    rate.sleep();
  }

  cmd = "A1S#";
  my_serial.write(cmd);
  my_serial.close();

  return 0;
}
