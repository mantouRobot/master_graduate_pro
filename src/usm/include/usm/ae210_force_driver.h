#ifndef AE210_FORCE_DRIVER_H
#define AE210_FORCE_DRIVER_H



#include <mutex>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <std_msgs/Empty.h>

#include <serial/serial.h>

namespace main_class {
class AE210 {
 public:
  AE210(ros::NodeHandle nh, ros::NodeHandle pnh);
  virtual ~AE210();

  void init();
  uint8_t checkSum(uint8_t *array, int start, int len);

  serial::Serial force_serial_;
  uint8_t force_cmd_[7];

  boost::thread *force_thread_;
  void processThread();
  double getZForce();

  std::mutex force_mutex_;
  double bias_;

  ros::NodeHandle nh_, pnh_;
  ros::Publisher debug_pub_;
  std_msgs::Empty empty_;

  double force_left_, force_right_, bias_left_, bias_right_, z_force_;
};
}



#endif // AE210_FORCE_DRIVER_H
