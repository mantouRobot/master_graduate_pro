#ifndef FORCE_SENSOR_H
#define FORCE_SENSOR_H

#include <mutex>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <std_msgs/Empty.h>

#include <serial/serial.h>

namespace main_class {
class ForceSensor {
 public:
  ForceSensor(ros::NodeHandle nh, ros::NodeHandle pnh);
  virtual ~ForceSensor();

  void init();

  serial::Serial force_serial_;
  std::string force_cmd_;

  boost::thread *force_thread_;
  void processThread();
  bool processData(uint64_t len);

  double getZForce();
  std::mutex force_mutex_;
  double bias_;

  ros::NodeHandle nh_, pnh_;
  ros::Publisher debug_pub_;
  std_msgs::Empty empty_;


};
}



#endif // FORCE_SENSOR_H
