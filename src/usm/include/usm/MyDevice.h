#ifndef MyDeviceH
#define MyDeviceH

#include <opencv2/highgui/highgui.hpp>
#include <boost/thread.hpp>

#include "devices/CGenericHapticDevice.h"
namespace chai3d {

class MyDevice;
typedef std::shared_ptr<MyDevice> MyDevicePtr;

class MyDevice : public cGenericHapticDevice {
 public:
  MyDevice(unsigned int a_deviceNumber = 0);
  virtual ~MyDevice();

  //! Shared MyDevice allocator.不是单例模式，可能有多个设备
  static MyDevicePtr create(unsigned int a_deviceNumber = 0) { return (std::make_shared<MyDevice>(a_deviceNumber)); }

 public:

  //! This method opens a connection to the haptic device.
  virtual bool open();

  //! This method closes the connection to the haptic device.
  virtual bool close();

  //! This method calibrates the haptic device.
  virtual bool calibrate(bool a_forceCalibration = false);

  //! This method returns the position of the device.
  virtual bool getPosition(cVector3d& a_position);

  //! This method returns the orientation frame of the device end-effector.
  virtual bool getRotation(cMatrix3d& a_rotation);

  //! This method returns the gripper angle in radian [rad].
  virtual bool getGripperAngleRad(double& a_angle);

  //! This method returns the status of all user switches [__true__ = __ON__ / __false__ = __OFF__].
  virtual bool getUserSwitches(unsigned int& a_userSwitches);

  //! This method sends a force [N] and a torque [N*m] and gripper force [N] to the haptic device.
  virtual bool setForceAndTorqueAndGripperForce(const cVector3d& a_force, const cVector3d& a_torque, double a_gripperForce);

 public:
  static unsigned int getNumDevices();

 protected:
  int m_MyVariable;
  int mouse_x_, mouse_y_, mouse_vx_, mouse_vy_;
  boost::thread *thread_;
  void threadCb();
  void mouse(int event, int x, int y);
  friend void mouseCb(int event, int x, int y, int flags, void* obj);
};
}       // namespace chai3d
#endif
