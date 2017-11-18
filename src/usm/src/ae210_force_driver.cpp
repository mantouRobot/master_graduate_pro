#include <usm/ae210_force_driver.h>

namespace main_class {

AE210::AE210(ros::NodeHandle nh, ros::NodeHandle pnh) :
  nh_(nh),
  pnh_(nh),
  force_left_(0),
  force_right_(0),
  force_serial_("/dev/ttyUSB0", 115200, serial::Timeout::simpleTimeout(1000))
{
  debug_pub_ = nh_.advertise<std_msgs::Empty>("/force_debug", 1);
  init();
}

AE210::~AE210()
{

}

void AE210::init()
{
  // 驱动器串口打开初始化
  if(!force_serial_.isOpen()) {
    force_serial_.open();
    if(!force_serial_.isOpen())
      ROS_ERROR("Motor serial open failed!");
  }
  ROS_INFO("Force serial init successed.");

  force_serial_.flush();

  ros::Rate r(20);
  // 初始化校正
  force_cmd_[0] = 0x55;
  force_cmd_[1] = 0xAA;
  force_cmd_[2] = 0x09; // 矫正
  force_cmd_[3] = force_cmd_[4] = force_cmd_[5] = force_cmd_[6] = 0x00;
  force_cmd_[7] = checkSum(force_cmd_, 0, 7);
  force_serial_.write(force_cmd_, 8);
  r.sleep();
  // 读取校正结果
  uint8_t len = 6;
  uint8_t buffer[6];
  int real_read_len;
  real_read_len = force_serial_.read(buffer, len);
  ROS_WARN("Initia the AE210, real read len = %d", real_read_len);

  // 开始数据处理线程
  force_thread_ = new boost::thread(boost::bind(&AE210::processThread, this));
}

void AE210::processThread() {
  ros::Rate r(500);
  while(ros::ok()) {
    r.sleep();

    // 读取左侧力
    force_cmd_[0] = 0x55;
    force_cmd_[1] = 0xAA;
    force_cmd_[2] = 0x01; // AD
    force_cmd_[3] = 0x00; // ch0
    force_cmd_[4] = 0x02; // +-5V
    force_cmd_[5] = 0x0A; // samples
    force_cmd_[6] = 0x00; // no ues
    force_cmd_[7] = checkSum(force_cmd_, 0, 7);
    force_serial_.write(force_cmd_, 8);
    uint8_t len = 6;
    uint8_t buffer[6];
    force_serial_.read(buffer, len);
    force_left_ = (256*buffer[2] + buffer[1] - 2047) * 5000.0 / 2048.0;
    ROS_INFO_THROTTLE(1, "force_left = %.2f", force_left_);
}
}

uint8_t AE210::checkSum(uint8_t *array, int start, int len)
{
  uint8_t sum = 0;
  for(int i = start; i < len; i++) {
    sum += array[i];
  }
  return sum;
}



} // eof
