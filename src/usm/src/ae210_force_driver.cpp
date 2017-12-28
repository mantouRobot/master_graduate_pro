#include <usm/ae210_force_driver.h>

namespace main_class {

AE210::AE210(ros::NodeHandle nh, ros::NodeHandle pnh) :
  nh_(nh),
  pnh_(nh),
  force_left_(0),
  force_right_(0),
  bias_left_(0),
  bias_right_(0),
  z_force_(0),
  force_serial_("/dev/ttyUSB0", 115200, serial::Timeout::simpleTimeout(1000))
{
  debug_pub_ = nh_.advertise<std_msgs::Empty>("/force_debug", 1);
  init();
}

AE210::~AE210()
{
  if(force_serial_.isOpen()) {
    force_serial_.flush();
    force_serial_.close();
  }
  if(force_thread_ != nullptr) {
    force_thread_->interrupt();
    force_thread_->join();
    force_thread_ = nullptr;
  }
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
  if(buffer[0] == 0xff)
    ROS_ERROR("Error in calibrating the AE210");

  // 开始数据处理线程
  force_thread_ = new boost::thread(boost::bind(&AE210::processThread, this));
}

void AE210::processThread() {
  const double kScaleLeft = -210, kScaleRight = -260;
  int bias_cnt = 10;
  ros::Rate r(500);
  for(int i = 0; i < bias_cnt; i++) {
    r.sleep();
    // 读取左侧力计算bias
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
    double raw_voltage = (256*buffer[2] + buffer[1] - 2047) * 5000.0 / 2048.0;
    bias_left_ += raw_voltage;
    // 读取右侧
    force_cmd_[0] = 0x55;
    force_cmd_[1] = 0xAA;
    force_cmd_[2] = 0x01; // AD
    force_cmd_[3] = 0x01; // ch1
    force_cmd_[4] = 0x02; // +-5V
    force_cmd_[5] = 0x0A; // samples
    force_cmd_[6] = 0x00; // no ues
    force_cmd_[7] = checkSum(force_cmd_, 0, 7);
    force_serial_.write(force_cmd_, 8);
    force_serial_.read(buffer, len);
    raw_voltage = (256*buffer[2] + buffer[1] - 2047) * 5000.0 / 2048.0;
    bias_right_ += raw_voltage;
  }
  bias_left_ = bias_left_/bias_cnt;
  bias_right_ = bias_right_/bias_cnt;
  ROS_ERROR("bias_left = %.2f, bias_right = %.2f", bias_left_, bias_right_);

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
    double raw_voltage = (256*buffer[2] + buffer[1] - 2047) * 5000.0 / 2048.0;
    force_left_ = (raw_voltage - bias_left_) / kScaleLeft;
    // 读取右侧力
    force_cmd_[0] = 0x55;
    force_cmd_[1] = 0xAA;
    force_cmd_[2] = 0x01; // AD
    force_cmd_[3] = 0x01; // ch1
    force_cmd_[4] = 0x02; // +-5V
    force_cmd_[5] = 0x0A; // samples
    force_cmd_[6] = 0x00; // no ues
    force_cmd_[7] = checkSum(force_cmd_, 0, 7);
    force_serial_.write(force_cmd_, 8);
    force_serial_.read(buffer, len);
    raw_voltage = (256*buffer[2] + buffer[1] - 2047) * 5000.0 / 2048.0;
    force_right_ = (raw_voltage - bias_right_) / kScaleRight;

    z_force_ = force_left_ - force_right_;
    debug_pub_.publish(empty_);
  }
}

double AE210::getZForce()
{
  return z_force_;
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
