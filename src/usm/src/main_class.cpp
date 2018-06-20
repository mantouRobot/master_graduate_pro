/* author: mantou */

#include <stdio.h>

#include <opencv2/opencv.hpp>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

#include <usm/main_class.h>

#define CHECK(ret) {if(BioFailed(ret)) ROS_ERROR("DAQ init failed.");}

using namespace pangolin;

namespace main_class {

MainClass::MainClass(ros::NodeHandle nh, ros::NodeHandle pnh) :
  nh_(nh),
  pnh_(pnh),
  ball_theta_(0),
  ecd_link_(0),
  ecd_motor_(0),
  ecd_dpc_(0),
  ecd_link_last_(0),
  ecd_motor_last_(0),
  rpm_link_(0),
  rpm_motor_(0),
  rpm_link_last_(0),
  rpm_motor_last_(0),
  spring_press_radius_(30.0 / 180.0 * M_PI),
  motor_serial_("/dev/ttyS0", 115200, serial::Timeout::simpleTimeout(1000)),
  daq_link_(AdxUdCounterCtrlCreate()),
  daq_motor_(AdxUdCounterCtrlCreate()),
//  force_sensor_(nh, pnh),
  ae210_(nh, pnh),
  force_z_(0),
  control_mode_(MANUAL),
  work_mode_(STOP),
  kTransRatio_(5.5),
  test_ready_(false)
{
  initHardware();
  daq_thread_ = new boost::thread(boost::bind(&MainClass::daqThread, this));
//  test_thread_ = new boost::thread(boost::bind(&MainClass::testThread, this));
  pangolin_thread_ = new boost::thread(boost::bind(&MainClass::startPangolin, this));
  haptic_thread_ = new boost::thread(boost::bind(&MainClass::hapticRender, this));
  debug_pub_ = nh_.advertise<std_msgs::String>("/debug", 1);
}

MainClass::~MainClass() {
  if(pangolin_thread_ != nullptr) {
    pangolin_thread_->interrupt();
    pangolin_thread_->join();
    pangolin_thread_ = nullptr;
  }
  if(daq_thread_ != nullptr) {
    daq_thread_->interrupt();
    daq_thread_->join();
    daq_thread_ = nullptr;
  }
  if(test_thread_ != nullptr) {
    test_thread_->interrupt();
    test_thread_->join();
    test_thread_ = nullptr;
  }

  motor_serial_.close();
  daq_link_->Dispose();
  daq_motor_->Dispose();
}

void MainClass::initHardware()
{
  // 驱动器串口打开初始化
  if(!motor_serial_.isOpen()) {
    motor_serial_.open();
    if(!motor_serial_.isOpen())
      ROS_ERROR("Motor serial open failed!");
  }

//  ROS_INFO("Serial init successed.");

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
//  ROS_INFO("DAQ init successed.");
}

void MainClass::keyHook(const std::string& input)
{
  if(input == "a") {
//    ROS_WARN_STREAM("To Left.");
    std::string cmd = "A2,P43000,500#";
//    std::string cmd = "A2,P42500,2000#";
    motor_serial_.write(cmd);
  }
  else if(input == "d") {
//    ROS_WARN_STREAM("To right.");
    std::string cmd = "A2,N43000,500#";
    motor_serial_.write(cmd);
  }
  else if(input == "s") {
    ROS_WARN_STREAM("Stop.");
    std::string cmd = "A2,N41500,0#";
    motor_serial_.write(cmd);
  }
  else if(input == "c") {
    ROS_WARN_STREAM("Continue run.");
    control_mode_ = AUTO;
  }
//  else if(input == "p")
//    pitch_ += 10;
//  else if(input == "y")
//    yaw_ += 10;
//  else if(input == "f")
//    roll_ -= 10;
//  else
//    std::cerr << "key input error!" << std::endl;
}

// 画细矩形墙
void MainClass::drawWall() {

  glEnable(GL_TEXTURE_2D);

  glBindTexture(GL_TEXTURE_2D, texture_);

  glColor3f(1.0, 1.0, 1.0);
  glBegin(GL_POLYGON);
  glTexCoord2f(1.0, 0.0); glVertex2f(0, 0);
  glTexCoord2f(1.0, 1.0); glVertex2f(5, -5);
  glTexCoord2f(0.0, 1.0); glVertex2f(5.707, -4.293);
  glTexCoord2f(0.0, 0.0); glVertex2f(0.707, 0.707);
  glEnd();

  glBegin(GL_POLYGON);
  glTexCoord2f(1.0, 0.0); glVertex2f(0, 0);
  glTexCoord2f(1.0, 1.0); glVertex2f(-5, -5);
  glTexCoord2f(0.0, 1.0); glVertex2f(-5.707, -4.293);
  glTexCoord2f(0.0, 0.0); glVertex2f(-0.707, 0.707);
  glEnd();

  glDisable(GL_TEXTURE_2D);

}

// 画小球
void MainClass::drawBall(double ball_theta) {
  double cx, cy, r, num_segments;
  cx = 5 * sin(ball_theta);
  cy = -5 * cos(ball_theta);
  r = 0.3;
  num_segments = 30;
  glColor3f(0, 1.0, 0.0);
  glPolygonMode(GL_FRONT_AND_BACK,GL_FILL);
  glBegin(GL_TRIANGLE_FAN);
  for (int i = 0; i <= num_segments; i++)
    glVertex2f(cx + r * cos((2 * M_PI * i) / num_segments),
               cy + r * sin((2 * M_PI * i) / num_segments));
  glEnd();

}

// 画圆段，半径为5
void MainClass::drawCircle() {
  glColor3f(1.0, 1.0, 1.0);
  GLUquadricObj *qobj;
  GLuint startList = glGenLists(1);
  qobj = gluNewQuadric();
  gluQuadricDrawStyle(qobj, GLU_SILHOUETTE); /* 只画边缘  */
  gluQuadricNormals(qobj, GLU_NONE);/*不算NORMAL(顶点的方向）*/
  glNewList(startList, GL_COMPILE);/*只编译DISPLAY LIST不绘制*/
  gluPartialDisk(qobj, 0, 5.0, 50, 50, 135.0, 90.0);/* 第一个参数是A quadric object，然后内圆半径为0.0，外圆半径为1.0，z轴向分割片数，最后两个参数是圆弧的起始角和圆弧角度  */
  glEndList();
  glCallList(startList);
}

// 画弹簧，使用极坐标
void MainClass::drawSpring(double spring_press_radius) {
  const int kNumPoints = 15;
  const double kRChange = 0.5;
  const double R = 5;

  double r, theta, x, y;
  glColor3f(1.0, 0.0, 0.0);
  glLineWidth(2);
  glBegin(GL_LINE_STRIP);
  glVertex2f(-3.535, -3.535);
  for(int i = 1; i <= kNumPoints; i++) {
    if(i % 2 == 1)
      r = R - kRChange;
    else
      r = R + kRChange;
    theta = spring_press_radius / (2.0 * kNumPoints) * (2.0 * i - 1);
    x = r * cos(theta + 225.0 / 180.0 * M_PI);
    y = r * sin(theta + 225.0 / 180.0 * M_PI);
    glVertex2f(x, y);
  }
  r = R;
  theta += spring_press_radius / (2.0 * kNumPoints);
  x = r * cos(theta + 225.0 / 180.0 * M_PI);
  y = r * sin(theta + 225.0 / 180.0 * M_PI);
  glVertex2f(x, y);
  glEnd();
}

GLuint MainClass::loadTexture() {
  //OpenCV读取图像
  cv::Mat img = cv::imread("/home/mantou/usm_ws/src/usm/data/1.png", CV_LOAD_IMAGE_UNCHANGED);
  if(img.empty())
    ROS_ERROR("NO img.");

  //OpenGL纹理用整型数表示
  GLuint texture_ID;

  //将texture_ID设置为2D纹理信息
  glGenTextures(1, &texture_ID);
  glBindTexture(GL_TEXTURE_2D, texture_ID);
  //纹理放大缩小使用线性插值
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

  //纹理水平竖直方向外扩使用重复贴图
  //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  //glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  //纹理水平竖直方向外扩使用边缘像素贴图(与重复贴图二选一)
//  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
//  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

  // cv的图像存储含有对齐，小心
  glPixelStorei(GL_UNPACK_ALIGNMENT, (img.step & 3) ? 1 : 4);
  glPixelStorei(GL_UNPACK_ROW_LENGTH, img.step/img.elemSize());
  cv::flip(img, img, 0);
  //将图像内存用作纹理信息
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, img.cols, img.rows, 0, GL_BGR, GL_UNSIGNED_BYTE, img.ptr());
  return texture_ID;
}

void MainClass::initGL() {
  texture_ = loadTexture();
  GLfloat lightAmbient[4] = { 0.5, 0.5, 0.5, 1.0 };
  GLfloat lightDiffuse[4] = { 1.0, 1.0, 1.0, 1.0 };
  GLfloat lightPosition[4] = { 0.0, 0.0, 2.0, 1.0 };
  glLightfv( GL_LIGHT1, GL_AMBIENT, lightAmbient );
  glLightfv( GL_LIGHT1, GL_DIFFUSE, lightDiffuse );
  glLightfv( GL_LIGHT1, GL_POSITION, lightPosition );
  glEnable( GL_LIGHT1 );
  //  glEnable( GL_LIGHTING );
}

void MainClass::startPangolin() {
  // Load configuration data
  pangolin::ParseVarsFile("/home/mantou/usm_ws/src/usm/src/app.cfg");

  pangolin::CreateWindowAndBind("USM_FORCE_FEEDBACK", 640, 480);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  const int UI_WIDTH = 180;

  OpenGlRenderState s_cam(
        ProjectionMatrix(640,480,420,420,320,240,0.1,1000),
        ModelViewLookAt(-0,0,10, 0,0,0, pangolin::AxisY));

  // Add named OpenGL viewport to window and provide 3D Handler
  pangolin::View& d_cam = pangolin::CreateDisplay()
      .SetBounds(0.0, 1.0, Attach::Pix(UI_WIDTH), 1.0, -640.0f/480.0f)
      .SetHandler(new Handler3D(s_cam));
  // Add named Panel and bind to variables beginning 'ui'
  // A Panel is just a View with a default layout and input handling
  View& d_panel = pangolin::CreatePanel("ui")
      .SetBounds(0.0, 1.0, 0.0, Attach::Pix(UI_WIDTH));

  RegisterKeyPressCallback('a', boost::bind(&MainClass::keyHook, this, "a"));
  RegisterKeyPressCallback('d', boost::bind(&MainClass::keyHook, this, "d"));
  RegisterKeyPressCallback('s', boost::bind(&MainClass::keyHook, this, "s"));
  RegisterKeyPressCallback('c', boost::bind(&MainClass::keyHook, this, "c"));

  initGL();

  // 60Hz刷新频率
  while(!pangolin::ShouldQuit() && ros::ok())
  {
//    std_msgs::Empty empty;
//    debug_pub_.publish(empty);
    // Clear entire screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    static Var<bool> button_stop("ui.Stop!", false, false);
    static Var<bool> button_follow("ui.Follow", false, false);
    static Var<bool> button_virture_wall("ui.Virture Wall",false,false);
    static Var<bool> button_virture_spring("ui.Virtual Spring", false, false);
//    static Var<bool> button("ui.Reset", false, false);

    if(Pushed(button_stop)) {
      ROS_ERROR("Set Stop!");
      work_mode_ = STOP;
    }
    if(Pushed(button_follow)) {
      ROS_WARN("Set Follow.");
      work_mode_ = FOLLOW;
    }
    if(Pushed(button_virture_wall)) {
      ROS_WARN("Set Virture Wall.");
      work_mode_ = VIRTURE_WALL;
    }
    if(Pushed(button_virture_spring)) {
      ROS_WARN("Set Virture Spring.");
      work_mode_ = VIRTURE_SPRING;
    }



//    if(Pushed(button))
//    {
//      std::cout << "push the reset button." << std::endl;
//      SetRPY(0, 0, 0);
//    }

    // Activate efficiently by object
    d_cam.Activate(s_cam);

    // Reader some stuff
    glPushMatrix();
    glTranslatef(0, -3, 0);
    glRotatef(180, 0.0f, 0.0f, 1.0f);
    //茶壶的pitch 是我的yaw
    //茶壶的roll 是我的pitch
    if(1)
    {
      glRotatef(0, 1.0f, 0.0f, 0.0f);
      glRotatef(0, 0.0f, 1.0f, 0.0f);
      glRotatef(0, 0.0f, 0.0f, 1.0f);
    }
    else
    {
      //转换到物体坐标系
//      glRotatef((float)roll_, 1.0f, 0.0f, 0.0f); // roll
//      glRotatef((float)yaw_, 0.0f, 1.0f, 0.0f); // pitch
//      glRotatef((float)-pitch_, 0.0f, 0.0f, 1.0f); // yaw
    }
    glColor3f(1.0, 1.0, 1.0);

    drawWall();
    drawCircle(); // 半径为5
    drawBall(ball_theta_);
    drawSpring(spring_press_radius_);
    glPopMatrix();
//    ROS_WARN("rpm_link: %.2f, rpm_motor: %.2f",rpm_link_, rpm_motor_);
    // Swap frames and Process Events
    pangolin::FinishFrame();

    std_msgs::String str;
    str.data = std::to_string(force_z_);
    debug_pub_.publish(str);
  }
//  this->~MainClass();
}

void MainClass::daqThread() {
  ros::Rate r(1000); // 1000Hz的位置采集和速度计算
//  std::unique_lock<std::mutex> lock(mutex_);
//  cv_.wait(lock, [&] {return test_ready_;});
  ros::Time start_time = ros::Time::now();
  while(ros::ok()) {
    // 获取连杆当前角度
    ecd_link_ = -daq_link_->getValue();
    // 获取电机当前角度，并根据电机和传动比计算物理约束的角度
    ecd_motor_ = daq_motor_->getValue();
    ecd_dpc_ = ecd_motor_ / kTransRatio_;
    // 计算连杆速度，单位rpm
    rpm_link_ = (ecd_link_ - ecd_link_last_) / 16384.0 * 1000.0 * 60.0;
    rpm_link_ = 0.5*rpm_link_ + 0.5*rpm_link_last_;
    // 计算电机速度，单位rpm
    rpm_motor_ = (ecd_motor_ - ecd_motor_last_) / 16384.0 * 1000.0 * 60.0;
    rpm_motor_ = 0.5*rpm_motor_ + 0.5*rpm_motor_last_;

    ecd_link_last_ = ecd_link_;
    ecd_motor_last_ = ecd_motor_;
    rpm_link_last_ = rpm_link_;
    rpm_motor_last_ = rpm_motor_;

    // 采集力
//    force_z_ = force_sensor_.getZForce();
    force_z_ = ae210_.getZForce();

//    if(work_mode_ == VIRTURE_WALL)
//      std::cout << ros::Time::now() - start_time << "\t" << ecd_link_ << "\t" << force_z_ << "\t" << force_z_ << std::endl;
//      std::cout << ecd_link_ << "\t" << ecd_motor_ << "\t" << dpc_target_ << "\t" << ecd_dpc_ << std::endl;
//    ROS_INFO_STREAM_THROTTLE(1, force_z_);
//    std::cout << force_z_ << std::endl;
//    ROS_INFO_STREAM_THROTTLE(1, force_z_);
    r.sleep();
  }
}

void MainClass::testThread() {
  ros::Rate r_50ms(20);
  std::unique_lock<std::mutex> lock(mutex_);

  switch(1) {
  case 1: // 台阶模式
    // 第一个50ms，启动自动模式
    control_mode_ = AUTO;
    work_mode_ = TEST;
    dpc_target_ = 0;

    test_ready_ = true;
//    lock.unlock();
//    cv_.notify_all();

    r_50ms.sleep();
    // 第二个50ms
    dpc_target_ = 50/kTransRatio_;
    r_50ms.sleep();
    // 3
    dpc_target_ = -50/kTransRatio_;
    r_50ms.sleep();
    // 4
    dpc_target_ = 100/kTransRatio_;
    r_50ms.sleep();
    // 5
    dpc_target_ = -100/kTransRatio_;
    r_50ms.sleep();
    // 6
    dpc_target_ = 150/kTransRatio_;
    r_50ms.sleep();
    // 7
    dpc_target_ = -150/kTransRatio_;
    r_50ms.sleep();
    // 8
    dpc_target_ = 200/kTransRatio_;
    r_50ms.sleep();
    break;
  case 2: // 求速度转换用时
    control_mode_ = AUTO;
    work_mode_ = TEST;
    dpc_target_ = 0;
    break;
  default:
    break;
  }
  work_mode_ = STOP;
}

void MainClass::hapticRender() {
  // 导纳控制100Hz
  ros::Rate r(100);

  double kGapTheta = 0.06; // 小球边缘相对于中心有角位移0.06弧度
  double kClearance = 75; // 间隙为75*2个脉冲，合计角度为1.868*2度
  double kStiffness = 400;//250; // 最小为200Nmm/度，再小就不稳定了
  double kWallPosition = (15.0 - kGapTheta*180.0/M_PI)/360.0*16384.0; // 526cnt
  double force_length = 168.5; // mm
  int driver_frequency = 42000;//43900; // 用于设置超声电机驱动频率41500~44000

  // 1. 根据位置信息判断自由与约束
  // 2. 自由情况跟随
  // 3. 约束保持不动

//  std::unique_lock<std::mutex> lock(mutex_);
//  cv_.wait(lock, [&] {return test_ready_;});

//  while(!test_ready_);
  while(ros::ok()) {

    if(control_mode_ == MANUAL)
      continue;

    // 根据DPC角度进行状态判定，一共有三个状态：自由，临界，约束（临界和约束对dpc一致）
    switch(work_mode_) {
    case STOP:
      motor_serial_.write("A2,P41500,0#");
      continue;
    case TEST:
      break;
    case FOLLOW:
      dpc_target_ = ecd_link_ + kClearance;// - ecd_dpc_ ; // 根据电机当前角度计算差量
      break;
    case VIRTURE_WALL:
      if(ecd_link_ < (-1891+75)) // 左墙面
        dpc_target_ = -1891+75*2;
      else if(ecd_link_ < (kWallPosition- kClearance)) // 自由
        dpc_target_ = ecd_link_ + kClearance;// - ecd_dpc_ ; // 根据电机当前角度计算差量
      else
        dpc_target_ = kWallPosition;// - ecd_dpc_; // 直接是墙
      break;
    case VIRTURE_SPRING:
      if(ecd_link_ < (-1891+75)) // 左墙面
        dpc_target_ = -1891+75*2;
      else if(ecd_link_ < (kWallPosition- kClearance)) // 自由
        dpc_target_ = ecd_link_ + kClearance;// - ecd_dpc_ ; // 根据电机当前角度计算差量
      else if(ecd_link_ < kWallPosition) // 中间间隙段
        dpc_target_ = kWallPosition+7;// - ecd_dpc_; // 直接是墙，此时可能出现dpc与连杆角度不一致的问题，因此dpc此时稍有嵌入
//        dpc_target_ = 530 - ecd_motor_;
      else // 弹簧段
        dpc_target_ = kWallPosition + force_z_*force_length/kStiffness/360.0*16384;// - ecd_dpc_;
      break;
    default:
      break;
    }

//    ROS_INFO_THROTTLE(1, "%d,%d,%d", ecd_link_, ecd_dpc_, dpc_target_);

    // 根据dpc目标角度，计算控制偏差，用该偏差调整速度
    double pid_P = 1.1112;
    double spd_cmd = pid_P * (dpc_target_ - ecd_dpc_);
    if(fabs(spd_cmd) > 10) {
      driver_frequency = (fabs(spd_cmd) - 2046.8) / (-47) * 1000.0;//(spd_cmd - 2294) / (-52.8) * 1000.0; // (spd_cmd - 2391.2) / (-55.2) * 1000.0;
      motor_target_ = 2000;
    } else {
      motor_target_ = 0;
    }

    if(driver_frequency > 43500)
      driver_frequency = 43500;
    else if(driver_frequency < 41500)
      driver_frequency = 41500;

//    ROS_INFO("dpc_target = %d, spd_cmd = %.2f, drive_frequency = %d", dpc_target_, spd_cmd, driver_frequency);

    // 下发差量
    // 另一种思路是模糊控制，需要根据连杆速度调整驱动频率
    if(spd_cmd > 0) {
        motor_cmd_ = "A2,N"+std::to_string(driver_frequency)+","+std::to_string(abs(motor_target_)) + "#"; //42.75,43.3
    } else {
        motor_cmd_ = "A2,P"+std::to_string(driver_frequency)+","+std::to_string(abs(motor_target_)) + "#";
    }

    motor_serial_.write(motor_cmd_);

    // 更新图形
    ball_theta_ = -ecd_link_ / 16384.0 * 2 * M_PI;
//    ROS_WARN("ecd: %d, ball: %.2f", ecd_link_, ball_theta_);
    // 压缩弹簧过程
    if((ball_theta_ - kGapTheta) < -(15.0 / 180.0 * M_PI) && (ball_theta_ - kGapTheta) > -(45.0 / 180.0 * M_PI)) {
      spring_press_radius_ = 45.0 / 180.0 * M_PI + ball_theta_ - kGapTheta;
    }
    // 压缩弹簧到左碰壁
    else if((ball_theta_ - kGapTheta) <= -(45.0 / 180.0 * M_PI)) {
      spring_press_radius_ = 0;
      ball_theta_ = -(45.0 / 180.0 * M_PI - kGapTheta);
    }
    // 右碰壁
    else if((ball_theta_ + kGapTheta) > (45.0 / 180.0 * M_PI)) {
      spring_press_radius_ = 30.0 / 180.0 * M_PI;
      ball_theta_ = 45.0 / 180.0 * M_PI - kGapTheta;
    }
    // 自由空间
    else {
      spring_press_radius_ = 30.0 / 180.0 * M_PI;
    }
//    std_msgs::Empty empty;
//    debug_pub_.publish(empty);
    r.sleep();
  }
}

}






