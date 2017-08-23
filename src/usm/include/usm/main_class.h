#ifndef MAIN_CLASS_H
#define MAIN_CLASS_H

#include <iostream>

#include <boost/thread.hpp>

#include <ros/ros.h>
#include <pangolin/pangolin.h>
#include <GL/glut.h>

namespace main_class {
class MainClass {
 public:
  MainClass(ros::NodeHandle nh, ros::NodeHandle pnh);
  virtual ~MainClass();

  void startPangolin();
  void keyHook(const std::string& input);
  void drawWall(void);
  void drawBall(double ball_theta);
  void drawCircle();
  void drawSpring(double spring_press_radius);
  void hapticRender();
  GLuint loadTexture();
  void initGL();

  ros::NodeHandle nh_, pnh_;
  ros::Publisher debug_pub_;
  boost::thread *pangolin_thread_, *haptic_thread_;
  double ball_theta_;
  double spring_press_radius_;
  GLuint texture_;
};
}
#endif // MAIN_CLASS_H
