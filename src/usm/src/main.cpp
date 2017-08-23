#include <usm/main_class.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "main");
  ros::NodeHandle nh, pnh("~");
//  glutInit(&argc, argv);
  main_class::MainClass main_class(nh, pnh);
  ros::spin();
  return 0;
}
