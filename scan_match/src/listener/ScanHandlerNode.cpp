#include <ros/ros.h>
#include "listener/ScanHandler.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "scan_node");
  ros::NodeHandle nodeHandle("~");

  ScanHandler scan_handler(nodeHandle);
  scan_handler.startListening();

  ros::spin();
  return 0;
}
