/*
 * @Descripttion: 
 * @version: 1.0
 * @Author: Li Xiaodong
 * @Date: 2022-04-09 09:42:49
 * @LastEditTime: 2022-04-10 14:53:40
 */
#include "pl-icp/node_plicp.h"

int main(int argc, char *argv[]) {

  ros::init(argc, argv, "lidar_odometry_based_plicp");
  plicp::Node ros_node;
  
  ros::spin();
  return 0;
}