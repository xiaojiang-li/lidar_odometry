/*
 * @Descripttion: 参数的传入，雷达数据的处理 
 * @version: 1.0
 * @Author: Li Xiaodong
 * @Date: 2022-04-09 09:12:57
 * @LastEditTime: 2022-04-09 16:52:09
 */

#include <string>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include <csm/csm_all.h>  // 这个头文件写在ros/ros.h后面就报错了

namespace plicp {
class ParamConfig {

private:
  void ExternalParamSet(ros::NodeHandle &private_node);
  void CsmParamSet(ros::NodeHandle& private_node);
  
public:
  std::string param_odom_frame_;  // 里程计坐标系,一般相对map(全局坐标系)静止
  std::string param_body_frame_;  // 载体坐标系

  double param_keyframe_distance_; // 产生关键帧所需距离
  double param_keyframe_angle_;    // 产生关键帧所需角度
  int param_keyframe_count_; //关键帧的数目
  // unsigned long int param_scan_count_; // 雷达总帧数

  sm_params param_csm_config_input_; // csm 进行plicp所需要的配置参数

  /**
   * @brief: 将ROS格式的激光雷达数据转换成CSM可识别的数据
   * @return {*}
   * @param {ConstPtr} &scan_msg
   * @param {LDP} &ldp
   */  
  static void LaserScanToLDP(const sensor_msgs::LaserScan::ConstPtr &scan_msg, LDP &ldp);
  
  ParamConfig(ros::NodeHandle& private_node);
  ~ParamConfig();
};
}