/*
 * @Descripttion: 主要部分，回调函数所在地
 * @version: 1.0
 * @Author: Li Xiaodong
 * @Date: 2022-04-09 09:13:12
 * @LastEditTime: 2022-04-10 09:35:03
 */
#ifndef PLICP_NODE_PLICP_
#define PLICP_NODE_PLICP_

#include <iostream>
#include <vector>
#include <ros/ros.h>  // 多个文件引用标准库文件没有什么影响

#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h> //这个和下面那个是发布话题的
#include <nav_msgs/Odometry.h>

#include "param_config_plicp.h"

namespace plicp{
class Node {

private:

  ros::NodeHandle node_handle_;           // ros中的句柄
  ros::NodeHandle private_node_;          // 私有句柄,接收外界参数
  ros::Subscriber laser_scan_subscriber_; // 接收者，接收雷达雷达话题的消息
  ros::Publisher odom_publisher_;         // 声明一个Publisher

  std::shared_ptr<ParamConfig> param_config_; // 从外部传参的函数

  tf2::Transform tf_laser_to_body_; // 雷达坐标系到载体坐标系的坐标变换矩阵
  tf2::Transform tf_body_to_odom_;  // 载体坐标系到里程计坐标系的坐标变换矩阵(关键帧坐标系)
  // tf2::Transform tf_odom_to_map_;   // 里程计坐标系到世界坐标系的坐标变换矩阵
  geometry_msgs::Twist state_velocity_; // 载体速度

  tf2_ros::Buffer tf_buffer_; // tf2监听器，具体也不知道怎么用
  tf2_ros::TransformListener tf_listener_; // 配合上面食用
  tf2_ros::TransformBroadcaster tf_broadcaster_; // tf 话题发布器


  sm_params* csm_input_ = nullptr;
  sm_result csm_output_; // plicp的输出
  
  bool sign_initialized_ = false; //
  ros::Time sign_prve_time_; // 上一次推位的时间

  std::vector<double> laser_data_cos_;
  std::vector<double> laser_data_sin_;
  
  LDP csm_prve_scan_; // CSM格式的上一帧数据

  /**
   * @brief: 回调函数，接收雷达数据后的处理 
   * @return {*}
   * @param {ConstPtr} &scan_msg
   */  
  void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg);

  /**
   * @brief: 获得雷达单帧每个数据间夹角，这个参数是固定的 
   * @return {*}
   * @param {ConstPtr} &scan_msg
   */  
  void GetLaserAngle(const sensor_msgs::LaserScan::ConstPtr &scan_msg);

  /**
   * @brief: lidar odometry 参数的初始化 
   * @return {*}
   */  
  void InitParams();

  /**
   * @brief: 得到激光雷达相对与载体系的坐标变换
   * @return {bool}
   * @param {string} &frame_id
   */
  bool GetLaserToBodyTf(const std::string &frame_id);

  /**
   * @brief: 推位函数
   * @return {*}
   * @param {LDP} &csm_curr_scan
   * @param {Time} &currect_time
   */
  void ScanMatchUsePLICP(LDP &csm_curr_scan, const ros::Time &currect_time);

  /**
   * @brief: 系统状态方程(匀速模型),结果用于给CSM赋初值
   * @return {*}
   * @param {Transform} prediction_change
   */
  void StatePropagate(tf2::Transform& prediction_change, const double& dt);

  /**
   * @brief: 将数据转换为tf下的x,y,和转角
   * @return {*}
   * @param {double*} xyr
   * @param {Transform&} prediction_change
   */  
  void SetTfXYTheta(const double* xyr, tf2::Transform& prediction_change);
public:
  Node();
  ~Node();
};

}

#endif // PLICP_NODE_PLICP_