/*
 * @Descripttion: 
 * @version: 1.0
 * @Author: Li Xiaodong
 * @Date: 2022-04-09 09:25:54
 * @LastEditTime: 2022-04-11 15:03:38
 */
#include "pl-icp/node_plicp.h"

plicp::Node::Node() : private_node_("~"), tf_listener_(tf_buffer_){
  laser_scan_subscriber_ = node_handle_.subscribe("laser_scan", 1, &plicp::Node::ScanCallback, this);
  odom_publisher_ = node_handle_.advertise<nav_msgs::Odometry>("odom_plicp", 50);

  InitParams();
}
plicp::Node::~Node() {}

void plicp::Node::InitParams() {
  param_config_ = std::make_shared<plicp::ParamConfig>(private_node_);
  csm_input_ = &param_config_->param_csm_config_input_; // 把接收到的csm配置参数传入到csm_input,地址传递

  tf_body_to_odom_.setIdentity(); // 初始时刻载体系与map重合
  // tf_odom_to_map_.setIdentity();  // 初始时刻关键帧坐标系与map坐标系重合

  csm_input_->laser[0] = 0.;
  csm_input_->laser[1] = 0.;
  csm_input_->laser[2] = 0.;

  csm_output_.cov_x_m = 0;
  csm_output_.dx_dy1_m = 0;
  csm_output_.dx_dy2_m = 0;
  
  sign_initialized_ = true; // 初始化成功
}


void plicp::Node::ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
  
  ros::Time sign_curr_time = scan_msg->header.stamp;
  if(sign_initialized_) { // 初始化成功后进入
    GetLaserAngle(scan_msg);
    if(!GetLaserToBodyTf(scan_msg->header.frame_id))
      return;
    ParamConfig::LaserScanToLDP(scan_msg, csm_prve_scan_);
    sign_prve_time_ = sign_curr_time;
    sign_initialized_ = false; // 初始化彻底结束
    return;
  }
  // 开始推位
  LDP csm_curr_scan;
  ParamConfig::LaserScanToLDP(scan_msg, csm_curr_scan);
  ScanMatchUsePLICP(csm_curr_scan, sign_curr_time); // 
  
  geometry_msgs::TransformStamped tf_msg; // 包装tf消息，为了发布
  tf_msg.header.stamp = sign_curr_time;
  tf_msg.header.frame_id = param_config_->param_odom_frame_;
  tf_msg.child_frame_id = param_config_->param_body_frame_;
  tf_msg.transform = tf2::toMsg(tf_body_to_odom_);

  // 发布 odom 到 base_link 的 tf
  tf_broadcaster_.sendTransform(tf_msg);

  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = sign_curr_time;
  odom_msg.header.frame_id = param_config_->param_odom_frame_;
  odom_msg.child_frame_id = param_config_->param_body_frame_;
  tf2::toMsg(tf_body_to_odom_, odom_msg.pose.pose);
  odom_msg.twist.twist = state_velocity_;
  
  // 发布 odomemtry 话题
  odom_publisher_.publish(odom_msg);
  sign_prve_time_ = sign_curr_time;
}

void plicp::Node::GetLaserAngle(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {
  laser_data_cos_.clear();
  laser_data_sin_.clear();
  double angle = 0.;
  for(size_t i = 0; i < scan_msg->ranges.size(); i++) {
    angle = scan_msg->angle_min + i * scan_msg->angle_increment; // scan_msg->angle_increment 雷达角增量
    laser_data_cos_.push_back(cos(angle));
    laser_data_sin_.push_back(sin(angle));

    csm_input_->min_reading = scan_msg->range_min; //反正csm需要这些参数
    csm_input_->max_reading = scan_msg->range_max;
  }
}

bool plicp::Node::GetLaserToBodyTf(const std::string &frame_id) {
  geometry_msgs::TransformStamped transform_stamped;

  // 一旦创建了侦听器，它就开始通过连接接收tf2转换，并对它们进行长达10秒的缓冲,因为不可能一下子就得到位姿关系
  try{
    transform_stamped = tf_buffer_.lookupTransform(frame_id, param_config_->param_body_frame_, ros::Time::now(), ros::Duration(10.));
  }
  catch(tf2::TransformException &ex) { // 大家都是这样写的
    ROS_WARN("%s",ex.what());  
    ros::Duration(1.0).sleep();
    return false;
  }
  
  tf2::fromMsg(transform_stamped.transform, tf_laser_to_body_); // tf_laser_to_body_获取正确
  return true;
}

void plicp::Node::ScanMatchUsePLICP(LDP &csm_curr_scan, const ros::Time &currect_time) {
  std::memset(csm_prve_scan_->odometry, 0, sizeof(csm_prve_scan_->odometry)); // 数组清零的方式
  csm_prve_scan_->estimate[0] =0.;
  csm_prve_scan_->estimate[1] =0.; 
  csm_prve_scan_->estimate[2] =0.; 
  for(auto& i : csm_prve_scan_->true_pose) i = 0;
  
  csm_input_->laser_ref = csm_prve_scan_;
  csm_input_->laser_sens = csm_curr_scan;   

  // 两帧间的运动预测(匀速模型)
  double dt = (currect_time - sign_prve_time_).toSec();
  tf2::Transform prediction_change;
  StatePropagate(prediction_change, dt);

  csm_input_->first_guess[0] = prediction_change.getOrigin().getX();
  csm_input_->first_guess[1] = prediction_change.getOrigin().getY();
  csm_input_->first_guess[2] = tf2::getYaw(prediction_change.getRotation());
  
  // 释放协方差矩阵(我也不知道是用来干啥的)
  if (csm_output_.cov_x_m){
    gsl_matrix_free(csm_output_.cov_x_m);
    csm_output_.cov_x_m = 0;
  }
  if (csm_output_.dx_dy1_m){
    gsl_matrix_free(csm_output_.dx_dy1_m);
    csm_output_.dx_dy1_m = 0;
  }
  if (csm_output_.dx_dy2_m){
    gsl_matrix_free(csm_output_.dx_dy2_m);
    csm_output_.dx_dy2_m = 0;
  }

  // 调用CSM
  sm_icp(csm_input_, &csm_output_); // 下面两行测试成功
  // std::cout << "输出结果: "; 
  // std::cout << csm_output_.x[0]  << " " << csm_output_.x[1] << " " << std::endl;
  
  // 将结果转换到关键帧(odom)坐标系下
  tf2::Transform corr_ch;
  if(csm_output_.valid) {
    SetTfXYTheta(csm_output_.x, corr_ch); // 得到的结果是老雷达系相对于新雷达系的坐标变换矩阵
    corr_ch = tf_laser_to_body_ * corr_ch * tf_laser_to_body_.inverse();
    // 更新现在的载体坐标系
    tf_body_to_odom_ = tf_body_to_odom_ * corr_ch;
  }
  else {
    ROS_WARN("not Converged");
  }
  state_velocity_.linear.x = corr_ch.getOrigin().getX() / dt;
  state_velocity_.linear.y = corr_ch.getOrigin().getY() / dt;
  state_velocity_.linear.z = tf2::getYaw(corr_ch.getRotation()) / dt;

  ld_free(csm_prve_scan_);
  csm_prve_scan_ = csm_curr_scan; // 调试了3个小时，忘了这句话
}

void plicp::Node::StatePropagate(tf2::Transform& prediction_change, const double& dt) {
  double dx, dy, dr; 
  dx = state_velocity_.linear.x < 1e-6 ? 0. : dt * state_velocity_.linear.x;
  dy = state_velocity_.linear.y < 1e-6 ? 0. : dt * state_velocity_.linear.y;
  dr = state_velocity_.linear.z < 1e-6 ? 0. : dt * state_velocity_.linear.z; // 角速度也这样估计了，这里不是z轴位移

  // 角度限制在[-pi,pi)
  if(dr >= M_PI) 
    dr -= 2. *M_PI;
  else if(dr < M_PI)
    dr += 2. * M_PI; // 服了，这里忘乘2,又调了1个小时

  // 将预测的位姿转换为tf形式
  double xyr[3] = {dx,dy,dr};
  SetTfXYTheta(xyr, prediction_change);
  // 变换到雷达坐标系下
  prediction_change = tf_laser_to_body_.inverse() * prediction_change * tf_laser_to_body_;
}

void plicp::Node::SetTfXYTheta(const double* xyr, tf2::Transform& prediction_change) {
  prediction_change.setOrigin(tf2::Vector3(xyr[0], xyr[1], 0.));
  tf2::Quaternion q;
  q.setRPY(0., 0., xyr[2]);
  prediction_change.setRotation(q); // 此时的prediction_change即使该帧相对于上一帧的位姿预测值
}

