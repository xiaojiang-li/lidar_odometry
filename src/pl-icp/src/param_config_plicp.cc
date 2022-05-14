/*
 * @Descripttion: 
 * @version: 1.0
 * @Author: Li Xiaodong
 * @Date: 2022-04-09 11:14:49
 * @LastEditTime: 2022-04-09 19:57:40
 */
#include "pl-icp/param_config_plicp.h"

plicp::ParamConfig::ParamConfig(ros::NodeHandle& private_node) {
  ExternalParamSet(private_node);
  CsmParamSet(private_node);
}

plicp::ParamConfig::~ParamConfig() {}

void plicp::ParamConfig::ExternalParamSet(ros::NodeHandle& private_node) {
  private_node.param<std::string>("odom_frame", param_odom_frame_, "odom");
  private_node.param<std::string>("body_frame", param_body_frame_, "base_link");
  private_node.param<double>("kf_dist_linear",  param_keyframe_distance_, 0.1);
  private_node.param<double>("kf_dist_angular", param_keyframe_angle_, 5.0 * (M_PI / 180.0));
  private_node.param<int>("kf_scan_count", param_keyframe_count_, 10);
  return;
}
void plicp::ParamConfig::CsmParamSet(ros::NodeHandle& private_node) {
  // **** CSM 的参数 - comments copied from algos.h (by Andrea Censi)
  // Maximum angular displacement between scans
  if (!private_node.getParam("max_angular_correction_deg", param_csm_config_input_.max_angular_correction_deg))
    param_csm_config_input_.max_angular_correction_deg = 45.0;

  // Maximum translation between scans (m)
  if (!private_node.getParam("max_linear_correction", param_csm_config_input_.max_linear_correction))
    param_csm_config_input_.max_linear_correction = 1.0;

  // Maximum ICP cycle iterations
  if (!private_node.getParam("max_iterations", param_csm_config_input_.max_iterations))
    param_csm_config_input_.max_iterations = 10;

  // A threshold for stopping (m)
  if (!private_node.getParam("epsilon_xy", param_csm_config_input_.epsilon_xy))
    param_csm_config_input_.epsilon_xy = 0.000001;

  // A threshold for stopping (rad)
  if (!private_node.getParam("epsilon_theta", param_csm_config_input_.epsilon_theta))
    param_csm_config_input_.epsilon_theta = 0.000001;

  // Maximum distance for a correspondence to be valid
  if (!private_node.getParam("max_correspondence_dist", param_csm_config_input_.max_correspondence_dist))
    param_csm_config_input_.max_correspondence_dist = 1.0;

  // Noise in the scan (m)
  if (!private_node.getParam("sigma", param_csm_config_input_.sigma))
    param_csm_config_input_.sigma = 0.010;

  // Use smart tricks for finding correspondences.
  if (!private_node.getParam("use_corr_tricks", param_csm_config_input_.use_corr_tricks))
    param_csm_config_input_.use_corr_tricks = 1;

  // Restart: Restart if error is over threshold
  if (!private_node.getParam("restart", param_csm_config_input_.restart))
    param_csm_config_input_.restart = 0;

  // Restart: Threshold for restarting
  if (!private_node.getParam("restart_threshold_mean_error", param_csm_config_input_.restart_threshold_mean_error))
    param_csm_config_input_.restart_threshold_mean_error = 0.01;

  // Restart: displacement for restarting. (m)
  if (!private_node.getParam("restart_dt", param_csm_config_input_.restart_dt))
    param_csm_config_input_.restart_dt = 1.0;

  // Restart: displacement for restarting. (rad)
  if (!private_node.getParam("restart_dtheta", param_csm_config_input_.restart_dtheta))
    param_csm_config_input_.restart_dtheta = 0.1;

  // Max distance for staying in the same clustering
  if (!private_node.getParam("clustering_threshold", param_csm_config_input_.clustering_threshold))
    param_csm_config_input_.clustering_threshold = 0.25;

  // Number of neighbour rays used to estimate the orientation
  if (!private_node.getParam("orientation_neighbourhood", param_csm_config_input_.orientation_neighbourhood))
    param_csm_config_input_.orientation_neighbourhood = 20;

  // If 0, it's vanilla ICP
  if (!private_node.getParam("use_point_to_line_distance", param_csm_config_input_.use_point_to_line_distance))
    param_csm_config_input_.use_point_to_line_distance = 1;

  // Discard correspondences based on the angles
  if (!private_node.getParam("do_alpha_test", param_csm_config_input_.do_alpha_test))
    param_csm_config_input_.do_alpha_test = 0;

  // Discard correspondences based on the angles - threshold angle, in degrees
  if (!private_node.getParam("do_alpha_test_thresholdDeg", param_csm_config_input_.do_alpha_test_thresholdDeg))
    param_csm_config_input_.do_alpha_test_thresholdDeg = 20.0;

  // Percentage of correspondences to consider: if 0.9,
  // always discard the top 10% of correspondences with more error
  if (!private_node.getParam("outliers_maxPerc", param_csm_config_input_.outliers_maxPerc))
    param_csm_config_input_.outliers_maxPerc = 0.90;

  // Parameters describing a simple adaptive algorithm for discarding.
  //  1) Order the errors.
  //  2) Choose the percentile according to outliers_adaptive_order.
  //     (if it is 0.7, get the 70% percentile)
  //  3) Define an adaptive threshold multiplying outliers_adaptive_mult
  //     with the value of the error at the chosen percentile.
  //  4) Discard correspondences over the threshold.
  //  This is useful to be conservative; yet remove the biggest errors.
  if (!private_node.getParam("outliers_adaptive_order", param_csm_config_input_.outliers_adaptive_order))
    param_csm_config_input_.outliers_adaptive_order = 0.7;

  if (!private_node.getParam("outliers_adaptive_mult", param_csm_config_input_.outliers_adaptive_mult))
    param_csm_config_input_.outliers_adaptive_mult = 2.0;

  // If you already have a guess of the solution, you can compute the polar angle
  // of the points of one scan in the new position. If the polar angle is not a monotone
  // function of the readings index, it means that the surface is not visible in the
  // next position. If it is not visible, then we don't use it for matching.
  if (!private_node.getParam("do_visibility_test", param_csm_config_input_.do_visibility_test))
    param_csm_config_input_.do_visibility_test = 0;

  // no two points in laser_sens can have the same corr.
  if (!private_node.getParam("outliers_remove_doubles", param_csm_config_input_.outliers_remove_doubles))
    param_csm_config_input_.outliers_remove_doubles = 1;

  // If 1, computes the covariance of ICP using the method http://purl.org/censi/2006/icpcov
  if (!private_node.getParam("do_compute_covariance", param_csm_config_input_.do_compute_covariance))
    param_csm_config_input_.do_compute_covariance = 0;

  // Checks that find_correspondences_tricks gives the right answer
  if (!private_node.getParam("debug_verify_tricks", param_csm_config_input_.debug_verify_tricks))
    param_csm_config_input_.debug_verify_tricks = 0;

  // If 1, the field 'true_alpha' (or 'alpha') in the first scan is used to compute the
  // incidence beta, and the factor (1/cos^2(beta)) used to weight the correspondence.");
  if (!private_node.getParam("use_ml_weights", param_csm_config_input_.use_ml_weights))
    param_csm_config_input_.use_ml_weights = 0;

  // If 1, the field 'readings_sigma' in the second scan is used to weight the
  // correspondence by 1/sigma^2
  if (!private_node.getParam("use_sigma_weights", param_csm_config_input_.use_sigma_weights))
    param_csm_config_input_.use_sigma_weights = 0;
  
  return;
}
void plicp::ParamConfig::LaserScanToLDP(const sensor_msgs::LaserScan::ConstPtr &scan_msg, LDP &ldp) {
  
  unsigned int range_size = scan_msg->ranges.size();
  ldp = ld_alloc_new(range_size);

  for (unsigned int i = 0; i < range_size; i++)
  {
    // calculate position in laser frame
    double r = scan_msg->ranges[i];

    if (r > scan_msg->range_min && r < scan_msg->range_max)
    {
        // fill in laser scan data

        ldp->valid[i] = 1;
        ldp->readings[i] = r;
    }
    else
    {
        ldp->valid[i] = 0;
        ldp->readings[i] = -1; // for invalid range
    }

    ldp->theta[i] = scan_msg->angle_min + i * scan_msg->angle_increment;
    ldp->cluster[i] = -1;
  }

  ldp->min_theta = ldp->theta[0];
  ldp->max_theta = ldp->theta[range_size - 1];

  ldp->odometry[0] = 0.0;
  ldp->odometry[1] = 0.0;
  ldp->odometry[2] = 0.0;

  ldp->true_pose[0] = 0.0;
  ldp->true_pose[1] = 0.0;
  ldp->true_pose[2] = 0.0;  
}