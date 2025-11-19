/* includes //{ */

#include <rclcpp/rclcpp.hpp>
#include <omp.h>
#include <mutex>
#include <math.h>
#include <csignal>
#include <unistd.h>
#include <so3_math.h>
#include <Eigen/Core>
#include <imu_processing.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/vector3.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#include <estimator.h>
#include <preprocess.h>
#include <common_lib.h>

#include <mrs_lib/node.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>
#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/timer_handler.h>
#include <mrs_lib/transform_broadcaster.h>

//}

/* defines //{ */

#define MAXN (720000)
#define PUBFRAME_PERIOD (20)

//}

namespace point_lio
{

/* class PointLio //{ */

class PointLio : public mrs_lib::Node {
public:
  PointLio(rclcpp::NodeOptions options);

  void initialize();

  std::atomic<bool> is_initialized_ = false;

private:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;

  static PointLio *instance;

private:
  // | ----------------------- parameters ----------------------- |

  double main_timer_rate_;

  bool   is_first_frame = true;
  double lidar_end_time = 0.0, first_lidar_time = 0.0, time_con = 0.0;
  double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0;

  std::string            lid_topic, imu_topic;
  bool                   prop_at_freq_of_imu, check_satu, con_frame, cut_frame;
  bool                   use_imu_as_input, space_down_sample, publish_odometry_without_downsample;
  int                    init_map_size, con_frame_num;
  double                 match_s, satu_acc, satu_gyro, cut_frame_time_interval;
  float                  plane_thr;
  double                 filter_size_surf_min, filter_size_map_min, fov_deg;
  double                 cube_len;
  float                  DET_RANGE;
  bool                   imu_en, gravity_align, non_station_start;
  double                 imu_time_inte;
  double                 laser_point_cov, acc_norm;
  double                 vel_cov, acc_cov_input, gyr_cov_input;
  double                 gyr_cov_output, acc_cov_output, b_gyr_cov, b_acc_cov;
  double                 imu_meas_acc_cov, imu_meas_omg_cov;
  int                    lidar_type;
  std::vector<double>    gravity_init, gravity;
  std::vector<double>    extrinT;
  std::vector<double>    extrinR;
  bool                   runtime_pos_log, path_en, extrinsic_est_en = true;
  bool                   scan_pub_en, scan_body_pub_en;
  double                 path_diff_t, path_diff_R;
  shared_ptr<Preprocess> p_pre;
  double                 time_lag_imu_to_lidar = 0.0;
  std::string            uav_name;
  bool                   _publish_fcu_tf_;

  // | ------------------------- common ------------------------- |

  M3D Eye3d;
  M3F Eye3f;
  V3D Zero3d;
  V3F Zero3f;

  // | ------------------------ Estimator ----------------------- |

  PointCloudXYZI::Ptr                           normvec;
  std::vector<int>                              time_seq;
  PointCloudXYZI::Ptr                           feats_down_body;
  PointCloudXYZI::Ptr                           feats_down_world;
  std::vector<V3D>                              pbody_list;
  std::vector<PointVector>                      Nearest_Points;
  KD_TREE<PointType>                            ikdtree;
  std::vector<float>                            pointSearchSqDis;
  bool                                          point_selected_surf[100000] = {0};
  std::vector<M3D>                              crossmat_list;
  int                                           effct_feat_num = 0;
  int                                           k;
  int                                           idx;
  esekfom::esekf<state_input, 24, input_ikfom>  kf_input;
  esekfom::esekf<state_output, 30, input_ikfom> kf_output;
  state_input                                   state_in;
  state_output                                  state_out;
  input_ikfom                                   input_in;
  V3D                                           angvel_avr, acc_avr;

  V3D Lidar_T_wrt_IMU;
  M3D Lidar_R_wrt_IMU;

  typedef MTK::vect<3, double>             vect3;
  typedef MTK::SO3<double>                 SO3;
  typedef MTK::S2<double, 98090, 10000, 1> S2;
  typedef MTK::vect<1, double>             vect1;
  typedef MTK::vect<2, double>             vect2;

  Eigen::Matrix<double, 24, 24> process_noise_cov_input();
  Eigen::Matrix<double, 30, 30> process_noise_cov_output();

  Eigen::Matrix<double, 24, 1> get_f_input(state_input &s, const input_ikfom &in);

  static Eigen::Matrix<double, 24, 1> get_f_input_static(state_input &s, const input_ikfom &in) {
    return PointLio::instance->get_f_input(s, in);
  }

  Eigen::Matrix<double, 30, 1> get_f_output(state_output &s, const input_ikfom &in);

  static Eigen::Matrix<double, 30, 1> get_f_output_static(state_output &s, const input_ikfom &in) {
    return instance->get_f_output(s, in);
  }

  Eigen::Matrix<double, 24, 24> df_dx_input(state_input &s, const input_ikfom &in);

  static Eigen::Matrix<double, 24, 24> df_dx_input_static(state_input &s, const input_ikfom &in) {
    return instance->df_dx_input(s, in);
  }

  Eigen::Matrix<double, 30, 30> df_dx_output(state_output &s, const input_ikfom &in);

  static Eigen::Matrix<double, 30, 30> df_dx_output_static(state_output &s, const input_ikfom &in) {
    return instance->df_dx_output(s, in);
  }

  vect3 SO3ToEuler(const SO3 &rot);

  void h_model_input(state_input &s, esekfom::dyn_share_modified<double> &ekfom_data);

  static void h_model_input_static(state_input &s, esekfom::dyn_share_modified<double> &ekfom_data) {
    return instance->h_model_input(s, ekfom_data);
  }

  void h_model_output(state_output &s, esekfom::dyn_share_modified<double> &ekfom_data);

  static void h_model_output_static(state_output &s, esekfom::dyn_share_modified<double> &ekfom_data) {
    return instance->h_model_output(s, ekfom_data);
  }

  void h_model_IMU_output(state_output &s, esekfom::dyn_share_modified<double> &ekfom_data);

  static void h_model_IMU_output_static(state_output &s, esekfom::dyn_share_modified<double> &ekfom_data) {
    return instance->h_model_IMU_output(s, ekfom_data);
  }

  void pointBodyToWorld(PointType const *const pi, PointType *const po);

  // | ------------------------- methods ------------------------ |

  void pointBodyLidarToIMU(PointType const *const pi, PointType *const po);

  void points_cache_collect();

  void lasermap_fov_segment();

  void callbackStandardPC(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  void callbackLivox(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg);

  void callbackIMU(const sensor_msgs::msg::Imu::ConstSharedPtr msg_in);

  bool sync_packages(MeasureGroup &meas);

  void map_incremental();

  void publish_init_kdtree(void);

  void publish_frame_world();

  void publish_frame_body();

  template <typename T>
  void set_posestamp(T &out);

  template <typename T>
  void set_odomtwist(T &out, const tf2::Quaternion &quat_world);

  void set_acc(geometry_msgs::msg::Vector3 &out);

  void publish_odometry();

  static double point_distance(const geometry_msgs::msg::Point &A, const geometry_msgs::msg::Point &B) {

    return std::sqrt(std::pow(A.x - B.x, 2) + std::pow(A.y - B.y, 2) + std::pow(A.z - B.z, 2));
  }

  double max_angular_distance(const geometry_msgs::msg::Quaternion &q_from, const geometry_msgs::msg::Quaternion &q_to);

  void publish_path();

  BoxPointType LocalMap_Points;
  bool         Localmap_Initialized = false;

  int points_cache_size = 0;

  const float MOV_THRESHOLD = 1.5f;

  mutex              mtx_buffer;
  condition_variable sig_buffer;

  int feats_down_size = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;

  int    frame_ct         = 0;
  double time_update_last = 0.0, time_current = 0.0, time_predict_last_const = 0.0, t_last = 0.0;

  shared_ptr<ImuProcess> p_imu;
  bool                   init_map = false, flg_first_scan = true;
  PointCloudXYZI::Ptr    ptr_con;

  // Time Log Variables
  double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot11[MAXN];
  double match_time = 0, solve_time = 0, propag_time = 0, update_time = 0;

  bool lidar_pushed = false, flg_reset = false, flg_exit = false;

  vector<BoxPointType> cub_needrm;

  deque<PointCloudXYZI::Ptr>                   lidar_buffer;
  deque<double>                                time_buffer;
  deque<sensor_msgs::msg::Imu::ConstSharedPtr> imu_deque;

  // surf feature in map
  PointCloudXYZI::Ptr feats_undistort;
  PointCloudXYZI::Ptr feats_down_body_space;
  PointCloudXYZI::Ptr init_feats_world;

  pcl::VoxelGrid<PointType> downSizeFilterSurf;
  pcl::VoxelGrid<PointType> downSizeFilterMap;

  V3D euler_cur;

  MeasureGroup Measures;

  sensor_msgs::msg::Imu                 imu_last, imu_next;
  sensor_msgs::msg::Imu::ConstSharedPtr imu_last_ptr;
  nav_msgs::msg::Path                   path;
  nav_msgs::msg::Odometry               odomAftMapped;
  geometry_msgs::msg::Vector3Stamped    accAftMapped;
  geometry_msgs::msg::PoseStamped       msg_body_pose;
  geometry_msgs::msg::Pose              msg_body_pose_prev;

  // Frame names
  string init_frame;
  string odom_frame;
  string lidar_frame;

  Eigen::Matrix<double, 30, 30> Q_output;
  Eigen::Matrix<double, 24, 24> Q_input;


  int frame_num = 0;

  double aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_solve = 0, aver_time_propag = 0;

  double FOV_DEG;
  double HALF_FOV_COS;

  // | ----------------------- publishers ----------------------- |

  mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2>      ph_laser_cloud_full_res_;
  mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2>      ph_laser_cloud_full_res_body_;
  mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2>      ph_laser_cloud_map_;
  mrs_lib::PublisherHandler<nav_msgs::msg::Odometry>            ph_odom_aft_mapped_;
  mrs_lib::PublisherHandler<geometry_msgs::msg::Vector3Stamped> ph_acc_aft_mapped_;
  mrs_lib::PublisherHandler<nav_msgs::msg::Path>                ph_path_;

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscriberHandler<livox_ros_driver2::msg::CustomMsg> sh_livox_;
  mrs_lib::SubscriberHandler<sensor_msgs::msg::PointCloud2>     sh_pc_;
  mrs_lib::SubscriberHandler<sensor_msgs::msg::Imu>             sh_imu_;

  // | ------------------------- timers ------------------------- |

  std::shared_ptr<mrs_lib::ThreadTimer> timer_main_;

  void timerMain();

  // | ------------------ transform broadcaster ----------------- |

  std::shared_ptr<mrs_lib::TransformBroadcaster> tf_broadcaster_;
};

PointLio *PointLio::instance = nullptr;

static bool time_list(PointType &x, PointType &y) {
  return (x.curvature < y.curvature);
};

//}

/* PointLio() //{ */

PointLio::PointLio(rclcpp::NodeOptions options) : Node("point_lio", options) {

  this->instance = this;

  initialize();
}

//}

/* initialize() //{ */

void PointLio::initialize() {

  node_  = this->this_node_ptr();
  clock_ = node_->get_clock();

  cbkgrp_subs_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  p_pre.reset(new Preprocess());

  // | ----------------------- load params ---------------------- |

  mrs_lib::ParamLoader param_loader(node_);

  // load custom config

  std::string custom_config_path;
  param_loader.loadParam("custom_config", custom_config_path);

  if (custom_config_path != "") {
    RCLCPP_INFO(node_->get_logger(), "loading custom config '%s", custom_config_path.c_str());
    bool succ = param_loader.addYamlFile(custom_config_path);

    if (!succ) {
      RCLCPP_ERROR(node_->get_logger(), "failed to load custom config");
      rclcpp::shutdown();
      exit(1);
    }
  }

  // load preset

  std::string preset_path;
  param_loader.loadParam("preset", preset_path);

  if (preset_path != "") {
    RCLCPP_INFO(node_->get_logger(), "loading preset config '%s", preset_path.c_str());
    bool succ = param_loader.addYamlFile(preset_path);

    if (!succ) {
      RCLCPP_ERROR(node_->get_logger(), "failed to load preset");
      rclcpp::shutdown();
      exit(1);
    }
  }

  // load other configs

  {
    bool succ = param_loader.addYamlFileFromParam("config");

    if (!succ) {
      RCLCPP_ERROR(node_->get_logger(), "failed to load config");
      rclcpp::shutdown();
      exit(1);
    }
  }

  //                                                                                                            // DEFAULTS FROM ORIGINAL C++ IMPLEMENTATION
  //                                                                                                            DOWN THERE...
  param_loader.loadParam("main_timer/rate", main_timer_rate_);                                                  // 1
  param_loader.loadParam("prop_at_freq_of_imu", prop_at_freq_of_imu);                                           // 1
  param_loader.loadParam("use_imu_as_input", use_imu_as_input);                                                 // 1
  param_loader.loadParam("check_satu", check_satu);                                                             // 1
  param_loader.loadParam("init_map_size", init_map_size);                                                       // 100
  param_loader.loadParam("space_down_sample", space_down_sample);                                               // 1
  param_loader.loadParam("mapping/satu_acc", satu_acc);                                                         // 3.0
  param_loader.loadParam("mapping/satu_gyro", satu_gyro);                                                       // 35.0
  param_loader.loadParam("mapping/acc_norm", acc_norm);                                                         // 1.0
  param_loader.loadParam("mapping/plane_thr", plane_thr);                                                       // 0.05f
  param_loader.loadParam("point_filter_num", p_pre->point_filter_num);                                          // 2
  param_loader.loadParam("common/con_frame", con_frame);                                                        // false
  param_loader.loadParam("common/con_frame_num", con_frame_num);                                                // 1
  param_loader.loadParam("common/cut_frame", cut_frame);                                                        // false
  param_loader.loadParam("common/cut_frame_time_interval", cut_frame_time_interval);                            // 0.1
  param_loader.loadParam("common/time_lag_imu_to_lidar", time_lag_imu_to_lidar);                                // 0.0
  param_loader.loadParam("filter_size_surf", filter_size_surf_min);                                             // 0.5
  param_loader.loadParam("filter_size_map", filter_size_map_min);                                               // 0.5
  param_loader.loadParam("cube_side_length", cube_len);                                                         // 200
  param_loader.loadParam("mapping/det_range", DET_RANGE);                                                       // 300.f
  param_loader.loadParam("mapping/fov_degree", fov_deg);                                                        // 180
  param_loader.loadParam("mapping/imu_en", imu_en);                                                             // true
  param_loader.loadParam("mapping/start_in_aggressive_motion", non_station_start);                              // false
  param_loader.loadParam("mapping/extrinsic_est_en", extrinsic_est_en);                                         // true
  param_loader.loadParam("mapping/imu_time_inte", imu_time_inte);                                               // 0.005
  param_loader.loadParam("mapping/lidar_meas_cov", laser_point_cov);                                            // 0.1
  param_loader.loadParam("mapping/acc_cov_input", acc_cov_input);                                               // 0.1
  param_loader.loadParam("mapping/vel_cov", vel_cov);                                                           // 20
  param_loader.loadParam("mapping/gyr_cov_input", gyr_cov_input);                                               // 0.1
  param_loader.loadParam("mapping/gyr_cov_output", gyr_cov_output);                                             // 0.1
  param_loader.loadParam("mapping/acc_cov_output", acc_cov_output);                                             // 0.1
  param_loader.loadParam("mapping/b_gyr_cov", b_gyr_cov);                                                       // 0.0001
  param_loader.loadParam("mapping/b_acc_cov", b_acc_cov);                                                       // 0.0001
  param_loader.loadParam("mapping/imu_meas_acc_cov", imu_meas_acc_cov);                                         // 0.1
  param_loader.loadParam("mapping/imu_meas_omg_cov", imu_meas_omg_cov);                                         // 0.1
  param_loader.loadParam("preprocess/blind", p_pre->blind);                                                     // 1.0
  param_loader.loadParam("preprocess/lidar_type", lidar_type);                                                  // 1
  param_loader.loadParam("preprocess/scan_line", p_pre->N_SCANS);                                               // 16
  param_loader.loadParam("preprocess/scan_rate", p_pre->SCAN_RATE);                                             // 10
  param_loader.loadParam("preprocess/timestamp_unit", p_pre->time_unit);                                        // 1
  param_loader.loadParam("mapping/match_s", match_s);                                                           // 81
  param_loader.loadParam("mapping/gravity_align", gravity_align);                                               // true
  param_loader.loadParam("mapping/gravity", gravity);                                                           // std::vector<double>()
  param_loader.loadParam("mapping/gravity_init", gravity_init);                                                 // std::vector<double>()
  param_loader.loadParam("mapping/extrinsic_T", extrinT);                                                       // std::vector<double>()
  param_loader.loadParam("mapping/extrinsic_R", extrinR);                                                       // std::vector<double>()
  param_loader.loadParam("odometry/publish_odometry_without_downsample", publish_odometry_without_downsample);  // false
  param_loader.loadParam("publish/path/enable", path_en);                                                       // true
  param_loader.loadParam("publish/path/diff_t", path_diff_t);                                                   // 0.1
  param_loader.loadParam("publish/path/diff_R", path_diff_R);                                                   // 0.1
  param_loader.loadParam("publish/scan_publish_en", scan_pub_en);                                               // 1
  param_loader.loadParam("publish/scan_bodyframe_pub_en", scan_body_pub_en);                                    // 1
  param_loader.loadParam("publish/tf/fcu", _publish_fcu_tf_);                                                   // "false"
  param_loader.loadParam("runtime_pos_log_enable", runtime_pos_log);                                            // 0
  param_loader.loadParam("uav_name", uav_name);                                                                 // ""

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "failed to load non-optional parameters!");
    rclcpp::shutdown();
    exit(1);
  }

  p_imu = std::make_shared<ImuProcess>(node_);

  ptr_con               = std::make_shared<PointCloudXYZI>();
  feats_undistort       = std::make_shared<PointCloudXYZI>();
  feats_down_body_space = std::make_shared<PointCloudXYZI>();
  init_feats_world      = std::make_shared<PointCloudXYZI>();

  // | ------------------------- common ------------------------- |

  Eye3d  = M3D::Identity();
  Eye3f  = M3F::Identity();
  Zero3d = V3D(0, 0, 0);
  Zero3f = V3F(0, 0, 0);

  // | ------------------------ Estimator ----------------------- |

  normvec          = std::make_shared<PointCloudXYZI>(100000, 1);
  feats_down_body  = std::make_shared<PointCloudXYZI>();
  feats_down_world = std::make_shared<PointCloudXYZI>();
  pointSearchSqDis = std::vector<float>(NUM_MATCH_POINTS);

  Lidar_T_wrt_IMU = Zero3d;
  Lidar_R_wrt_IMU = Eye3d;

  // | ----------------------- publishers ----------------------- |

  if (scan_pub_en) {
    ph_laser_cloud_full_res_ = mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2>(node_, "~/cloud_registered_out");
  }

  if (scan_body_pub_en) {
    ph_laser_cloud_full_res_body_ = mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2>(node_, "~/cloud_registered_body_out");
  }

  ph_odom_aft_mapped_ = mrs_lib::PublisherHandler<nav_msgs::msg::Odometry>(node_, "~/odometry_out");
  ph_acc_aft_mapped_  = mrs_lib::PublisherHandler<geometry_msgs::msg::Vector3Stamped>(node_, "~/linear_acceleration_out");

  if (path_en) {
    ph_path_ = mrs_lib::PublisherHandler<nav_msgs::msg::Path>(node_, "~/path_out");
  }

  {
    mrs_lib::PublisherHandlerOptions ph_options;

    ph_options.node = node_;

    rclcpp::QoS qos_profile = rclcpp::SystemDefaultsQoS();
    qos_profile.transient_local();

    ph_options.qos = qos_profile;

    ph_laser_cloud_map_ = mrs_lib::PublisherHandler<sensor_msgs::msg::PointCloud2>(ph_options, "~/laser_cloud_map_out");
  }

  // | ----------------------- subscribers ---------------------- |

  mrs_lib::SubscriberHandlerOptions shopts;

  shopts.node                                = node_;
  shopts.no_message_timeout                  = mrs_lib::no_timeout;
  shopts.threadsafe                          = true;
  shopts.autostart                           = true;
  shopts.subscription_options.callback_group = cbkgrp_subs_;
  shopts.qos                                 = rclcpp::SensorDataQoS();

  sh_imu_   = mrs_lib::SubscriberHandler<sensor_msgs::msg::Imu>(shopts, "~/imu_in", &PointLio::callbackIMU, this);
  sh_pc_    = mrs_lib::SubscriberHandler<sensor_msgs::msg::PointCloud2>(shopts, "~/pc_in", &PointLio::callbackStandardPC, this);
  sh_livox_ = mrs_lib::SubscriberHandler<livox_ros_driver2::msg::CustomMsg>(shopts, "~/livox_in", &PointLio::callbackLivox, this);

  // | ------------------ transform broadcaster ----------------- |

  tf_broadcaster_ = std::make_shared<mrs_lib::TransformBroadcaster>(node_);

  // | ------------------------ old main ------------------------ |

  cout << "lidar_type: " << lidar_type << endl;
  init_frame = uav_name + "/" + "point_lio_origin";
  odom_frame = uav_name + "/" + "fcu";

  /* path.header.stamp    = ros::Time().fromSec(lidar_end_time); */

  const double secs     = floor(lidar_end_time);
  const double nanosecs = (lidar_end_time - secs) * 1e9;

  path.header.stamp = rclcpp::Time(secs, nanosecs, clock_->get_clock_type());

  path.header.frame_id = init_frame;
  /*** variables definition for counting ***/

  /*** initialize variables ***/
  FOV_DEG      = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
  HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

  memset(point_selected_surf, true, sizeof(point_selected_surf));

  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
  downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);

  Lidar_T_wrt_IMU << VEC_FROM_ARRAY(extrinT);
  Lidar_R_wrt_IMU << MAT_FROM_ARRAY(extrinR);

  if (extrinsic_est_en) {
    if (!use_imu_as_input) {
      kf_output.x_.offset_R_L_I = Lidar_R_wrt_IMU;
      kf_output.x_.offset_T_L_I = Lidar_T_wrt_IMU;
    } else {
      kf_input.x_.offset_R_L_I = Lidar_R_wrt_IMU;
      kf_input.x_.offset_T_L_I = Lidar_T_wrt_IMU;
    }
  }

  p_imu->lidar_type = p_pre->lidar_type = lidar_type;
  p_imu->imu_en                         = imu_en;

  kf_input.init_dyn_share_modified(PointLio::get_f_input_static, PointLio::df_dx_input_static, PointLio::h_model_input_static);

  kf_output.init_dyn_share_modified_2h(PointLio::get_f_output_static, PointLio::df_dx_output_static, PointLio::h_model_output_static,
                                       PointLio::h_model_IMU_output_static);

  Eigen::Matrix<double, 24, 24> P_init = MD(24, 24)::Identity() * 0.01;
  P_init.block<3, 3>(21, 21)           = MD(3, 3)::Identity() * 0.0001;
  P_init.block<6, 6>(15, 15)           = MD(6, 6)::Identity() * 0.001;
  P_init.block<6, 6>(6, 6)             = MD(6, 6)::Identity() * 0.0001;
  kf_input.change_P(P_init);
  Eigen::Matrix<double, 30, 30> P_init_output = MD(30, 30)::Identity() * 0.01;
  P_init_output.block<3, 3>(21, 21)           = MD(3, 3)::Identity() * 0.0001;
  P_init_output.block<6, 6>(6, 6)             = MD(6, 6)::Identity() * 0.0001;
  P_init_output.block<6, 6>(24, 24)           = MD(6, 6)::Identity() * 0.001;
  kf_input.change_P(P_init);
  kf_output.change_P(P_init_output);
  Q_input  = process_noise_cov_input();
  Q_output = process_noise_cov_output();

  // | ------------------------- timers ------------------------- |

  mrs_lib::TimerHandlerOptions timer_opts_start;

  timer_opts_start.node      = node_;
  timer_opts_start.autostart = true;

  {
    std::function<void()> callback_fcn = std::bind(&PointLio::timerMain, this);

    timer_main_ = std::make_shared<mrs_lib::ThreadTimer>(timer_opts_start, rclcpp::Rate(main_timer_rate_, clock_), callback_fcn);
  }

  // | --------------------- finish the init -------------------- |

  is_initialized_ = true;

  RCLCPP_INFO(node_->get_logger(), "initialized");
}

//}

/* original functions from the estimator.cpp //{ */

Eigen::Matrix<double, 24, 24> PointLio::process_noise_cov_input() {

  Eigen::Matrix<double, 24, 24> cov;

  cov.setZero();
  cov.block<3, 3>(3, 3).diagonal() << gyr_cov_input, gyr_cov_input, gyr_cov_input;
  cov.block<3, 3>(12, 12).diagonal() << acc_cov_input, acc_cov_input, acc_cov_input;
  cov.block<3, 3>(15, 15).diagonal() << b_gyr_cov, b_gyr_cov, b_gyr_cov;
  cov.block<3, 3>(18, 18).diagonal() << b_acc_cov, b_acc_cov, b_acc_cov;
  // MTK::get_cov<process_noise_input>::type cov = MTK::get_cov<process_noise_input>::type::Zero();
  // MTK::setDiagonal<process_noise_input, vect3, 0>(cov, &process_noise_input::ng, gyr_cov_input);// 0.03
  // MTK::setDiagonal<process_noise_input, vect3, 3>(cov, &process_noise_input::na, acc_cov_input); // *dt 0.01 0.01 * dt * dt 0.05
  // MTK::setDiagonal<process_noise_input, vect3, 6>(cov, &process_noise_input::nbg, b_gyr_cov); // *dt 0.00001 0.00001 * dt *dt 0.3 //0.001 0.0001 0.01
  // MTK::setDiagonal<process_noise_input, vect3, 9>(cov, &process_noise_input::nba, b_acc_cov);   //0.001 0.05 0.0001/out 0.01
  return cov;
}

Eigen::Matrix<double, 30, 30> PointLio::process_noise_cov_output() {
  Eigen::Matrix<double, 30, 30> cov;
  cov.setZero();
  cov.block<3, 3>(12, 12).diagonal() << vel_cov, vel_cov, vel_cov;
  cov.block<3, 3>(15, 15).diagonal() << gyr_cov_output, gyr_cov_output, gyr_cov_output;
  cov.block<3, 3>(18, 18).diagonal() << acc_cov_output, acc_cov_output, acc_cov_output;
  cov.block<3, 3>(24, 24).diagonal() << b_gyr_cov, b_gyr_cov, b_gyr_cov;
  cov.block<3, 3>(27, 27).diagonal() << b_acc_cov, b_acc_cov, b_acc_cov;
  // MTK::get_cov<process_noise_output>::type cov = MTK::get_cov<process_noise_output>::type::Zero();
  // MTK::setDiagonal<process_noise_output, vect3, 0>(cov, &process_noise_output::vel, vel_cov);// 0.03
  // MTK::setDiagonal<process_noise_output, vect3, 3>(cov, &process_noise_output::ng, gyr_cov_output); // *dt 0.01 0.01 * dt * dt 0.05
  // MTK::setDiagonal<process_noise_output, vect3, 6>(cov, &process_noise_output::na, acc_cov_output); // *dt 0.00001 0.00001 * dt *dt 0.3 //0.001 0.0001 0.01
  // MTK::setDiagonal<process_noise_output, vect3, 9>(cov, &process_noise_output::nbg, b_gyr_cov);   //0.001 0.05 0.0001/out 0.01
  // MTK::setDiagonal<process_noise_output, vect3, 12>(cov, &process_noise_output::nba, b_acc_cov);   //0.001 0.05 0.0001/out 0.01
  return cov;
}

Eigen::Matrix<double, 24, 1> PointLio::get_f_input(state_input &s, const input_ikfom &in) {

  Eigen::Matrix<double, 24, 1> res = Eigen::Matrix<double, 24, 1>::Zero();

  vect3 omega;
  in.gyro.boxminus(omega, s.bg);
  vect3 a_inertial = s.rot * (in.acc - s.ba);
  for (int i = 0; i < 3; i++) {
    res(i)      = s.vel[i];
    res(i + 3)  = omega[i];
    res(i + 12) = a_inertial[i] + s.gravity[i];
  }
  return res;
}

Eigen::Matrix<double, 30, 1> PointLio::get_f_output(state_output &s, [[maybe_unused]] const input_ikfom &in) {
  Eigen::Matrix<double, 30, 1> res        = Eigen::Matrix<double, 30, 1>::Zero();
  vect3                        a_inertial = s.rot * s.acc;
  for (int i = 0; i < 3; i++) {
    res(i)      = s.vel[i];
    res(i + 3)  = s.omg[i];
    res(i + 12) = a_inertial[i] + s.gravity[i];
  }
  return res;
}

Eigen::Matrix<double, 24, 24> PointLio::df_dx_input(state_input &s, const input_ikfom &in) {
  Eigen::Matrix<double, 24, 24> cov = Eigen::Matrix<double, 24, 24>::Zero();
  cov.template block<3, 3>(0, 12)   = Eigen::Matrix3d::Identity();
  vect3 acc_;
  in.acc.boxminus(acc_, s.ba);
  vect3 omega;
  in.gyro.boxminus(omega, s.bg);
  cov.template block<3, 3>(12, 3)  = -s.rot * MTK::hat(acc_);
  cov.template block<3, 3>(12, 18) = -s.rot;
  // Eigen::Matrix<state_ikfom::scalar, 2, 1> vec = Eigen::Matrix<state_ikfom::scalar, 2, 1>::Zero();
  // Eigen::Matrix<state_ikfom::scalar, 3, 2> grav_matrix;
  // s.S2_Mx(grav_matrix, vec, 21);
  cov.template block<3, 3>(12, 21) = Eigen::Matrix3d::Identity();  // grav_matrix;
  cov.template block<3, 3>(3, 15)  = -Eigen::Matrix3d::Identity();
  return cov;
}

// Eigen::Matrix<double, 24, 12> df_dw_input(state_input &s, const input_ikfom &in)
// {
// 	Eigen::Matrix<double, 24, 12> cov = Eigen::Matrix<double, 24, 12>::Zero();
// 	cov.template block<3, 3>(12, 3) = -s.rot.normalized().toRotationMatrix();
// 	cov.template block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
// 	cov.template block<3, 3>(15, 6) = Eigen::Matrix3d::Identity();
// 	cov.template block<3, 3>(18, 9) = Eigen::Matrix3d::Identity();
// 	return cov;
// }

Eigen::Matrix<double, 30, 30> PointLio::df_dx_output(state_output &s, [[maybe_unused]] const input_ikfom &in) {

  Eigen::Matrix<double, 30, 30> cov = Eigen::Matrix<double, 30, 30>::Zero();
  cov.template block<3, 3>(0, 12)   = Eigen::Matrix3d::Identity();
  cov.template block<3, 3>(12, 3)   = -s.rot * MTK::hat(s.acc);
  cov.template block<3, 3>(12, 18)  = s.rot;
  // Eigen::Matrix<state_ikfom::scalar, 2, 1> vec = Eigen::Matrix<state_ikfom::scalar, 2, 1>::Zero();
  // Eigen::Matrix<state_ikfom::scalar, 3, 2> grav_matrix;
  // s.S2_Mx(grav_matrix, vec, 21);
  cov.template block<3, 3>(12, 21) = Eigen::Matrix3d::Identity();  // grav_matrix;
  cov.template block<3, 3>(3, 15)  = Eigen::Matrix3d::Identity();
  return cov;
}

// Eigen::Matrix<double, 30, 15> df_dw_output(state_output &s)
// {
// 	Eigen::Matrix<double, 30, 15> cov = Eigen::Matrix<double, 30, 15>::Zero();
// 	cov.template block<3, 3>(12, 0) = Eigen::Matrix3d::Identity();
// 	cov.template block<3, 3>(15, 3) = Eigen::Matrix3d::Identity();
// 	cov.template block<3, 3>(18, 6) = Eigen::Matrix3d::Identity();
// 	cov.template block<3, 3>(24, 9) = Eigen::Matrix3d::Identity();
// 	cov.template block<3, 3>(27, 12) = Eigen::Matrix3d::Identity();
// 	return cov;
// }

vect3 PointLio::SO3ToEuler(const SO3 &rot) {

  // Eigen::Matrix<double, 3, 1> _ang;
  // Eigen::Vector4d q_data = orient.coeffs().transpose();
  // //scalar w=orient.coeffs[3], x=orient.coeffs[0], y=orient.coeffs[1], z=orient.coeffs[2];
  // double sqw = q_data[3]*q_data[3];
  // double sqx = q_data[0]*q_data[0];
  // double sqy = q_data[1]*q_data[1];
  // double sqz = q_data[2]*q_data[2];
  // double unit = sqx + sqy + sqz + sqw; // if normalized is one, otherwise is correction factor
  // double test = q_data[3]*q_data[1] - q_data[2]*q_data[0];

  // if (test > 0.49999*unit) { // singularity at north pole

  // 	_ang << 2 * std::atan2(q_data[0], q_data[3]), M_PI/2, 0;
  // 	double temp[3] = {_ang[0] * 57.3, _ang[1] * 57.3, _ang[2] * 57.3};
  // 	vect3 euler_ang(temp, 3);
  // 	return euler_ang;
  // }
  // if (test < -0.49999*unit) { // singularity at south pole
  // 	_ang << -2 * std::atan2(q_data[0], q_data[3]), -M_PI/2, 0;
  // 	double temp[3] = {_ang[0] * 57.3, _ang[1] * 57.3, _ang[2] * 57.3};
  // 	vect3 euler_ang(temp, 3);
  // 	return euler_ang;
  // }

  // _ang <<
  // 		std::atan2(2*q_data[0]*q_data[3]+2*q_data[1]*q_data[2] , -sqx - sqy + sqz + sqw),
  // 		std::asin (2*test/unit),
  // 		std::atan2(2*q_data[2]*q_data[3]+2*q_data[1]*q_data[0] , sqx - sqy - sqz + sqw);
  // double temp[3] = {_ang[0] * 57.3, _ang[1] * 57.3, _ang[2] * 57.3};
  // vect3 euler_ang(temp, 3);
  // return euler_ang;
  double sy       = sqrt(rot(0, 0) * rot(0, 0) + rot(1, 0) * rot(1, 0));
  bool   singular = sy < 1e-6;
  double x, y, z;
  if (!singular) {
    x = atan2(rot(2, 1), rot(2, 2));
    y = atan2(-rot(2, 0), sy);
    z = atan2(rot(1, 0), rot(0, 0));
  } else {
    x = atan2(-rot(1, 2), rot(1, 1));
    y = atan2(-rot(2, 0), sy);
    z = 0;
  }
  Eigen::Matrix<double, 3, 1> ang(x, y, z);
  return ang;
}

void PointLio::h_model_input(state_input &s, esekfom::dyn_share_modified<double> &ekfom_data) {

  VF(4) pabcd;

  pabcd.setZero();

  normvec->resize(time_seq[k]);

  int effect_num_k = 0;

  for (int j = 0; j < time_seq[k]; j++) {

    PointType &point_body_j  = feats_down_body->points[idx + j + 1];
    PointType &point_world_j = feats_down_world->points[idx + j + 1];
    pointBodyToWorld(&point_body_j, &point_world_j);
    V3D p_body = pbody_list[idx + j + 1];
    V3D p_world;
    p_world << point_world_j.x, point_world_j.y, point_world_j.z;

    {
      PointVector &points_near = Nearest_Points[idx + j + 1];

      ikdtree.Nearest_Search(point_world_j, NUM_MATCH_POINTS, points_near, pointSearchSqDis, 2.236);  // 1.0); //, 3.0); // 2.236;

      if ((points_near.size() < NUM_MATCH_POINTS) || pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5)  // 5)
      {
        point_selected_surf[idx + j + 1] = false;
      } else {
        point_selected_surf[idx + j + 1] = false;
        if (esti_plane(pabcd, points_near, plane_thr))  //(planeValid)
        {
          float pd2 = pabcd(0) * point_world_j.x + pabcd(1) * point_world_j.y + pabcd(2) * point_world_j.z + pabcd(3);

          if (p_body.norm() > match_s * pd2 * pd2) {
            point_selected_surf[idx + j + 1] = true;
            normvec->points[j].x             = pabcd(0);
            normvec->points[j].y             = pabcd(1);
            normvec->points[j].z             = pabcd(2);
            normvec->points[j].intensity     = pabcd(3);
            effect_num_k++;
          }
        }
      }
    }
  }
  if (effect_num_k == 0) {
    ekfom_data.valid = false;
    return;
  }
  ekfom_data.M_Noise = laser_point_cov;
  ekfom_data.h_x     = Eigen::MatrixXd::Zero(effect_num_k, 12);
  ekfom_data.z.resize(effect_num_k);
  int m = 0;

  for (int j = 0; j < time_seq[k]; j++) {
    if (point_selected_surf[idx + j + 1]) {
      V3D norm_vec(normvec->points[j].x, normvec->points[j].y, normvec->points[j].z);

      if (extrinsic_est_en) {
        V3D p_body = pbody_list[idx + j + 1];
        M3D p_crossmat, p_imu_crossmat;
        p_crossmat << SKEW_SYM_MATRX(p_body);
        V3D point_imu = s.offset_R_L_I * p_body + s.offset_T_L_I;
        p_imu_crossmat << SKEW_SYM_MATRX(point_imu);
        V3D C(s.rot.transpose() * norm_vec);
        V3D A(p_imu_crossmat * C);
        V3D B(p_crossmat * s.offset_R_L_I.transpose() * C);
        ekfom_data.h_x.block<1, 12>(m, 0) << norm_vec(0), norm_vec(1), norm_vec(2), VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
      } else {
        M3D point_crossmat = crossmat_list[idx + j + 1];
        V3D C(s.rot.transpose() * norm_vec);
        V3D A(point_crossmat * C);
        ekfom_data.h_x.block<1, 12>(m, 0) << norm_vec(0), norm_vec(1), norm_vec(2), VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
      }
      ekfom_data.z(m) = -norm_vec(0) * feats_down_world->points[idx + j + 1].x - norm_vec(1) * feats_down_world->points[idx + j + 1].y -
                        norm_vec(2) * feats_down_world->points[idx + j + 1].z - normvec->points[j].intensity;
      m++;
    }
  }
  effct_feat_num += effect_num_k;
}

void PointLio::h_model_output(state_output &s, esekfom::dyn_share_modified<double> &ekfom_data) {

  VF(4) pabcd;
  pabcd.setZero();

  normvec->resize(time_seq[k]);
  int effect_num_k = 0;

  for (int j = 0; j < time_seq[k]; j++) {

    PointType &point_body_j  = feats_down_body->points[idx + j + 1];
    PointType &point_world_j = feats_down_world->points[idx + j + 1];
    pointBodyToWorld(&point_body_j, &point_world_j);
    V3D p_body = pbody_list[idx + j + 1];
    V3D p_world;
    p_world << point_world_j.x, point_world_j.y, point_world_j.z;

    {
      PointVector &points_near = Nearest_Points[idx + j + 1];

      ikdtree.Nearest_Search(point_world_j, NUM_MATCH_POINTS, points_near, pointSearchSqDis, 2.236);

      if ((points_near.size() < NUM_MATCH_POINTS) || pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5) {
        point_selected_surf[idx + j + 1] = false;
      } else {
        point_selected_surf[idx + j + 1] = false;
        if (esti_plane(pabcd, points_near, plane_thr))  //(planeValid)
        {
          float pd2 = pabcd(0) * point_world_j.x + pabcd(1) * point_world_j.y + pabcd(2) * point_world_j.z + pabcd(3);

          if (p_body.norm() > match_s * pd2 * pd2) {
            // point_selected_surf[i] = true;
            point_selected_surf[idx + j + 1] = true;
            normvec->points[j].x             = pabcd(0);
            normvec->points[j].y             = pabcd(1);
            normvec->points[j].z             = pabcd(2);
            normvec->points[j].intensity     = pabcd(3);
            effect_num_k++;
          }
        }
      }
    }
  }

  if (effect_num_k == 0) {
    ekfom_data.valid = false;
    return;
  }

  ekfom_data.M_Noise = laser_point_cov;
  ekfom_data.h_x     = Eigen::MatrixXd::Zero(effect_num_k, 12);
  ekfom_data.z.resize(effect_num_k);
  int m = 0;

  for (int j = 0; j < time_seq[k]; j++) {

    if (point_selected_surf[idx + j + 1]) {

      V3D norm_vec(normvec->points[j].x, normvec->points[j].y, normvec->points[j].z);

      if (extrinsic_est_en) {
        V3D p_body = pbody_list[idx + j + 1];
        M3D p_crossmat, p_imu_crossmat;
        p_crossmat << SKEW_SYM_MATRX(p_body);
        V3D point_imu = s.offset_R_L_I * p_body + s.offset_T_L_I;
        p_imu_crossmat << SKEW_SYM_MATRX(point_imu);
        V3D C(s.rot.transpose() * norm_vec);
        V3D A(p_imu_crossmat * C);
        V3D B(p_crossmat * s.offset_R_L_I.transpose() * C);
        ekfom_data.h_x.block<1, 12>(m, 0) << norm_vec(0), norm_vec(1), norm_vec(2), VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
      } else {
        M3D point_crossmat = crossmat_list[idx + j + 1];
        V3D C(s.rot.transpose() * norm_vec);
        V3D A(point_crossmat * C);
        // V3D A(point_crossmat * state.rot_end.transpose() * norm_vec);
        ekfom_data.h_x.block<1, 12>(m, 0) << norm_vec(0), norm_vec(1), norm_vec(2), VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
      }
      ekfom_data.z(m) = -norm_vec(0) * feats_down_world->points[idx + j + 1].x - norm_vec(1) * feats_down_world->points[idx + j + 1].y -
                        norm_vec(2) * feats_down_world->points[idx + j + 1].z - normvec->points[j].intensity;
      m++;
    }
  }

  effct_feat_num += effect_num_k;
}

void PointLio::h_model_IMU_output(state_output &s, esekfom::dyn_share_modified<double> &ekfom_data) {

  std::memset(ekfom_data.satu_check, false, 6);

  ekfom_data.z_IMU.block<3, 1>(0, 0) = angvel_avr - s.omg - s.bg;
  ekfom_data.z_IMU.block<3, 1>(3, 0) = acc_avr * G_m_s2 / acc_norm - s.acc - s.ba;
  ekfom_data.R_IMU << imu_meas_omg_cov, imu_meas_omg_cov, imu_meas_omg_cov, imu_meas_acc_cov, imu_meas_acc_cov, imu_meas_acc_cov;

  if (check_satu) {
    if (fabs(angvel_avr(0)) >= 0.99 * satu_gyro) {
      ekfom_data.satu_check[0] = true;
      ekfom_data.z_IMU(0)      = 0.0;
    }

    if (fabs(angvel_avr(1)) >= 0.99 * satu_gyro) {
      ekfom_data.satu_check[1] = true;
      ekfom_data.z_IMU(1)      = 0.0;
    }

    if (fabs(angvel_avr(2)) >= 0.99 * satu_gyro) {
      ekfom_data.satu_check[2] = true;
      ekfom_data.z_IMU(2)      = 0.0;
    }

    if (fabs(acc_avr(0)) >= 0.99 * satu_acc) {
      ekfom_data.satu_check[3] = true;
      ekfom_data.z_IMU(3)      = 0.0;
    }

    if (fabs(acc_avr(1)) >= 0.99 * satu_acc) {
      ekfom_data.satu_check[4] = true;
      ekfom_data.z_IMU(4)      = 0.0;
    }

    if (fabs(acc_avr(2)) >= 0.99 * satu_acc) {
      ekfom_data.satu_check[5] = true;
      ekfom_data.z_IMU(5)      = 0.0;
    }
  }
}

void PointLio::pointBodyToWorld(PointType const *const pi, PointType *const po) {

  V3D p_body(pi->x, pi->y, pi->z);

  V3D p_global;

  if (extrinsic_est_en) {

    if (!use_imu_as_input) {
      p_global = kf_output.x_.rot * (kf_output.x_.offset_R_L_I * p_body + kf_output.x_.offset_T_L_I) + kf_output.x_.pos;
    } else {
      p_global = kf_input.x_.rot * (kf_input.x_.offset_R_L_I * p_body + kf_input.x_.offset_T_L_I) + kf_input.x_.pos;
    }

  } else {
    if (!use_imu_as_input) {
      p_global = kf_output.x_.rot * (Lidar_R_wrt_IMU * p_body + Lidar_T_wrt_IMU) + kf_output.x_.pos;
    } else {
      p_global = kf_input.x_.rot * (Lidar_R_wrt_IMU * p_body + Lidar_T_wrt_IMU) + kf_input.x_.pos;
    }
  }

  po->x         = p_global(0);
  po->y         = p_global(1);
  po->z         = p_global(2);
  po->intensity = pi->intensity;
}

//}

/* pointBodyLidarToIMU() //{ */

void PointLio::pointBodyLidarToIMU(PointType const *const pi, PointType *const po) {

  V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_imu;

  if (extrinsic_est_en) {

    if (!use_imu_as_input) {
      p_body_imu = kf_output.x_.offset_R_L_I * p_body_lidar + kf_output.x_.offset_T_L_I;
    } else {
      p_body_imu = kf_input.x_.offset_R_L_I * p_body_lidar + kf_input.x_.offset_T_L_I;
    }

  } else {
    p_body_imu = Lidar_R_wrt_IMU * p_body_lidar + Lidar_T_wrt_IMU;
  }

  po->x         = p_body_imu(0);
  po->y         = p_body_imu(1);
  po->z         = p_body_imu(2);
  po->intensity = pi->intensity;
}

//}

/* points_cache_collect() //{ */

void PointLio::points_cache_collect() {

  PointVector points_history;
  ikdtree.acquire_removed_points(points_history);
  points_cache_size = points_history.size();
}

//}

/* lasermap_fov_segment() //{ */

void PointLio::lasermap_fov_segment() {

  cub_needrm.shrink_to_fit();

  V3D pos_LiD;

  if (use_imu_as_input) {
    pos_LiD = kf_input.x_.pos + kf_input.x_.rot * Lidar_T_wrt_IMU;
  } else {
    pos_LiD = kf_output.x_.pos + kf_output.x_.rot * Lidar_T_wrt_IMU;
  }

  if (!Localmap_Initialized) {

    for (int i = 0; i < 3; i++) {
      LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
      LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
    }

    Localmap_Initialized = true;

    return;
  }

  float dist_to_map_edge[3][2];
  bool  need_move = false;

  for (int i = 0; i < 3; i++) {

    dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
    dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);

    if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) {
      need_move = true;
    }
  }

  if (!need_move) {
    return;
  }

  BoxPointType New_LocalMap_Points, tmp_boxpoints;
  New_LocalMap_Points = LocalMap_Points;
  float mov_dist      = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD - 1)));

  for (int i = 0; i < 3; i++) {

    tmp_boxpoints = LocalMap_Points;

    if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE) {
      New_LocalMap_Points.vertex_max[i] -= mov_dist;
      New_LocalMap_Points.vertex_min[i] -= mov_dist;
      tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
      cub_needrm.emplace_back(tmp_boxpoints);
    } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) {
      New_LocalMap_Points.vertex_max[i] += mov_dist;
      New_LocalMap_Points.vertex_min[i] += mov_dist;
      tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
      cub_needrm.emplace_back(tmp_boxpoints);
    }
  }

  LocalMap_Points = New_LocalMap_Points;

  points_cache_collect();

  if (cub_needrm.size() > 0) {
    ikdtree.Delete_Point_Boxes(cub_needrm);
  }
}

//}

/* callbackStandardPC() //{ */

void PointLio::callbackStandardPC(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "got standard point cloud");

  std::scoped_lock lock(mtx_buffer);

  if (scan_count == 0) {
    lidar_frame = msg->header.frame_id;
  }

  scan_count++;
  double preprocess_start_time = omp_get_wtime();

  if (rclcpp::Time(msg->header.stamp).seconds() < last_timestamp_lidar) {

    RCLCPP_ERROR(node_->get_logger(), "loop back, clear buffer");
    // lidar_buffer.shrink_to_fit();

    return;
  }

  last_timestamp_lidar = rclcpp::Time(msg->header.stamp).seconds();

  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  PointCloudXYZI::Ptr ptr_div(new PointCloudXYZI());
  double              time_div = rclcpp::Time(msg->header.stamp).seconds();
  p_pre->process(msg, ptr);

  if (cut_frame) {

    sort(ptr->points.begin(), ptr->points.end(), time_list);

    for (size_t i = 0; i < ptr->size(); i++) {

      ptr_div->push_back(ptr->points[i]);
      // cout << "check time:" << ptr->points[i].curvature << endl;
      //
      if (ptr->points[i].curvature / double(1000) + rclcpp::Time(msg->header.stamp).seconds() - time_div > cut_frame_time_interval) {

        if (ptr_div->size() < 1)
          continue;

        PointCloudXYZI::Ptr ptr_div_i(new PointCloudXYZI());
        *ptr_div_i = *ptr_div;
        lidar_buffer.push_back(ptr_div_i);
        time_buffer.push_back(time_div);
        time_div += ptr->points[i].curvature / double(1000);
        ptr_div->clear();
      }
    }

    if (!ptr_div->empty()) {
      lidar_buffer.push_back(ptr_div);
      // ptr_div->clear();
      time_buffer.push_back(time_div);
    }

  } else if (con_frame) {

    if (frame_ct == 0) {
      time_con = last_timestamp_lidar;  // msg->header.stamp.toSec();
    }

    if (frame_ct < con_frame_num) {

      for (size_t i = 0; i < ptr->size(); i++) {
        ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
        ptr_con->push_back(ptr->points[i]);
      }

      frame_ct++;

    } else {
      PointCloudXYZI::Ptr ptr_con_i(new PointCloudXYZI());
      *ptr_con_i = *ptr_con;
      lidar_buffer.push_back(ptr_con_i);
      double time_con_i = time_con;
      time_buffer.push_back(time_con_i);
      ptr_con->clear();
      frame_ct = 0;
    }

  } else {
    lidar_buffer.emplace_back(ptr);
    time_buffer.emplace_back(rclcpp::Time(msg->header.stamp).seconds());
  }

  s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
}

//}

/* callbackLivox() //{ */

void PointLio::callbackLivox(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting livox data");

  std::scoped_lock lock(mtx_buffer);

  double preprocess_start_time = omp_get_wtime();

  if (scan_count == 0) {
    lidar_frame = msg->header.frame_id;
  }

  scan_count++;

  if (rclcpp::Time(msg->header.stamp).seconds() < last_timestamp_lidar) {

    RCLCPP_ERROR(node_->get_logger(), "loop back, clear buffer");
    return;
  }

  last_timestamp_lidar = rclcpp::Time(msg->header.stamp).seconds();

  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  PointCloudXYZI::Ptr ptr_div(new PointCloudXYZI());
  p_pre->process(msg, ptr);
  double time_div = rclcpp::Time(msg->header.stamp).seconds();

  if (cut_frame) {

    sort(ptr->points.begin(), ptr->points.end(), time_list);

    for (size_t i = 0; i < ptr->size(); i++) {
      ptr_div->push_back(ptr->points[i]);
      if (ptr->points[i].curvature / double(1000) + rclcpp::Time(msg->header.stamp).seconds() - time_div > cut_frame_time_interval) {
        if (ptr_div->size() < 1)
          continue;
        PointCloudXYZI::Ptr ptr_div_i(new PointCloudXYZI());
        // cout << "ptr div num:" << ptr_div->size() << endl;
        *ptr_div_i = *ptr_div;
        // cout << "ptr div i num:" << ptr_div_i->size() << endl;
        lidar_buffer.push_back(ptr_div_i);
        time_buffer.push_back(time_div);
        time_div += ptr->points[i].curvature / double(1000);
        ptr_div->clear();
      }
    }
    if (!ptr_div->empty()) {
      lidar_buffer.push_back(ptr_div);
      // ptr_div->clear();
      time_buffer.push_back(time_div);
    }
  } else if (con_frame) {

    if (frame_ct == 0) {
      time_con = last_timestamp_lidar;  // msg->header.stamp.toSec();
    }

    if (frame_ct < con_frame_num) {

      for (size_t i = 0; i < ptr->size(); i++) {
        ptr->points[i].curvature += (last_timestamp_lidar - time_con) * 1000;
        ptr_con->push_back(ptr->points[i]);
      }

      frame_ct++;

    } else {
      PointCloudXYZI::Ptr ptr_con_i(new PointCloudXYZI());
      *ptr_con_i        = *ptr_con;
      double time_con_i = time_con;
      lidar_buffer.push_back(ptr_con_i);
      time_buffer.push_back(time_con_i);
      ptr_con->clear();
      frame_ct = 0;
    }

  } else {
    lidar_buffer.emplace_back(ptr);
    time_buffer.emplace_back(rclcpp::Time(msg->header.stamp).seconds());
  }

  s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
}

//}

/* callbackIMU() //{ */

void PointLio::callbackIMU(const sensor_msgs::msg::Imu::ConstSharedPtr msg_in) {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "getting imu");

  publish_count++;

  // Tomas: old and dirty code
  /* sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in)); */
  /* msg->header.stamp = ros::Time().fromSec(msg_in->header.stamp.toSec() - time_lag_imu_to_lidar); */

  double timestamp = rclcpp::Time(msg_in->header.stamp).seconds() - time_lag_imu_to_lidar;

  {
    std::scoped_lock lock(mtx_buffer);

    if (timestamp < last_timestamp_imu) {

      RCLCPP_ERROR(node_->get_logger(), "loop back, clear deque");
      // imu_deque.shrink_to_fit();
      return;
    }

    imu_deque.emplace_back(msg_in);
    last_timestamp_imu = timestamp;
  }
}

//}

/* sync_packages() //{ */

bool PointLio::sync_packages(MeasureGroup &meas) {

  std::scoped_lock lock(mtx_buffer);

  if (!imu_en) {

    if (!lidar_buffer.empty()) {

      meas.lidar          = lidar_buffer.front();
      meas.lidar_beg_time = time_buffer.front();

      time_buffer.pop_front();
      lidar_buffer.pop_front();

      if (meas.lidar->points.size() < 1) {
        cout << "lose lidar" << std::endl;
        return false;
      }

      double end_time = meas.lidar->points.back().curvature;

      for (auto pt : meas.lidar->points) {
        if (pt.curvature > end_time) {
          end_time = pt.curvature;
        }
      }

      lidar_end_time       = meas.lidar_beg_time + end_time / double(1000);
      meas.lidar_last_time = lidar_end_time;
      return true;
    }

    return false;
  }

  if (lidar_buffer.empty() || imu_deque.empty()) {
    return false;
  }

  /*** push a lidar scan ***/
  if (!lidar_pushed) {

    meas.lidar = lidar_buffer.front();

    if (meas.lidar->points.size() < 1) {
      cout << "lose lidar" << endl;
      lidar_buffer.pop_front();
      time_buffer.pop_front();
      return false;
    }

    meas.lidar_beg_time = time_buffer.front();
    double end_time     = meas.lidar->points.back().curvature;

    for (auto pt : meas.lidar->points) {
      if (pt.curvature > end_time) {
        end_time = pt.curvature;
      }
    }

    lidar_end_time = meas.lidar_beg_time + end_time / double(1000);

    meas.lidar_last_time = lidar_end_time;
    lidar_pushed         = true;
  }

  if (last_timestamp_imu < lidar_end_time) {
    return false;
  }

  /*** push imu data, and pop from imu buffer ***/
  if (p_imu->imu_need_init_) {
    double imu_time = rclcpp::Time(imu_deque.front()->header.stamp).seconds();
    meas.imu.shrink_to_fit();
    while ((!imu_deque.empty()) && (imu_time < lidar_end_time)) {
      imu_time = rclcpp::Time(imu_deque.front()->header.stamp).seconds();
      if (imu_time > lidar_end_time)
        break;
      meas.imu.emplace_back(imu_deque.front());
      imu_last     = imu_next;
      imu_last_ptr = imu_deque.front();
      imu_next     = *(imu_deque.front());
      imu_deque.pop_front();
    }

  } else if (!init_map) {

    double imu_time = rclcpp::Time(imu_deque.front()->header.stamp).seconds();
    meas.imu.shrink_to_fit();
    meas.imu.emplace_back(imu_last_ptr);

    while ((!imu_deque.empty()) && (imu_time < lidar_end_time)) {

      imu_time = rclcpp::Time(imu_deque.front()->header.stamp).seconds();

      if (imu_time > lidar_end_time)
        break;

      meas.imu.emplace_back(imu_deque.front());
      imu_last     = imu_next;
      imu_last_ptr = imu_deque.front();
      imu_next     = *(imu_deque.front());
      imu_deque.pop_front();
    }
  }

  lidar_buffer.pop_front();
  time_buffer.pop_front();
  lidar_pushed = false;
  return true;
}

//}

/* map_incremental() //{ */

void PointLio::map_incremental() {

  PointVector PointToAdd;
  PointVector PointNoNeedDownsample;
  PointToAdd.reserve(feats_down_size);
  PointNoNeedDownsample.reserve(feats_down_size);

  for (int i = 0; i < feats_down_size; i++) {

    if (!Nearest_Points[i].empty()) {

      const PointVector &points_near = Nearest_Points[i];
      bool               need_add    = true;
      PointType          mid_point;
      mid_point.x = floor(feats_down_world->points[i].x / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
      mid_point.y = floor(feats_down_world->points[i].y / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;
      mid_point.z = floor(feats_down_world->points[i].z / filter_size_map_min) * filter_size_map_min + 0.5 * filter_size_map_min;

      /* If the nearest points is definitely outside the downsample box */
      if (fabs(points_near[0].x - mid_point.x) > 0.866 * filter_size_map_min || fabs(points_near[0].y - mid_point.y) > 0.866 * filter_size_map_min ||
          fabs(points_near[0].z - mid_point.z) > 0.866 * filter_size_map_min) {
        PointNoNeedDownsample.emplace_back(feats_down_world->points[i]);
        continue;
      }

      /* Check if there is a point already in the downsample box */
      /* float dist = calc_dist<float>(feats_down_world->points[i], mid_point); */

      for (size_t readd_i = 0; readd_i < points_near.size(); readd_i++) {
        /* Those points which are outside the downsample box should not be considered. */
        if (fabs(points_near[readd_i].x - mid_point.x) < 0.5 * filter_size_map_min && fabs(points_near[readd_i].y - mid_point.y) < 0.5 * filter_size_map_min &&
            fabs(points_near[readd_i].z - mid_point.z) < 0.5 * filter_size_map_min) {
          need_add = false;
          break;
        }
      }

      if (need_add) {
        PointToAdd.emplace_back(feats_down_world->points[i]);
      }

    } else {
      // PointToAdd.emplace_back(feats_down_world->points[i]);
      PointNoNeedDownsample.emplace_back(feats_down_world->points[i]);
    }
  }

  ikdtree.Add_Points(PointToAdd, true);
  ikdtree.Add_Points(PointNoNeedDownsample, false);
}

//}

/* publish_init_kdtree() //{ */

void PointLio::publish_init_kdtree(void) {

  PointVector().swap(ikdtree.PCL_Storage);
  ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);

  const int                 size_init_ikdtree = ikdtree.size();
  const PointCloudXYZI::Ptr laserCloudInit(new PointCloudXYZI(size_init_ikdtree, 1));
  laserCloudInit->points = ikdtree.PCL_Storage;

  sensor_msgs::msg::PointCloud2 laserCloudmsg;
  pcl::toROSMsg(*laserCloudInit, laserCloudmsg);

  const double secs     = floor(lidar_end_time);
  const double nanosecs = (lidar_end_time - secs) * 1e9;

  laserCloudmsg.header.stamp    = rclcpp::Time(secs, nanosecs, clock_->get_clock_type());
  laserCloudmsg.header.frame_id = init_frame;

  ph_laser_cloud_map_.publish(laserCloudmsg);
}

//}

/* publish_frame_world() //{ */

void PointLio::publish_frame_world() {

  if (ph_laser_cloud_full_res_.getNumSubscribers() > 0) {

    const int size = feats_down_world->points.size();

    const PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++) {
      // if (i % 3 == 0)
      // {
      laserCloudWorld->points[i].x         = feats_down_world->points[i].x;
      laserCloudWorld->points[i].y         = feats_down_world->points[i].y;
      laserCloudWorld->points[i].z         = feats_down_world->points[i].z;
      laserCloudWorld->points[i].intensity = feats_down_world->points[i].intensity;  // feats_down_world->points[i].y; //
                                                                                     // }
    }
    sensor_msgs::msg::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);

    // Tomas: old from ROS1
    /* laserCloudmsg.header.stamp    = ros::Time().fromSec(lidar_end_time); */

    const double secs     = floor(lidar_end_time);
    const double nanosecs = (lidar_end_time - secs) * 1e9;

    laserCloudmsg.header.stamp = rclcpp::Time(secs, nanosecs, clock_->get_clock_type());

    laserCloudmsg.header.frame_id = init_frame;
    ph_laser_cloud_full_res_.publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
  }
}

//}

/* publish_frame_body() //{ */

void PointLio::publish_frame_body() {

  if (ph_laser_cloud_full_res_body_.getNumSubscribers() > 0) {

    const int                 size = feats_undistort->points.size();
    const PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++) {
      pointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::msg::PointCloud2 laserCloudmsg;

    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);

    // Tomas: old from ROS1
    /* laserCloudmsg.header.stamp    = ros::Time().fromSec(lidar_end_time); */

    const double secs     = floor(lidar_end_time);
    const double nanosecs = (lidar_end_time - secs) * 1e9;

    laserCloudmsg.header.stamp = rclcpp::Time(secs, nanosecs, clock_->get_clock_type());

    laserCloudmsg.header.frame_id = lidar_frame;

    ph_laser_cloud_full_res_body_.publish(laserCloudmsg);

    publish_count -= PUBFRAME_PERIOD;
  }
}

//}

/* set_posestamp() //{ */

template <typename T>
void PointLio::set_posestamp(T &out) {

  if (!use_imu_as_input) {

    out.position.x = kf_output.x_.pos(0);
    out.position.y = kf_output.x_.pos(1);
    out.position.z = kf_output.x_.pos(2);

    Eigen::Quaterniond q(kf_output.x_.rot);

    out.orientation.x = q.coeffs()[0];
    out.orientation.y = q.coeffs()[1];
    out.orientation.z = q.coeffs()[2];
    out.orientation.w = q.coeffs()[3];

  } else {

    out.position.x = kf_input.x_.pos(0);
    out.position.y = kf_input.x_.pos(1);
    out.position.z = kf_input.x_.pos(2);

    Eigen::Quaterniond q(kf_input.x_.rot);

    out.orientation.x = q.coeffs()[0];
    out.orientation.y = q.coeffs()[1];
    out.orientation.z = q.coeffs()[2];
    out.orientation.w = q.coeffs()[3];
  }
}

//}

/* set_odomtwist() //{ */

template <typename T>
void PointLio::set_odomtwist(T &out, const tf2::Quaternion &quat_world) {

  // get velocities in the mapping/world frame
  tf2::Vector3 lin_vel;

  if (!use_imu_as_input) {
    lin_vel = tf2::Vector3(kf_output.x_.vel(0), kf_output.x_.vel(1), kf_output.x_.vel(2));

    out.angular.x = kf_output.x_.omg(0);
    out.angular.y = kf_output.x_.omg(1);
    out.angular.z = kf_output.x_.omg(2);
  } else {
    lin_vel = tf2::Vector3(kf_input.x_.vel(0), kf_input.x_.vel(1), kf_input.x_.vel(2));
  }

  const tf2::Transform tf_world     = tf2::Transform(quat_world, tf2::Vector3(0, 0, 0));
  const tf2::Vector3   lin_vel_body = tf_world.inverse() * lin_vel;

  out.linear.x = lin_vel_body.getX();
  out.linear.y = lin_vel_body.getY();
  out.linear.z = lin_vel_body.getZ();
}

//}

/* set_acc() //{ */

void PointLio::set_acc(geometry_msgs::msg::Vector3 &out) {

  // acceleration is in body frame, but with biases!!
  // TODO: check frame of biases and if they are in body, subtract the biases!

  if (!use_imu_as_input) {
    out.x = kf_output.x_.acc(0);
    out.y = kf_output.x_.acc(1);
    out.z = kf_output.x_.acc(2);
  }
}

//}

/* publish_odometry() //{ */

void PointLio::publish_odometry() {

  odomAftMapped.header.frame_id = init_frame;
  odomAftMapped.child_frame_id  = odom_frame;

  if (publish_odometry_without_downsample) {

    /* odomAftMapped.header.stamp = ros::Time().fromSec(time_current); */

    const double secs     = floor(time_current);
    const double nanosecs = (time_current - secs) * 1e9;

    odomAftMapped.header.stamp = rclcpp::Time(secs, nanosecs, clock_->get_clock_type());

  } else {

    /* odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time); */

    const double secs     = floor(lidar_end_time);
    const double nanosecs = (lidar_end_time - secs) * 1e9;

    odomAftMapped.header.stamp = rclcpp::Time(secs, nanosecs, clock_->get_clock_type());
  }

  set_posestamp(odomAftMapped.pose.pose);

  tf2::Quaternion q_world;
  q_world.setW(odomAftMapped.pose.pose.orientation.w);
  q_world.setX(odomAftMapped.pose.pose.orientation.x);
  q_world.setY(odomAftMapped.pose.pose.orientation.y);
  q_world.setZ(odomAftMapped.pose.pose.orientation.z);

  set_odomtwist(odomAftMapped.twist.twist, q_world);
  ph_odom_aft_mapped_.publish(odomAftMapped);

  if (_publish_fcu_tf_) {

    geometry_msgs::msg::TransformStamped tf;

    tf.header.stamp    = odomAftMapped.header.stamp;
    tf.header.frame_id = init_frame;
    tf.child_frame_id  = odom_frame;

    tf.transform.translation.x = odomAftMapped.pose.pose.position.x;
    tf.transform.translation.y = odomAftMapped.pose.pose.position.y;
    tf.transform.translation.z = odomAftMapped.pose.pose.position.z;

    tf.transform.rotation = mrs_lib::AttitudeConverter(odomAftMapped.pose.pose.orientation);

    tf_broadcaster_->sendTransform(tf);
  }

  accAftMapped.header = odomAftMapped.header;
  set_acc(accAftMapped.vector);

  ph_acc_aft_mapped_.publish(accAftMapped);
}

//}

/* max_angular_distance() //{ */

double PointLio::max_angular_distance(const geometry_msgs::msg::Quaternion &q_from, const geometry_msgs::msg::Quaternion &q_to) {

  // gets max value of RPY angles

  tf2::Quaternion qF, qT;
  qF.setX(q_from.x);
  qF.setY(q_from.y);
  qF.setZ(q_from.z);
  qF.setW(q_from.w);

  qT.setX(q_to.x);
  qT.setY(q_to.y);
  qT.setZ(q_to.z);
  qT.setW(q_to.w);

  // Get rotation from->to
  const tf2::Quaternion qF2T = qT * qF.inverse();

  // Convert to Euler
  double roll, pitch, yaw;
  tf2::Matrix3x3(qF2T).getRPY(roll, pitch, yaw);
  return std::fmax(yaw, std::fmax(roll, pitch));
}

//}

/* publish_path() //{ */

void PointLio::publish_path() {

  set_posestamp(msg_body_pose.pose);

  if (scan_count == 1) {
    msg_body_pose_prev = msg_body_pose.pose;
  } else if (point_distance(msg_body_pose_prev.position, msg_body_pose.pose.position) < path_diff_t &&
             max_angular_distance(msg_body_pose_prev.orientation, msg_body_pose_prev.orientation) < path_diff_R) {
    return;
  }

  // msg_body_pose.header.stamp = ros::Time::now();

  // Tomas: old from ROS1
  /* msg_body_pose.header.stamp    = ros::Time().fromSec(lidar_end_time); */

  const double secs     = floor(lidar_end_time);
  const double nanosecs = (lidar_end_time - secs) * 1e9;

  msg_body_pose.header.stamp = rclcpp::Time(secs, nanosecs, clock_->get_clock_type());

  msg_body_pose.header.frame_id = init_frame;
  path.poses.emplace_back(msg_body_pose);

  msg_body_pose_prev = msg_body_pose.pose;

  ph_path_.publish(path);
}

//}

/* timerMain() //{ */

void PointLio::timerMain() {

  if (!is_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "timerMain() spinning");

  if (sync_packages(Measures)) {

    if (flg_first_scan) {
      first_lidar_time = Measures.lidar_beg_time;
      flg_first_scan   = false;
    }

    if (flg_reset) {
      RCLCPP_WARN(node_->get_logger(), "when rosbag play back");
      p_imu->Reset();
      flg_reset = false;
      return;
    }

    double t0, t1, t2, t3, t4, t5, solve_start;
    match_time  = 0;
    solve_time  = 0;
    propag_time = 0;
    update_time = 0;
    t0          = omp_get_wtime();

    p_imu->Process(Measures, feats_undistort);

    // if (feats_undistort->empty() || feats_undistort == NULL)
    if (p_imu->imu_need_init_) {
      return;
    }

    if (imu_en) {

      if (!p_imu->gravity_align_) {

        while (Measures.lidar_beg_time > rclcpp::Time(imu_next.header.stamp).seconds()) {
          imu_last = imu_next;
          imu_next = *(imu_deque.front());
          imu_deque.pop_front();
          // imu_deque.pop();
        }

        if (non_station_start) {
          state_in.gravity << VEC_FROM_ARRAY(gravity_init);
          state_out.gravity << VEC_FROM_ARRAY(gravity_init);
          state_out.acc << VEC_FROM_ARRAY(gravity_init);
          state_out.acc *= -1;
        } else {
          state_in.gravity  = -1 * p_imu->mean_acc * G_m_s2 / acc_norm;
          state_out.gravity = -1 * p_imu->mean_acc * G_m_s2 / acc_norm;
          state_out.acc     = p_imu->mean_acc * G_m_s2 / acc_norm;
        }

        if (gravity_align) {
          Eigen::Matrix3d rot_init;
          p_imu->gravity_ << VEC_FROM_ARRAY(gravity);
          p_imu->Set_init(state_in.gravity, rot_init);
          state_in.gravity  = p_imu->gravity_;
          state_out.gravity = p_imu->gravity_;
          state_in.rot      = rot_init;
          state_out.rot     = rot_init;
          // state_in.rot.normalize();
          // state_out.rot.normalize();
          state_out.acc = -rot_init.transpose() * state_out.gravity;
        }

        kf_input.change_x(state_in);
        kf_output.change_x(state_out);
        p_imu->gravity_align_ = true;
      }

    } else {

      if (!p_imu->gravity_align_) {

        state_in.gravity << VEC_FROM_ARRAY(gravity_init);

        if (gravity_align) {
          Eigen::Matrix3d rot_init;
          p_imu->gravity_ << VEC_FROM_ARRAY(gravity);
          p_imu->Set_init(state_in.gravity, rot_init);
          state_out.gravity = p_imu->gravity_;
          state_out.rot     = rot_init;
          // state_in.rot.normalize();
          // state_out.rot.normalize();
          state_out.acc = -rot_init.transpose() * state_out.gravity;
        } else {
          state_out.gravity << VEC_FROM_ARRAY(gravity_init);
          state_out.acc << VEC_FROM_ARRAY(gravity_init);
          state_out.acc *= -1;
        }

        // kf_input.change_x(state_in);
        kf_output.change_x(state_out);
        p_imu->gravity_align_ = true;
      }
    }

    /*** Segment the map in lidar FOV ***/
    lasermap_fov_segment();

    /*** downsample the feature points in a scan ***/
    t1 = omp_get_wtime();

    if (space_down_sample) {
      downSizeFilterSurf.setInputCloud(feats_undistort);
      downSizeFilterSurf.filter(*feats_down_body);
      sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
    } else {
      feats_down_body = Measures.lidar;
      sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
    }

    time_seq        = time_compressing<int>(feats_down_body);
    feats_down_size = feats_down_body->points.size();

    /*** initialize the map kdtree ***/
    if (!init_map) {

      if (ikdtree.Root_Node == nullptr)  //
      // if(feats_down_size > 5)
      {
        ikdtree.set_downsample_param(filter_size_map_min);
      }

      feats_down_world->resize(feats_down_size);

      for (int i = 0; i < feats_down_size; i++) {
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
      }

      for (size_t i = 0; i < feats_down_world->size(); i++) {
        init_feats_world->points.emplace_back(feats_down_world->points[i]);
      }

      if (int(init_feats_world->size()) < init_map_size) {
        return;
      }

      ikdtree.Build(init_feats_world->points);
      init_map = true;

      publish_init_kdtree();

      return;
    }

    /*** ICP and Kalman filter update ***/
    normvec->resize(feats_down_size);
    feats_down_world->resize(feats_down_size);

    Nearest_Points.resize(feats_down_size);

    t2 = omp_get_wtime();

    /*** iterated state estimation ***/
    crossmat_list.reserve(feats_down_size);
    pbody_list.reserve(feats_down_size);
    // pbody_ext_list.reserve(feats_down_size);

    for (size_t i = 0; i < feats_down_body->size(); i++) {

      V3D point_this(feats_down_body->points[i].x, feats_down_body->points[i].y, feats_down_body->points[i].z);
      pbody_list[i] = point_this;

      if (extrinsic_est_en) {
        if (!use_imu_as_input) {
          point_this = kf_output.x_.offset_R_L_I * point_this + kf_output.x_.offset_T_L_I;
        } else {
          point_this = kf_input.x_.offset_R_L_I * point_this + kf_input.x_.offset_T_L_I;
        }
      } else {
        point_this = Lidar_R_wrt_IMU * point_this + Lidar_T_wrt_IMU;
      }

      M3D point_crossmat;
      point_crossmat << SKEW_SYM_MATRX(point_this);
      crossmat_list[i] = point_crossmat;
    }

    if (!use_imu_as_input) {

      // bool imu_upda_cov = false;
      effct_feat_num = 0;
      /**** point by point update ****/

      double pcl_beg_time = Measures.lidar_beg_time;
      idx                 = -1;

      for (k = 0; k < time_seq.size(); k++) {

        PointType &point_body = feats_down_body->points[idx + time_seq[k]];

        time_current = point_body.curvature / 1000.0 + pcl_beg_time;

        if (is_first_frame) {

          if (imu_en) {

            while (time_current > rclcpp::Time(imu_next.header.stamp).seconds()) {
              imu_last = imu_next;
              imu_next = *(imu_deque.front());
              imu_deque.pop_front();
              // imu_deque.pop();
            }

            angvel_avr << imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.angular_velocity.z;
            acc_avr << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;
          }

          is_first_frame = false;
          // imu_upda_cov = true;
          time_update_last        = time_current;
          time_predict_last_const = time_current;
        }

        if (imu_en) {

          bool imu_comes = time_current > rclcpp::Time(imu_next.header.stamp).seconds();

          while (imu_comes) {

            // imu_upda_cov = true;
            angvel_avr << imu_next.angular_velocity.x, imu_next.angular_velocity.y, imu_next.angular_velocity.z;
            acc_avr << imu_next.linear_acceleration.x, imu_next.linear_acceleration.y, imu_next.linear_acceleration.z;

            /*** covariance update ***/
            imu_last = imu_next;
            imu_next = *(imu_deque.front());
            imu_deque.pop_front();
            double dt = rclcpp::Time(imu_last.header.stamp).seconds() - time_predict_last_const;
            kf_output.predict(dt, Q_output, input_in, true, false);
            time_predict_last_const = rclcpp::Time(imu_last.header.stamp).seconds();  // big problem
            imu_comes               = time_current > rclcpp::Time(imu_next.header.stamp).seconds();

            // if (!imu_comes)
            {
              double dt_cov = rclcpp::Time(imu_last.header.stamp).seconds() - time_update_last;

              if (dt_cov > 0.0) {
                time_update_last        = rclcpp::Time(imu_last.header.stamp).seconds();
                double propag_imu_start = omp_get_wtime();

                kf_output.predict(dt_cov, Q_output, input_in, false, true);

                propag_time += omp_get_wtime() - propag_imu_start;
                double solve_imu_start = omp_get_wtime();
                kf_output.update_iterated_dyn_share_IMU();
                solve_time += omp_get_wtime() - solve_imu_start;
              }
            }
          }
        }

        double dt                 = time_current - time_predict_last_const;
        double propag_state_start = omp_get_wtime();

        if (!prop_at_freq_of_imu) {
          double dt_cov = time_current - time_update_last;
          if (dt_cov > 0.0) {
            kf_output.predict(dt_cov, Q_output, input_in, false, true);
            time_update_last = time_current;
          }
        }

        kf_output.predict(dt, Q_output, input_in, true, false);
        propag_time += omp_get_wtime() - propag_state_start;
        time_predict_last_const = time_current;
        // if(k == 0)
        // {
        //     fout_imu_pbp << Measures.lidar_last_time - first_lidar_time << " " << imu_last.angular_velocity.x << " " << imu_last.angular_velocity.y << " "
        //     << imu_last.angular_velocity.z \
                    //             << " " << imu_last.linear_acceleration.x << " " << imu_last.linear_acceleration.y << " " << imu_last.linear_acceleration.z << endl;
        // }

        double t_update_start = omp_get_wtime();

        if (feats_down_size < 1) {
          RCLCPP_WARN(node_->get_logger(), "point, skip this scan!\n");
          idx += time_seq[k];
          continue;
        }

        if (!kf_output.update_iterated_dyn_share_modified()) {
          idx = idx + time_seq[k];
          continue;
        }

        // if(prop_at_freq_of_imu)
        // {
        // double dt_cov = time_current - time_update_last;
        // if ((dt_cov >= imu_time_inte)) // (point_cov_not_prop && imu_prop_cov)
        // {
        //     double propag_cov_start = omp_get_wtime();
        //     kf_output.predict(dt_cov, Q_output, input_in, false, true);
        //     // imu_upda_cov = false;
        //     time_update_last = time_current;
        //     propag_time += omp_get_wtime() - propag_cov_start;
        // }
        // }

        solve_start = omp_get_wtime();

        if (publish_odometry_without_downsample) {
          /******* Publish odometry *******/

          publish_odometry();

          if (runtime_pos_log) {
            state_out = kf_output.x_;
            euler_cur = SO3ToEuler(state_out.rot);
          }
        }

        for (int j = 0; j < time_seq[k]; j++) {
          PointType &point_body_j  = feats_down_body->points[idx + j + 1];
          PointType &point_world_j = feats_down_world->points[idx + j + 1];
          pointBodyToWorld(&point_body_j, &point_world_j);
        }

        solve_time += omp_get_wtime() - solve_start;

        update_time += omp_get_wtime() - t_update_start;
        idx += time_seq[k];
        // cout << "pbp output effect feat num:" << effct_feat_num << endl;
      }

    } else {

      bool imu_prop_cov = false;
      effct_feat_num    = 0;

      double pcl_beg_time = Measures.lidar_beg_time;
      idx                 = -1;

      for (k = 0; k < time_seq.size(); k++) {

        PointType &point_body = feats_down_body->points[idx + time_seq[k]];
        time_current          = point_body.curvature / 1000.0 + pcl_beg_time;

        if (is_first_frame) {

          while (time_current > rclcpp::Time(imu_next.header.stamp).seconds()) {
            imu_last = imu_next;
            imu_next = *(imu_deque.front());
            imu_deque.pop_front();
            // imu_deque.pop();
          }

          imu_prop_cov = true;
          // imu_upda_cov = true;

          is_first_frame   = false;
          t_last           = time_current;
          time_update_last = time_current;
          // if(prop_at_freq_of_imu)
          {
            input_in.gyro << imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.angular_velocity.z;

            input_in.acc << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;
            // angvel_avr<<0.5 * (imu_last.angular_velocity.x + imu_next.angular_velocity.x),
            //             0.5 * (imu_last.angular_velocity.y + imu_next.angular_velocity.y),
            //             0.5 * (imu_last.angular_velocity.z + imu_next.angular_velocity.z);

            // acc_avr   <<0.5 * (imu_last.linear_acceleration.x + imu_next.linear_acceleration.x),
            //             0.5 * (imu_last.linear_acceleration.y + imu_next.linear_acceleration.y),
            // 0.5 * (imu_last.linear_acceleration.z + imu_next.linear_acceleration.z);

            // angvel_avr -= state.bias_g;
            input_in.acc = input_in.acc * G_m_s2 / acc_norm;
          }
        }

        while (time_current > rclcpp::Time(imu_next.header.stamp).seconds())  // && !imu_deque.empty())
        {
          imu_last = imu_next;
          imu_next = *(imu_deque.front());
          imu_deque.pop_front();
          input_in.gyro << imu_last.angular_velocity.x, imu_last.angular_velocity.y, imu_last.angular_velocity.z;
          input_in.acc << imu_last.linear_acceleration.x, imu_last.linear_acceleration.y, imu_last.linear_acceleration.z;

          // angvel_avr<<0.5 * (imu_last.angular_velocity.x + imu_next.angular_velocity.x),
          //             0.5 * (imu_last.angular_velocity.y + imu_next.angular_velocity.y),
          //             0.5 * (imu_last.angular_velocity.z + imu_next.angular_velocity.z);

          // acc_avr   <<0.5 * (imu_last.linear_acceleration.x + imu_next.linear_acceleration.x),
          //             0.5 * (imu_last.linear_acceleration.y + imu_next.linear_acceleration.y),
          //             0.5 * (imu_last.linear_acceleration.z + imu_next.linear_acceleration.z);
          input_in.acc = input_in.acc * G_m_s2 / acc_norm;
          double dt    = rclcpp::Time(imu_last.header.stamp).seconds() - t_last;

          // if(!prop_at_freq_of_imu)
          // {
          double dt_cov = rclcpp::Time(imu_last.header.stamp).seconds() - time_update_last;
          if (dt_cov > 0.0) {
            kf_input.predict(dt_cov, Q_input, input_in, false, true);
            time_update_last = rclcpp::Time(imu_last.header.stamp).seconds();  // time_current;
          }
          kf_input.predict(dt, Q_input, input_in, true, false);
          t_last       = rclcpp::Time(imu_last.header.stamp).seconds();
          imu_prop_cov = true;
          // imu_upda_cov = true;
        }

        double dt           = time_current - t_last;
        t_last              = time_current;
        double propag_start = omp_get_wtime();

        if (!prop_at_freq_of_imu) {
          double dt_cov = time_current - time_update_last;
          if (dt_cov > 0.0) {
            kf_input.predict(dt_cov, Q_input, input_in, false, true);
            time_update_last = time_current;
          }
        }
        kf_input.predict(dt, Q_input, input_in, true, false);

        propag_time += omp_get_wtime() - propag_start;

        // if(k == 0)
        // {
        //     fout_imu_pbp << Measures.lidar_last_time - first_lidar_time << " " << imu_last.angular_velocity.x << " " << imu_last.angular_velocity.y << " "
        //     << imu_last.angular_velocity.z \
                    //             << " " << imu_last.linear_acceleration.x << " " << imu_last.linear_acceleration.y << " " << imu_last.linear_acceleration.z << endl;
        // }

        double t_update_start = omp_get_wtime();

        if (feats_down_size < 1) {
          RCLCPP_WARN(node_->get_logger(), "point, skip this scan!\n");

          idx += time_seq[k];
          continue;
        }
        if (!kf_input.update_iterated_dyn_share_modified()) {
          idx = idx + time_seq[k];
          continue;
        }

        solve_start = omp_get_wtime();

        // if(prop_at_freq_of_imu)
        // {
        //     double dt_cov = time_current - time_update_last;
        //     if ((imu_prop_cov && dt_cov > 0.0) || (dt_cov >= imu_time_inte * 1.2))
        //     {
        //         double propag_cov_start = omp_get_wtime();
        //         kf_input.predict(dt_cov, Q_input, input_in, false, true);
        //         propag_time += omp_get_wtime() - propag_cov_start;
        //         time_update_last = time_current;
        //         imu_prop_cov = false;
        //     }
        // }
        if (publish_odometry_without_downsample) {
          /******* Publish odometry *******/

          publish_odometry();

          if (runtime_pos_log) {
            state_in  = kf_input.x_;
            euler_cur = SO3ToEuler(state_in.rot);
          }
        }

        for (int j = 0; j < time_seq[k]; j++) {
          PointType &point_body_j  = feats_down_body->points[idx + j + 1];
          PointType &point_world_j = feats_down_world->points[idx + j + 1];
          pointBodyToWorld(&point_body_j, &point_world_j);
        }
        solve_time += omp_get_wtime() - solve_start;

        update_time += omp_get_wtime() - t_update_start;
        idx = idx + time_seq[k];
      }
    }

    /******* Publish odometry downsample *******/
    if (!publish_odometry_without_downsample) {
      publish_odometry();
    }

    /*** add the feature points to map kdtree ***/
    t3 = omp_get_wtime();

    if (feats_down_size > 4) {
      map_incremental();
    }

    t5 = omp_get_wtime();

    if (path_en) {
      publish_path();
    }

    if (scan_pub_en) {
      publish_frame_world();
    }

    if (scan_body_pub_en) {
      publish_frame_body();
    }

    /*** Debug variables Logging ***/
    if (runtime_pos_log) {

      frame_num++;
      aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
      { aver_time_icp = aver_time_icp * (frame_num - 1) / frame_num + update_time / frame_num; }
      aver_time_match           = aver_time_match * (frame_num - 1) / frame_num + (match_time) / frame_num;
      aver_time_solve           = aver_time_solve * (frame_num - 1) / frame_num + solve_time / frame_num;
      aver_time_propag          = aver_time_propag * (frame_num - 1) / frame_num + propag_time / frame_num;
      T1[time_log_counter]      = Measures.lidar_beg_time;
      s_plot[time_log_counter]  = t5 - t0;
      s_plot2[time_log_counter] = feats_undistort->points.size();
      s_plot3[time_log_counter] = aver_time_consu;
      time_log_counter++;

      printf(
          "[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: "
          "%0.6f propogate: %0.6f \n",
          t1 - t0, aver_time_match, aver_time_solve, t3 - t1, t5 - t3, aver_time_consu, aver_time_icp, aver_time_propag);

      if (!publish_odometry_without_downsample) {

        if (!use_imu_as_input) {
          state_out = kf_output.x_;
          euler_cur = SO3ToEuler(state_out.rot);
        } else {
          state_in  = kf_input.x_;
          euler_cur = SO3ToEuler(state_in.rot);
        }
      }
    }
  }
}

//}

}  // namespace point_lio

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(point_lio::PointLio)
