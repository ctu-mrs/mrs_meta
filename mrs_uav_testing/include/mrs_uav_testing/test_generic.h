#ifndef TEST_GENERIC_H
#define TEST_GENERIC_H

/* includes //{ */

#include <rclcpp/rclcpp.hpp>

#include <mrs_lib/subscriber_handler.h>
#include <mrs_lib/service_client_handler.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/geometry/cyclic.h>
#include <mrs_lib/param_loader.h>
#include <mrs_lib/publisher_handler.h>

#include <mrs_msgs/msg/control_manager_diagnostics.hpp>
#include <mrs_msgs/msg/uav_manager_diagnostics.hpp>
#include <mrs_msgs/msg/hw_api_status.hpp>
#include <mrs_msgs/msg/estimation_diagnostics.hpp>
#include <mrs_msgs/srv/vec4.hpp>
#include <mrs_msgs/srv/vec1.hpp>
#include <mrs_msgs/msg/gain_manager_diagnostics.hpp>
#include <mrs_msgs/msg/constraint_manager_diagnostics.hpp>
#include <mrs_msgs/msg/safety_area_manager_diagnostics.hpp>
#include <mrs_msgs/srv/string.hpp>
#include <mrs_msgs/msg/uav_state.hpp>
#include <mrs_msgs/srv/path_srv.hpp>
#include <mrs_msgs/msg/float64_stamped.hpp>
#include <mrs_msgs/msg/tracker_command.hpp>
#include <mrs_msgs/msg/dynamics_constraints.hpp>
#include <mrs_msgs/msg/trajectory_reference.hpp>
#include <mrs_msgs/srv/get_path_srv.hpp>
#include <mrs_msgs/msg/velocity_reference_stamped.hpp>
#include <mrs_msgs/srv/velocity_reference_srv.hpp>
#include <mrs_msgs/srv/reference_stamped_srv.hpp>
#include <mrs_msgs/srv/validate_reference.hpp>
#include <mrs_msgs/srv/validate_reference_array.hpp>
#include <mrs_msgs/srv/transform_reference_srv.hpp>
#include <mrs_msgs/srv/transform_vector3_srv.hpp>
#include <mrs_msgs/srv/transform_pose_srv.hpp>
#include <mrs_msgs/srv/constraints_override.hpp>

#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <std_msgs/msg/bool.hpp>

//}

namespace mrs_uav_testing
{

using radians  = mrs_lib::geometry::radians;
using sradians = mrs_lib::geometry::sradians;

using namespace std;

/* class UAVHandler //{ */

class UAVHandler {

public:
  UAVHandler(const rclcpp::Node::SharedPtr node, std::string uav_name, std::shared_ptr<mrs_lib::SubscriberHandlerOptions> shopts,
             std::shared_ptr<mrs_lib::Transformer> transformer, bool use_hw_api = true);

  virtual void initialize(const rclcpp::Node::SharedPtr node, std::string uav_name, std::shared_ptr<mrs_lib::SubscriberHandlerOptions> shopts,
                          std::shared_ptr<mrs_lib::Transformer> transformer, bool use_hw_api = true);

  virtual tuple<bool, string> checkPreconditions(void);

  void sleep(const double &duration);

  tuple<bool, string> takeoff(void);
  tuple<bool, string> arming(const bool input);
  tuple<bool, string> offboard(void);
  tuple<bool, string> land(void);
  tuple<bool, string> eland(void);
  tuple<bool, string> failsafe(void);
  tuple<bool, string> escalatingFailsafe(void);
  tuple<bool, string> landHome(void);
  tuple<bool, string> landThere(const double x, const double y, const double heading);
  tuple<bool, string> activateMidAir(void);

  tuple<bool, string> gotoAbs(const double &x, const double &y, const double &z, const double &hdg);
  tuple<bool, string> gotoRel(const double &x, const double &y, const double &z, const double &hdg);
  tuple<bool, string> gotoFcu(const double &x, const double &y, const double &z, const double &hdg);
  tuple<bool, string> setHeading(const double &setpoint);
  tuple<bool, string> setHeadingRelative(const double &hdg);
  tuple<bool, string> gotoAltitude(const double &z);
  tuple<bool, string> gotoTrajectoryStart();
  tuple<bool, string> startTrajectoryTracking();
  tuple<bool, string> resumeTrajectoryTracking();
  tuple<bool, string> stopTrajectoryTracking();

  tuple<bool, string> gotoRelativeService(const double &x, const double &y, const double &z, const double &hdg);
  tuple<bool, string> gotoService(const double &x, const double &y, const double &z, const double &hdg);
  tuple<bool, string> setReferenceTopic(const double &x, const double &y, const double &z, const double &hdg, const std::string &frame_id);
  tuple<bool, string> setReferenceService(const double &x, const double &y, const double &z, const double &hdg, const std::string &frame_id);
  tuple<bool, string> gotoFcuService(const double &x, const double &y, const double &z, const double &hdg);
  tuple<bool, string> takeoffService(void);

  tuple<bool, string> gotoReference(const double &x, const double &y, const double &z, const double &hdg, const std::string &frame_id);
  tuple<bool, string> gotoReferenceTopic(const double &x, const double &y, const double &z, const double &hdg, const std::string &frame_id);
  tuple<bool, string> referenceService(const double &x, const double &y, const double &z, const double &hdg, const std::string &frame_id);
  tuple<bool, string> referenceTopic(const double &x, const double &y, const double &z, const double &hdg, const std::string &frame_id);

  bool hasGoal(void);
  bool isFlyingNormally(void);
  bool isOutputEnabled(void);
  bool isArmed(void);
  bool isInOffboard(void);
  bool isAtPosition(const double &x, const double &y, const double &z, const double &hdg, const double &pos_tolerance, const std::string frame_id = "");
  bool isAtPosition(const double &x, const double &y, const double &hdg, const double &pos_tolerance, const std::string frame_id = "");
  bool isReferenceAtPosition(const double &x, const double &y, const double &z, const double &hdg, const double &pos_tolerance);

  std::optional<bool> isStationary(void);

  std::optional<double>          getSpeed(void);
  std::optional<double>          getHeading(void);
  std::optional<Eigen::Vector3d> getVelocity(const std::string frame_id);

  std::string                                       getActiveTracker(void);
  std::string                                       getActiveController(void);
  std::string                                       getActiveEstimator(void);
  std::optional<mrs_msgs::msg::TrackerCommand>      getTrackerCmd(void);
  std::optional<double>                             getHeightAgl(void);
  std::optional<mrs_msgs::msg::DynamicsConstraints> getCurrentConstraints(void);

  tuple<bool, string> setPathSrv(const mrs_msgs::msg::Path &path_in);
  tuple<bool, string> setPathTopic(const mrs_msgs::msg::Path &path_in);
  tuple<bool, string> switchEstimator(const std::string &estimator);
  tuple<bool, string> switchController(const std::string &controller);
  tuple<bool, string> switchTracker(const std::string &tracker);
  tuple<bool, string> setGains(const std::string &gains);
  tuple<bool, string> setConstraints(const std::string &constraints);
  tuple<bool, string> overrideConstraints(const double hor_a, const double ver_a);
  tuple<bool, string> hover();

  tuple<std::optional<mrs_msgs::msg::TrajectoryReference>, std::optional<Eigen::VectorXd>, string> getPathSrv(const mrs_msgs::msg::Path &path_in);

  bool mrsSystemReady(void);

  tuple<bool, string> validateReference(const mrs_msgs::msg::ReferenceStamped &msg);

  tuple<bool, std::optional<mrs_msgs::srv::ValidateReferenceArray::Response>>
  validateReferenceArray(const mrs_msgs::srv::ValidateReferenceArray::Request &request);

  std::tuple<bool, std::optional<std::string>, std::optional<geometry_msgs::msg::PoseStamped>> transformPose(const geometry_msgs::msg::PoseStamped &msg,
                                                                                                             std::string target_frame);
  tuple<bool, std::optional<std::string>, std::optional<mrs_msgs::msg::ReferenceStamped>>      transformReference(const mrs_msgs::msg::ReferenceStamped &msg,
                                                                                                                  std::string target_frame);
  tuple<bool, std::optional<std::string>, std::optional<geometry_msgs::msg::Vector3Stamped>>   transformVector3(const geometry_msgs::msg::Vector3Stamped &msg,
                                                                                                                std::string target_frame);

  mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlManagerDiagnostics>    sh_control_manager_diag_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::DynamicsConstraints>          sh_current_constraints_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::UavManagerDiagnostics>        sh_uav_manager_diag_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::EstimationDiagnostics>        sh_estim_manager_diag_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::GainManagerDiagnostics>       sh_gain_manager_diag_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::ConstraintManagerDiagnostics> sh_constraint_manager_diag_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::SafetyAreaManagerDiagnostics> sh_safety_area_manager_diag_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::UavState>                     sh_uav_state_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::TrackerCommand>               sh_tracker_cmd_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::Float64Stamped>               sh_height_agl_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::Float64Stamped>               sh_max_height_;
  mrs_lib::SubscriberHandler<mrs_msgs::msg::Float64Stamped>               sh_speed_;

  mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>             sch_arming_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>             sch_offboard_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>             sch_midair_activation_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>             sch_land_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>             sch_eland_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>             sch_failsafe_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>             sch_escalating_failsafe_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>             sch_land_home_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::ReferenceStampedSrv> sch_land_there_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>              sch_switch_estimator_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>              sch_switch_controller_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>              sch_switch_tracker_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>              sch_set_gains_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>              sch_set_constraints_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>             sch_takeoff_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::ConstraintsOverride> sch_override_constraints_;

  mrs_lib::ServiceClientHandler<mrs_msgs::srv::Vec4>                sch_goto_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::Vec4>                sch_goto_fcu_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::PathSrv>             sch_path_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::Vec4>                sch_goto_relative_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::Vec1>                sch_set_heading_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::Vec1>                sch_set_heading_relative_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::Vec1>                sch_goto_altitude_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::ReferenceStampedSrv> sch_reference_;

  mrs_lib::ServiceClientHandler<mrs_msgs::srv::GetPathSrv> sch_get_path_;

  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> sch_start_trajectory_tracking_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> sch_stop_trajectory_tracking_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> sch_resume_trajectory_tracking_;
  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> sch_goto_trajectory_start_;

  mrs_lib::ServiceClientHandler<mrs_msgs::srv::ValidateReference>      sch_validate_reference_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::ValidateReferenceArray> sch_validate_reference_array_;

  mrs_lib::ServiceClientHandler<mrs_msgs::srv::TransformReferenceSrv> sch_tranform_reference_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::TransformVector3Srv>   sch_tranform_vector3_;
  mrs_lib::ServiceClientHandler<mrs_msgs::srv::TransformPoseSrv>      sch_tranform_pose_;

  mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger> sch_hover_;

  mrs_lib::PublisherHandler<mrs_msgs::msg::Path>                     ph_path_;
  mrs_lib::PublisherHandler<mrs_msgs::msg::TrajectoryReference>      ph_trajectory_;
  mrs_lib::PublisherHandler<mrs_msgs::msg::VelocityReferenceStamped> ph_velocity_reference_;
  mrs_lib::PublisherHandler<mrs_msgs::msg::ReferenceStamped>         ph_reference_;

  mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiStatus> sh_hw_api_status_;

  std::shared_ptr<mrs_lib::Transformer> transformer_;

protected:
  bool initialized_ = false;

  string _uav_name_;

  std::shared_ptr<mrs_lib::SubscriberHandlerOptions> shopts_;

  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_sc_;

  string name_;
  bool   use_hw_api_ = true;
};

//}

/* class TestGeneric //{ */

class TestGeneric {

public:
  TestGeneric();

  void initialize(void);

  virtual bool test() = 0;

  std::shared_ptr<mrs_lib::ParamLoader> pl_;

  std::tuple<std::optional<std::shared_ptr<UAVHandler>>, string> getUAVHandler(const string &uav_name, const bool use_hw_api = true);

  void sleep(const double &duration);

  void join();

  void stop();

  void reportTestResult(const bool result);

protected:
  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  std::thread main_thread_;

  std::shared_ptr<mrs_lib::Transformer> transformer_;

  std::shared_ptr<mrs_lib::SubscriberHandlerOptions> shopts_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_result_;

  string _test_name_;
  string name_;

  bool initialized_ = false;

  bool mrsSystemReady(void);

private:
  void spin();

  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
};

//}

} // namespace mrs_uav_testing

#endif // TEST_GENERIC_H
