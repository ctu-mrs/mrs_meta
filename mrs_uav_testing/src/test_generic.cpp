#include <mrs_uav_testing/test_generic.h>

namespace mrs_uav_testing
{

/* UAVHandler::UAVHandler //{ */

UAVHandler::UAVHandler(const rclcpp::Node::SharedPtr node, std::string uav_name, std::shared_ptr<mrs_lib::SubscriberHandlerOptions> shopts,
                       std::shared_ptr<mrs_lib::Transformer> transformer, bool use_hw_api) {

  initialize(node, uav_name, shopts, transformer, use_hw_api);
}

//}

/* UAVHandler::initialize() //{ */

void UAVHandler::initialize(const rclcpp::Node::SharedPtr node, std::string uav_name, std::shared_ptr<mrs_lib::SubscriberHandlerOptions> shopts,
                            std::shared_ptr<mrs_lib::Transformer> transformer, bool use_hw_api) {

  cbkgrp_subs_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_sc_   = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  _uav_name_ = uav_name;
  shopts_    = shopts;
  node_      = node;
  clock_     = node->get_clock();
  name_      = shopts->node_name;

  shopts_->subscription_options.callback_group = cbkgrp_subs_;

  transformer_ = transformer;

  use_hw_api_ = use_hw_api;

  sh_control_manager_diag_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::ControlManagerDiagnostics>(*shopts_, "/" + _uav_name_ + "/control_manager/diagnostics");
  sh_current_constraints_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::DynamicsConstraints>(*shopts_, "/" + _uav_name_ + "/control_manager/current_constraints");
  sh_uav_manager_diag_    = mrs_lib::SubscriberHandler<mrs_msgs::msg::UavManagerDiagnostics>(*shopts_, "/" + _uav_name_ + "/uav_manager/diagnostics");
  sh_tracker_cmd_         = mrs_lib::SubscriberHandler<mrs_msgs::msg::TrackerCommand>(*shopts_, "/" + _uav_name_ + "/control_manager/tracker_cmd");
  sh_estim_manager_diag_  = mrs_lib::SubscriberHandler<mrs_msgs::msg::EstimationDiagnostics>(*shopts_, "/" + _uav_name_ + "/estimation_manager/diagnostics");
  sh_constraint_manager_diag_ =
      mrs_lib::SubscriberHandler<mrs_msgs::msg::ConstraintManagerDiagnostics>(*shopts_, "/" + _uav_name_ + "/constraint_manager/diagnostics");
  sh_gain_manager_diag_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::GainManagerDiagnostics>(*shopts_, "/" + _uav_name_ + "/gain_manager/diagnostics");
  sh_safety_area_manager_diag_ =
      mrs_lib::SubscriberHandler<mrs_msgs::msg::SafetyAreaManagerDiagnostics>(*shopts_, "/" + _uav_name_ + "/safety_area_manager/diagnostics");
  sh_uav_state_     = mrs_lib::SubscriberHandler<mrs_msgs::msg::UavState>(*shopts_, "/" + _uav_name_ + "/estimation_manager/uav_state");
  sh_height_agl_    = mrs_lib::SubscriberHandler<mrs_msgs::msg::Float64Stamped>(*shopts_, "/" + _uav_name_ + "/estimation_manager/height_agl");
  sh_max_height_    = mrs_lib::SubscriberHandler<mrs_msgs::msg::Float64Stamped>(*shopts_, "/" + _uav_name_ + "/estimation_manager/max_flight_z_agl");
  sh_speed_         = mrs_lib::SubscriberHandler<mrs_msgs::msg::Float64Stamped>(*shopts_, "/" + _uav_name_ + "/control_manager/speed");
  sh_hw_api_status_ = mrs_lib::SubscriberHandler<mrs_msgs::msg::HwApiStatus>(*shopts_, "/" + _uav_name_ + "/hw_api/status");

  // | --------------------- service clients -------------------- |

  sch_arming_            = mrs_lib::ServiceClientHandler<std_srvs::srv::SetBool>(node_, "/" + _uav_name_ + "/hw_api/arming", cbkgrp_sc_);
  sch_offboard_          = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "/" + _uav_name_ + "/hw_api/offboard", cbkgrp_sc_);
  sch_midair_activation_ = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "/" + _uav_name_ + "/uav_manager/midair_activation", cbkgrp_sc_);
  sch_land_              = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "/" + _uav_name_ + "/uav_manager/land", cbkgrp_sc_);
  sch_eland_             = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "/" + _uav_name_ + "/control_manager/eland", cbkgrp_sc_);
  sch_failsafe_          = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "/" + _uav_name_ + "/control_manager/failsafe", cbkgrp_sc_);
  sch_escalating_failsafe_ =
      mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "/" + _uav_name_ + "/control_manager/failsafe_escalating", cbkgrp_sc_);
  sch_land_home_         = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "/" + _uav_name_ + "/uav_manager/land_home", cbkgrp_sc_);
  sch_land_there_        = mrs_lib::ServiceClientHandler<mrs_msgs::srv::ReferenceStampedSrv>(node_, "/" + _uav_name_ + "/uav_manager/land_there", cbkgrp_sc_);
  sch_switch_estimator_  = mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>(node_, "/" + _uav_name_ + "/estimation_manager/change_estimator", cbkgrp_sc_);
  sch_switch_controller_ = mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>(node_, "/" + _uav_name_ + "/control_manager/switch_controller", cbkgrp_sc_);
  sch_switch_tracker_    = mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>(node_, "/" + _uav_name_ + "/control_manager/switch_tracker", cbkgrp_sc_);
  sch_set_gains_         = mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>(node_, "/" + _uav_name_ + "/gain_manager/set_gains", cbkgrp_sc_);
  sch_set_constraints_   = mrs_lib::ServiceClientHandler<mrs_msgs::srv::String>(node_, "/" + _uav_name_ + "/constraint_manager/set_constraints", cbkgrp_sc_);
  sch_takeoff_           = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "/" + _uav_name_ + "/uav_manager/takeoff", cbkgrp_sc_);
  sch_override_constraints_ =
      mrs_lib::ServiceClientHandler<mrs_msgs::srv::ConstraintsOverride>(node_, "/" + _uav_name_ + "/constraint_manager/constraints_override", cbkgrp_sc_);

  sch_goto_                 = mrs_lib::ServiceClientHandler<mrs_msgs::srv::Vec4>(node_, "/" + _uav_name_ + "/control_manager/goto", cbkgrp_sc_);
  sch_goto_fcu_             = mrs_lib::ServiceClientHandler<mrs_msgs::srv::Vec4>(node_, "/" + _uav_name_ + "/control_manager/goto_fcu", cbkgrp_sc_);
  sch_goto_relative_        = mrs_lib::ServiceClientHandler<mrs_msgs::srv::Vec4>(node_, "/" + _uav_name_ + "/control_manager/goto_relative", cbkgrp_sc_);
  sch_set_heading_          = mrs_lib::ServiceClientHandler<mrs_msgs::srv::Vec1>(node_, "/" + _uav_name_ + "/control_manager/set_heading", cbkgrp_sc_);
  sch_set_heading_relative_ = mrs_lib::ServiceClientHandler<mrs_msgs::srv::Vec1>(node_, "/" + _uav_name_ + "/control_manager/set_heading_relative", cbkgrp_sc_);
  sch_goto_altitude_        = mrs_lib::ServiceClientHandler<mrs_msgs::srv::Vec1>(node_, "/" + _uav_name_ + "/control_manager/goto_altitude", cbkgrp_sc_);

  sch_reference_ = mrs_lib::ServiceClientHandler<mrs_msgs::srv::ReferenceStampedSrv>(node_, "/" + _uav_name_ + "/control_manager/reference", cbkgrp_sc_);

  sch_goto_trajectory_start_ =
      mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "/" + _uav_name_ + "/control_manager/goto_trajectory_start", cbkgrp_sc_);
  sch_start_trajectory_tracking_ =
      mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "/" + _uav_name_ + "/control_manager/start_trajectory_tracking", cbkgrp_sc_);
  sch_stop_trajectory_tracking_ =
      mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "/" + _uav_name_ + "/control_manager/stop_trajectory_tracking", cbkgrp_sc_);
  sch_resume_trajectory_tracking_ =
      mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "/" + _uav_name_ + "/control_manager/resume_trajectory_tracking", cbkgrp_sc_);

  sch_hover_ = mrs_lib::ServiceClientHandler<std_srvs::srv::Trigger>(node_, "/" + _uav_name_ + "/control_manager/hover", cbkgrp_sc_);

  sch_path_     = mrs_lib::ServiceClientHandler<mrs_msgs::srv::PathSrv>(node_, "/" + _uav_name_ + "/trajectory_generation/path", cbkgrp_sc_);
  sch_get_path_ = mrs_lib::ServiceClientHandler<mrs_msgs::srv::GetPathSrv>(node_, "/" + _uav_name_ + "/trajectory_generation/get_path", cbkgrp_sc_);

  sch_validate_reference_ =
      mrs_lib::ServiceClientHandler<mrs_msgs::srv::ValidateReference>(node_, "/" + _uav_name_ + "/control_manager/validate_reference", cbkgrp_sc_);
  sch_validate_reference_array_ =
      mrs_lib::ServiceClientHandler<mrs_msgs::srv::ValidateReferenceArray>(node_, "/" + _uav_name_ + "/control_manager/validate_reference_array", cbkgrp_sc_);

  sch_tranform_reference_ =
      mrs_lib::ServiceClientHandler<mrs_msgs::srv::TransformReferenceSrv>(node_, "/" + _uav_name_ + "/control_manager/transform_reference", cbkgrp_sc_);
  sch_tranform_vector3_ =
      mrs_lib::ServiceClientHandler<mrs_msgs::srv::TransformVector3Srv>(node_, "/" + _uav_name_ + "/control_manager/transform_vector3", cbkgrp_sc_);
  sch_tranform_pose_ = mrs_lib::ServiceClientHandler<mrs_msgs::srv::TransformPoseSrv>(node_, "/" + _uav_name_ + "/control_manager/transform_pose", cbkgrp_sc_);

  // | ----------------------- publishers ----------------------- |

  ph_path_               = mrs_lib::PublisherHandler<mrs_msgs::msg::Path>(node_, "/" + _uav_name_ + "/trajectory_generation/path");
  ph_trajectory_         = mrs_lib::PublisherHandler<mrs_msgs::msg::TrajectoryReference>(node_, "/" + _uav_name_ + "/control_manager/trajectory_reference");
  ph_velocity_reference_ = mrs_lib::PublisherHandler<mrs_msgs::msg::VelocityReferenceStamped>(node_, "/" + _uav_name_ + "/control_manager/velocity_reference");
  ph_reference_          = mrs_lib::PublisherHandler<mrs_msgs::msg::ReferenceStamped>(node_, "/" + _uav_name_ + "/control_manager/reference");

  initialized_ = true;
}

//}

//}

/* TestGeneric::TestGeneric() //{ */

TestGeneric::TestGeneric() {

  initialize();
}

//}

/* initialize() //{ */

void TestGeneric::initialize(void) {

  node_     = rclcpp::Node::make_shared("test");
  clock_    = node_->get_clock();
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  executor_->add_node(node_);

  RCLCPP_INFO(node_->get_logger(), "ROS node initialized");

  // | ----------------------- load params ---------------------- |

  pl_ = std::make_shared<mrs_lib::ParamLoader>(node_, "Test");

  pl_->loadParam("test_name", _test_name_, std::string());

  if (!pl_->loadedSuccessfully()) {
    RCLCPP_INFO(node_->get_logger(), "failed to load mandatory parameters");
    rclcpp::shutdown();
    exit(1);
  }

  std::vector<std::string> config_files;

  pl_->loadParam("config_files", config_files, std::vector<std::string>());

  if (config_files.size() > 0) {

    for (size_t i = 0; i < config_files.size(); i++) {
      if (!pl_->addYamlFile(config_files[i])) {
        RCLCPP_ERROR(node_->get_logger(), "failed to load config file '%s'", config_files[i].c_str());
        rclcpp::shutdown();
        exit(1);
      }
    }
  }

  name_ = "test/" + _test_name_;

  // | ----------------------- transformer ---------------------- |

  transformer_ = std::make_shared<mrs_lib::Transformer>(node_);
  transformer_->retryLookupNewest(true);

  // | ----------------------- subscribers ---------------------- |

  shopts_ = std::make_shared<mrs_lib::SubscriberHandlerOptions>();

  shopts_->node               = node_;
  shopts_->node_name          = name_;
  shopts_->no_message_timeout = mrs_lib::no_timeout;
  shopts_->threadsafe         = true;
  shopts_->autostart          = true;

  // | ----------------------- publishers ----------------------- |

  publisher_result_ = node_->create_publisher<std_msgs::msg::Bool>("/test_result", rclcpp::SystemDefaultsQoS());

  // | ------------------ start the main thread ----------------- |

  main_thread_ = std::thread(&TestGeneric::spin, this);

  // | --------------------- finish the init -------------------- |

  initialized_ = true;

  RCLCPP_INFO(node_->get_logger(), "[%s]: initialized", name_.c_str());
}

//}

/* spin() //{ */

void TestGeneric::spin() {

  printf("[TestGeneric]: spinning");

  executor_->spin();

  printf("[TestGeneric]: executor spin() has died, shit");
}

//}

/* join() //{ */

void TestGeneric::join() {

  printf("[TestGeneric]: joined");

  main_thread_.join();
}

//}

/* stop() //{ */

void TestGeneric::stop() {

  printf("[TestGeneric]: stopping");

  executor_->cancel();
}

//}

/* reportTestResult() //{ */

void TestGeneric::reportTestResult(const bool result) {

  printf("[%s]: publishing result %s", name_.c_str(), result ? "SUCCESS" : "FAILED");

  std_msgs::msg::Bool result_msg;
  result_msg.data = result;

  publisher_result_->publish(result_msg);
}

//}

// | --------------------- action methods --------------------- |

/* checkPreconditions() //{ */

tuple<bool, string> UAVHandler::checkPreconditions(void) {

  if (!initialized_) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "[" << node_->get_name() << "]: UAV handler for " << _uav_name_ << " is not initialized!");
    return {false, "UAV handler for " + _uav_name_ + " is not initialized!"};
  }

  return {true, "All clear."};
}

//}

/* getUAVHandler() //{ */

std::tuple<std::optional<std::shared_ptr<UAVHandler>>, string> TestGeneric::getUAVHandler(const string &uav_name, const bool use_hw_api) {

  if (!initialized_) {
    return {std::nullopt, string("Can not obtain UAV handler for  " + uav_name + " - testing is not initialized yet!")};
  } else {
    return {std::make_shared<UAVHandler>(node_, uav_name, shopts_, transformer_, use_hw_api), "Success!"};
  }
}

//}

/* arming() //{ */

tuple<bool, string> UAVHandler::arming(const bool input) {

  std::shared_ptr<std_srvs::srv::SetBool::Request> request = std::make_shared<std_srvs::srv::SetBool::Request>();

  request->data = input;

  {
    auto response = sch_arming_.callSync(request);

    if (!response) {
      return {false, "arming service call failed"};
    } else if (!response.value()->success) {
      return {false, "arming service call failed: " + response.value()->message};
    }
  }

  return {true, "armed"};
}

//}

/* offboard() //{ */

tuple<bool, string> UAVHandler::offboard() {

  std::shared_ptr<std_srvs::srv::Trigger::Request> request = std::make_shared<std_srvs::srv::Trigger::Request>();

  {
    auto response = sch_offboard_.callSync(request);

    if (!response) {
      return {false, "offboard service call failed"};
    } else if (!response.value()->success) {
      return {false, "offboard service call failed: " + response.value()->message};
    }
  }

  return {true, "offboard trigerred"};
}

//}

/* takeoff() //{ */

tuple<bool, string> UAVHandler::takeoff(void) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  // | ---------------- wait for ready to takeoff --------------- |

  while (true) {

    if (!rclcpp::ok()) {
      return {false, "shut down from outside"};
    }

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: waiting for the MRS UAV System", name_.c_str());

    if (mrsSystemReady()) {
      RCLCPP_INFO(node_->get_logger(), "[%s]: MRS UAV System is ready", name_.c_str());
      break;
    }

    sleep(0.01);
  }

  // | ---------------------- arm the drone --------------------- |

  RCLCPP_INFO(node_->get_logger(), "[%s]: arming the drone", name_.c_str());

  {
    auto [success, message] = arming(true);

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "arming failed with message: '%s'", message.c_str());
      return {false, message};
    }
  }

  // | ---------------------- wait a second --------------------- |

  sleep(2.0);

  // | --------------------- check if armed --------------------- |

  if (!isArmed()) {
    return {false, "not armed"};
  }

  sleep(0.2);

  // | --------------- check if output is enabled --------------- |

  if (!isOutputEnabled()) {
    return {false, "output not enabled by automatic start"};
  }

  // | ------------------- switch to offboard ------------------- |

  {
    auto [success, message] = offboard();

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "offboard trigger failed with message: '%s'", message.c_str());
      return {false, message};
    }
  }

  // | -------------------------- wait -------------------------- |

  sleep(0.1);

  // | ------------------ check if in offboard ------------------ |

  if (!isInOffboard()) {
    return {false, "not in offboard"};
  }

  // | --------------- wait for takeoff to finish --------------- |

  while (true) {

    if (!rclcpp::ok()) {
      return {false, "shut down from outside"};
    }

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: waiting for the takeoff to finish", name_.c_str());

    if (sh_control_manager_diag_.getMsg()->flying_normally) {

      return {true, "takeoff finished"};
    }

    sleep(0.01);
  }

  return {false, "reached end of the method without assertion"};
}

//}

/* takeoffService() //{ */

tuple<bool, string> UAVHandler::takeoffService() {

  std::shared_ptr<std_srvs::srv::Trigger::Request> request = std::make_shared<std_srvs::srv::Trigger::Request>();

  {
    auto response = sch_takeoff_.callSync(request);

    if (!response) {
      return {false, "takeoff service call failed"};
    } else if (!response.value()->success) {
      return {false, "takeoff service call failed: " + response.value()->message};
    }
  }

  return {true, "takeoff trigerred"};
}

//}

/* land() //{ */

tuple<bool, string> UAVHandler::land(void) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  if (!isFlyingNormally()) {
    return {false, "not flying normally in the beginning"};
  }

  // | -------------------- call land service ------------------- |

  RCLCPP_INFO(node_->get_logger(), "[%s]: calling for landing", name_.c_str());

  {
    std::shared_ptr<std_srvs::srv::Trigger::Request> request = std::make_shared<std_srvs::srv::Trigger::Request>();

    {
      auto response = sch_land_.callSync(request);

      if (!response || !response.value()->success) {
        return {false, "land service call failed"};
      }
    }
  }

  // | ---------------------- wait a second --------------------- |

  sleep(1.0);

  // | -------- wait till the right controller is active -------- |

  while (true) {

    if (!rclcpp::ok()) {
      return {false, "shut down from outside"};
    }

    if (sh_control_manager_diag_.getMsg()->active_tracker == "LandoffTracker" && sh_control_manager_diag_.getMsg()->active_controller == "MpcController") {
      break;
    }

    sleep(0.01);
  }

  // | ------------- wait for the landing to finish ------------- |

  while (true) {

    if (!rclcpp::ok()) {
      return {false, "shut down from outside"};
    }

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: waiting for the landing to finish", name_.c_str());

    if (!isArmed()) {
      return {true, "landing finished"};
    }

    sleep(0.01);
  }

  return {false, "reached end of the method without assertion"};
}

//}

/* eland() //{ */

tuple<bool, string> UAVHandler::eland(void) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  if (!isFlyingNormally()) {
    return {false, "not flying normally in the beginning"};
  }

  // | -------------------- call eland service ------------------- |

  RCLCPP_INFO(node_->get_logger(), "[%s]: calling for emergency landing", name_.c_str());

  {
    std::shared_ptr<std_srvs::srv::Trigger::Request> request = std::make_shared<std_srvs::srv::Trigger::Request>();

    {
      auto response = sch_eland_.callSync(request);

      if (!response || !response.value()->success) {
        return {false, "eland service call failed"};
      }
    }
  }

  // | ---------------------- wait a second --------------------- |

  sleep(1.0);

  // | -------- wait till the right controller is active -------- |

  while (true) {

    if (!rclcpp::ok()) {
      return {false, "shut down from outside"};
    }

    if (sh_control_manager_diag_.getMsg()->active_tracker == "LandoffTracker" &&
        sh_control_manager_diag_.getMsg()->active_controller == "EmergencyController") {
      break;
    }

    sleep(0.01);
  }

  // | ------------- wait for the elanding to finish ------------- |

  while (true) {

    if (!rclcpp::ok()) {
      return {false, "shut down from outside"};
    }

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: waiting for the elanding to finish", name_.c_str());

    if (!isOutputEnabled()) {

      return {true, "elanding finished"};
    }

    sleep(0.01);
  }

  return {false, "reached end of the method without assertion"};
}

//}

/* failsafe() //{ */

tuple<bool, string> UAVHandler::failsafe(void) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  if (!isFlyingNormally()) {
    return {false, "not flying normally in the beginning"};
  }

  // | ---------------- call failsafe service ------------------- |

  RCLCPP_INFO(node_->get_logger(), "[%s]: calling for failsafe", name_.c_str());

  {
    std::shared_ptr<std_srvs::srv::Trigger::Request> request = std::make_shared<std_srvs::srv::Trigger::Request>();

    {
      auto response = sch_failsafe_.callSync(request);

      if (!response || !response.value()->success) {
        return {false, "failsafe service call failed"};
      }
    }
  }

  return {true, "failsafe called"};
}

//}

/* escalatingFailsafe() //{ */

tuple<bool, string> UAVHandler::escalatingFailsafe(void) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  // | ---------------- call failsafe service ------------------- |

  RCLCPP_INFO(node_->get_logger(), "[%s]: calling for escalating failsafe", name_.c_str());

  {
    std::shared_ptr<std_srvs::srv::Trigger::Request> request = std::make_shared<std_srvs::srv::Trigger::Request>();

    {
      auto response = sch_escalating_failsafe_.callSync(request);

      if (!response || !response.value()->success) {
        return {false, "escalating failsafe service call failed"};
      }
    }
  }

  return {true, "escalating failsafe called"};
}

//}

/* landHome() //{ */

tuple<bool, string> UAVHandler::landHome(void) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  if (!isFlyingNormally()) {
    return {false, "not flying normally in the beginning"};
  }

  // | ----------------- call land home service ----------------- |

  RCLCPP_INFO(node_->get_logger(), "[%s]: calling for landing home", name_.c_str());

  {
    std::shared_ptr<std_srvs::srv::Trigger::Request> request = std::make_shared<std_srvs::srv::Trigger::Request>();

    {
      auto response = sch_land_home_.callSync(request);

      if (!response || !response.value()->success) {
        return {false, "land home service call failed"};
      }
    }
  }

  // | ---------------------- wait a second --------------------- |

  sleep(1.0);

  // | -------- wait till the right controller is active -------- |

  while (true) {

    if (!rclcpp::ok()) {
      return {false, "shut down from outside"};
    }

    if (sh_control_manager_diag_.getMsg()->active_tracker == "LandoffTracker" && sh_control_manager_diag_.getMsg()->active_controller == "MpcController") {
      break;
    }
  }

  // | ------------- wait for the landing to finish ------------- |

  while (true) {

    if (!rclcpp::ok()) {
      return {false, "shut down from outside"};
    }

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: waiting for the landing to finish", name_.c_str());

    if (!isOutputEnabled()) {

      return {true, "landing finished"};
    }

    sleep(0.01);
  }

  return {false, "reached end of the method without assertion"};
}

//}

/* landThere() //{ */

tuple<bool, string> UAVHandler::landThere(const double x, const double y, const double heading) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  if (!isFlyingNormally()) {
    return {false, "not flying normally in the beginning"};
  }

  // | ----------------- call land home service ----------------- |

  RCLCPP_INFO(node_->get_logger(), "[%s]: calling for landing there", name_.c_str());

  {
    std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Request> request = std::make_shared<mrs_msgs::srv::ReferenceStampedSrv::Request>();

    request->reference.position.x = x;
    request->reference.position.y = y;
    request->reference.heading    = heading;

    {
      auto response = sch_land_there_.callSync(request);

      if (!response || !response.value()->success) {
        return {false, "land there service call failed"};
      }
    }
  }

  // | ---------------------- wait a second --------------------- |

  sleep(1.0);

  // | -------- wait till the right controller is active -------- |

  while (true) {

    if (!rclcpp::ok()) {
      return {false, "shut down from outside"};
    }

    if (sh_control_manager_diag_.getMsg()->active_tracker == "LandoffTracker" && sh_control_manager_diag_.getMsg()->active_controller == "MpcController") {
      break;
    }
  }

  // | ------------- wait for the landing to finish ------------- |

  while (true) {

    if (!rclcpp::ok()) {
      return {false, "shut down from outside"};
    }

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: waiting for the landing to finish", name_.c_str());

    if (!isOutputEnabled()) {

      return {true, "landing finished"};
    }

    sleep(0.01);
  }

  return {false, "reached end of the method without assertion"};
}

//}

/* activateMidAir() //{ */

tuple<bool, string> UAVHandler::activateMidAir(void) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  // | ---------------- wait for ready to takeoff --------------- |

  while (true) {

    if (!rclcpp::ok()) {
      return {false, "shut down from outside"};
    }

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: waiting for the MRS UAV System", name_.c_str());

    if (mrsSystemReady()) {
      RCLCPP_INFO(node_->get_logger(), "[%s]: MRS UAV System is ready", name_.c_str());
      break;
    }

    sleep(0.01);
  }

  // | ---------------------- arm the drone --------------------- |

  RCLCPP_INFO(node_->get_logger(), "[%s]: arming the drone", name_.c_str());

  {
    std::shared_ptr<std_srvs::srv::SetBool::Request> request = std::make_shared<std_srvs::srv::SetBool::Request>();

    request->data = true;

    {
      auto response = sch_arming_.callSync(request);

      if (!response || !response.value()->success) {
        return {false, "arming service call failed"};
      }
    }
  }

  // | -------------------------- wait -------------------------- |

  sleep(0.1);

  // | --------------------- check if armed --------------------- |

  if (!isArmed()) {
    return {false, "not armed"};
  }

  // | ----------------- call midair activation ----------------- |

  {
    std::shared_ptr<std_srvs::srv::Trigger::Request> request = std::make_shared<std_srvs::srv::Trigger::Request>();

    {
      std::cout << "Test: calling midair activation" << std::endl;

      auto response = sch_midair_activation_.callSync(request);

      std::cout << "Test: midair activation result returned" << std::endl;

      if (!response) {

        std::cout << "Test: midair call activation failed" << std::endl;

        return {false, "midair activation service call failed"};

      } else {

        if (!response.value()->success) {

          std::cout << "Test: midair call activation failed: " << response.value()->message << std::endl;

          return {false, "midair activation service call failed"};
        }
      }
    }
  }

  // | --------------- waiting for flying normally -------------- |

  std::cout << "Test: waiting for flying normally" << std::endl;

  while (true) {

    if (!rclcpp::ok()) {
      return {false, "shut down from outside"};
    }

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "[%s]: waiting for the midair activation to finish", name_.c_str());

    auto control_diag = sh_control_manager_diag_.getMsg();

    if (control_diag->flying_normally && control_diag->active_controller != "MidairActivationController" &&
        control_diag->active_tracker != "MidairActivationTracker") {
      return {true, "midair activation finished"};
    }

    if (!control_diag->flying_normally && control_diag->active_controller == "EmergencyController" && control_diag->active_tracker == "LandoffTracker") {
      return {false, "midair activation failed to finish"};
    }

    sleep(0.01);
  }

  return {false, "reached end of the method without assertion"};
}

//}

/* gotoAbs() //{ */

tuple<bool, string> UAVHandler::gotoAbs(const double &x, const double &y, const double &z, const double &hdg) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  {
    auto [success, message] = gotoService(x, y, z, hdg);

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "[%s]: goto service failed with message: '%s'", node_->get_name(), message.c_str());
      return {success, message};
    }
  }

  // | -------------------- check for result -------------------- |

  while (true) {

    if (!rclcpp::ok()) {
      return {false, "shut down from outside"};
    }

    if (!isFlyingNormally()) {
      return {false, "not flying normally"};
    }

    if (isAtPosition(x, y, z, hdg, 0.1)) {
      return {true, "goal reached"};
    }

    sleep(0.01);
  }

  return {false, "reached end of the method without assertion"};
}

//}

/* gotoService() //{ */

tuple<bool, string> UAVHandler::gotoService(const double &x, const double &y, const double &z, const double &hdg) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  std::shared_ptr<mrs_msgs::srv::Vec4::Request> request = std::make_shared<mrs_msgs::srv::Vec4::Request>();

  request->goal[0] = x;
  request->goal[1] = y;
  request->goal[2] = z;
  request->goal[3] = hdg;

  {
    auto response = sch_goto_.callSync(request);

    if (!response || !response.value()->success) {
      return {false, "goto service call failed"};
    }
  }

  return {true, "goto service triggered"};
}

//}

/* gotoRel() //{ */

tuple<bool, string> UAVHandler::gotoRel(const double &x, const double &y, const double &z, const double &hdg) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  auto start_pose = sh_tracker_cmd_.getMsg()->position;
  auto start_hdg  = sh_tracker_cmd_.getMsg()->heading;

  {
    auto [success, message] = gotoRelativeService(x, y, z, hdg);

    if (!success) {
      return {false, message};
    }
  }

  // | -------------------- check for result -------------------- |

  while (true) {

    if (!rclcpp::ok()) {
      return {false, "shut down from outside"};
    }

    if (!isFlyingNormally()) {
      return {false, "not flying normally"};
    }

    if (isAtPosition(start_pose.x + x, start_pose.y + y, start_pose.z + z, start_hdg + hdg, 0.1)) {
      return {true, "goal reached"};
    }

    sleep(0.01);
  }

  return {false, "reached end of the method without assertion"};
}

//}

/* gotoFcu() //{ */

tuple<bool, string> UAVHandler::gotoFcu(const double &x, const double &y, const double &z, const double &hdg) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  this->sleep(2.0);

  auto tracker_cmd = getTrackerCmd();

  if (!tracker_cmd) {
    return {false, "missing tracker_cmd"};
  }

  // | ------- get the desired position in the world frame ------ |

  mrs_msgs::msg::ReferenceStamped ref_in;

  ref_in.header.frame_id      = _uav_name_ + "/fcu_untilted";
  ref_in.reference.position.x = x;
  ref_in.reference.position.y = y;
  ref_in.reference.position.z = z;
  ref_in.reference.heading    = hdg;

  auto ref_transformed = transformer_->transformSingle(ref_in, tracker_cmd->header.frame_id);

  if (!ref_transformed) {
    return {false, "could not transform the reference"};
  }

  // | -------------------- call the service -------------------- |

  {
    auto [success, message] = gotoFcuService(x, y, z, hdg);

    if (!success) {
      return {false, message};
    }
  }

  // | -------------------- check for result -------------------- |

  while (true) {

    if (!rclcpp::ok()) {
      return {false, "shut down from outside"};
    }

    if (!isFlyingNormally()) {
      return {false, "not flying normally"};
    }

    if (isAtPosition(ref_transformed->reference.position.x, ref_transformed->reference.position.y, ref_transformed->reference.position.z,
                     ref_transformed->reference.heading, 0.3, ref_transformed->header.frame_id)) {
      return {true, "goal reached"};
    }

    sleep(0.01);
  }

  return {false, "reached end of the method without assertion"};
}

//}

/* gotoReference() //{ */

tuple<bool, string> UAVHandler::gotoReference(const double &x, const double &y, const double &z, const double &hdg, const std::string &frame_id) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  // | ------------------ create the reference ------------------ |

  mrs_msgs::msg::ReferenceStamped ref_in;

  ref_in.header.frame_id      = frame_id;
  ref_in.reference.position.x = x;
  ref_in.reference.position.y = y;
  ref_in.reference.position.z = z;
  ref_in.reference.heading    = hdg;

  // | ------------------- issue the reference ------------------ |

  {
    auto [success, message] = referenceService(x, y, z, hdg, frame_id);

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "[%s]: reference service failed with message: '%s'", node_->get_name(), message.c_str());
      return {success, message};
    }
  }

  // | -------------------- check for result -------------------- |

  while (true) {

    if (!rclcpp::ok()) {
      return {false, "shut down from outside"};
    }

    if (!isFlyingNormally()) {
      return {false, "not flying normally"};
    }

    if (isAtPosition(x, y, z, hdg, 0.1, frame_id)) {
      return {true, "goal reached"};
    }

    sleep(0.01);
  }

  return {false, "reached end of the method without assertion"};
}

//}

/* gotoReferenceTopic() //{ */

tuple<bool, string> UAVHandler::gotoReferenceTopic(const double &x, const double &y, const double &z, const double &hdg, const std::string &frame_id) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  // | ------------------ create the reference ------------------ |

  mrs_msgs::msg::ReferenceStamped ref_in;

  ref_in.header.frame_id      = frame_id;
  ref_in.reference.position.x = x;
  ref_in.reference.position.y = y;
  ref_in.reference.position.z = z;
  ref_in.reference.heading    = hdg;

  // | ------------------- issue the reference ------------------ |

  {
    auto [success, message] = referenceTopic(x, y, z, hdg, frame_id);

    if (!success) {
      RCLCPP_ERROR(node_->get_logger(), "[%s]: reference topic failed with message: '%s'", node_->get_name(), message.c_str());
      return {success, message};
    }
  }

  // | -------------------- check for result -------------------- |

  while (true) {

    if (!rclcpp::ok()) {
      return {false, "shut down from outside"};
    }

    if (!isFlyingNormally()) {
      return {false, "not flying normally"};
    }

    if (isAtPosition(x, y, z, hdg, 0.1, frame_id)) {
      return {true, "goal reached"};
    }

    sleep(0.01);
  }

  return {false, "reached end of the method without assertion"};
}

//}

/* referenceService() //{ */

tuple<bool, string> UAVHandler::referenceService(const double &x, const double &y, const double &z, const double &hdg, const std::string &frame_id) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  std::shared_ptr<mrs_msgs::srv::ReferenceStampedSrv::Request> request = std::make_shared<mrs_msgs::srv::ReferenceStampedSrv::Request>();

  request->header.frame_id = frame_id;
  request->header.stamp    = clock_->now();

  request->reference.position.x = x;
  request->reference.position.y = y;
  request->reference.position.z = z;
  request->reference.heading    = hdg;

  {
    auto response = sch_reference_.callSync(request);

    if (!response || !response.value()->success) {
      return {false, "reference service call failed"};
    }
  }

  return {true, "reference service triggered"};
}

//}

/* referenceTopic() //{ */

tuple<bool, string> UAVHandler::referenceTopic(const double &x, const double &y, const double &z, const double &hdg, const std::string &frame_id) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  mrs_msgs::msg::ReferenceStamped msg;

  msg.header.frame_id = frame_id;
  msg.header.stamp    = clock_->now();

  msg.reference.position.x = x;
  msg.reference.position.y = y;
  msg.reference.position.z = z;
  msg.reference.heading    = hdg;

  ph_reference_.publish(msg);

  return {true, "reference published"};
}

//}

/* gotoRelativeService() //{ */

tuple<bool, string> UAVHandler::gotoRelativeService(const double &x, const double &y, const double &z, const double &hdg) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  {
    std::shared_ptr<mrs_msgs::srv::Vec4::Request> request = std::make_shared<mrs_msgs::srv::Vec4::Request>();

    request->goal[0] = x;
    request->goal[1] = y;
    request->goal[2] = z;
    request->goal[3] = hdg;

    {
      auto response = sch_goto_relative_.callSync(request);

      if (!response || !response.value()->success) {
        return {false, "goto relative service call failed"};
      }
    }
  }

  return {true, "goto relative service triggered"};
}

//}

/* gotoFcuService() //{ */

tuple<bool, string> UAVHandler::gotoFcuService(const double &x, const double &y, const double &z, const double &hdg) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  {
    std::shared_ptr<mrs_msgs::srv::Vec4::Request> request = std::make_shared<mrs_msgs::srv::Vec4::Request>();

    request->goal[0] = x;
    request->goal[1] = y;
    request->goal[2] = z;
    request->goal[3] = hdg;

    {
      auto response = sch_goto_fcu_.callSync(request);

      if (!response || !response.value()->success) {
        return {false, "goto fcu service call failed"};
      }
    }
  }

  return {true, "goto fcu service triggered"};
}

//}

/* setHeading() //{ */

tuple<bool, string> UAVHandler::setHeading(const double &setpoint) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  std::shared_ptr<mrs_msgs::srv::Vec1::Request> request = std::make_shared<mrs_msgs::srv::Vec1::Request>();

  request->goal = setpoint;

  {
    auto response = sch_set_heading_.callSync(request);

    if (!response || !response.value()->success) {
      return {false, "set heading service call failed"};
    }
  }

  // | -------------------- check for result -------------------- |

  while (true) {

    if (!rclcpp::ok()) {
      return {false, "shut down from outside"};
    }

    if (!isFlyingNormally()) {
      return {false, "not flying normally"};
    }

    auto heading = getHeading();

    if (!heading) {
      return {false, "could not obtain current heading"};
    }

    if (abs(sradians::diff(setpoint, heading.value())) < 0.1) {
      return {true, "heading goal reached"};
    }

    sleep(0.01);
  }

  return {false, "reached end of the method without assertion"};
}

//}

/* setHeadingRelative() //{ */

tuple<bool, string> UAVHandler::setHeadingRelative(const double &setpoint) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  // | -------------- remember the initial heading -------------- |

  auto initial_heading = getHeading();

  if (!initial_heading) {
    return {false, "could not obtain the initial heading"};
  }

  // | ------------------ set heading relative ------------------ |

  std::shared_ptr<mrs_msgs::srv::Vec1::Request> request = std::make_shared<mrs_msgs::srv::Vec1::Request>();

  request->goal = setpoint;

  {
    auto response = sch_set_heading_relative_.callSync(request);

    if (!response || !response.value()->success) {
      return {false, "set heading relative service call failed"};
    }
  }

  // | -------------------- check for result -------------------- |

  while (true) {

    if (!rclcpp::ok()) {
      return {false, "shut down from outside"};
    }

    if (!isFlyingNormally()) {
      return {false, "not flying normally"};
    }

    auto heading = getHeading();

    if (!heading) {
      return {false, "could not obtain current heading"};
    }

    if (abs(sradians::diff(initial_heading.value() + setpoint, heading.value())) < 0.1) {
      return {true, "heading goal reached"};
    }

    sleep(0.01);
  }

  return {false, "reached end of the method without assertion"};
}

//}

/* gotoAltitude() //{ */

tuple<bool, string> UAVHandler::gotoAltitude(const double &z) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  // | ----------------- save the initial state ----------------- |

  auto start_pose = sh_tracker_cmd_.getMsg()->position;
  auto start_hdg  = sh_tracker_cmd_.getMsg()->heading;

  // | -------------------- call the service -------------------- |

  std::shared_ptr<mrs_msgs::srv::Vec1::Request> request = std::make_shared<mrs_msgs::srv::Vec1::Request>();

  request->goal = z;

  {
    auto response = sch_goto_altitude_.callSync(request);

    if (!response || !response.value()->success) {
      return {false, "goto altitude service call failed"};
    }
  }

  // | -------------------- check for result -------------------- |

  while (true) {

    if (!rclcpp::ok()) {
      return {false, "shut down from outside"};
    }

    if (!isFlyingNormally()) {
      return {false, "not flying normally"};
    }

    if (isAtPosition(start_pose.x, start_pose.y, z, start_hdg, 0.1)) {
      return {true, "goal reached"};
    }

    sleep(0.01);
  }

  return {false, "reached end of the method without assertion"};
}

//}

/* setPathSrv() //{ */

tuple<bool, string> UAVHandler::setPathSrv(const mrs_msgs::msg::Path &path_in) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  std::shared_ptr<mrs_msgs::srv::PathSrv::Request> request = std::make_shared<mrs_msgs::srv::PathSrv::Request>();

  request->path = path_in;

  {
    auto response = sch_path_.callSync(request);

    if (!response || !response.value()->success) {
      return {false, "path service call failed"};
    }
  }

  return {true, "path set"};
}

//}

/* setPathTopic() //{ */

tuple<bool, string> UAVHandler::setPathTopic(const mrs_msgs::msg::Path &path_in) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  ph_path_.publish(path_in);

  return {true, "path set"};
}

//}

/* switchEstimator() //{ */

tuple<bool, string> UAVHandler::switchEstimator(const std::string &estimator) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  std::shared_ptr<mrs_msgs::srv::String::Request> request = std::make_shared<mrs_msgs::srv::String::Request>();
  request->value                                          = estimator;

  {
    auto response = sch_switch_estimator_.callSync(request);

    if (!response || !response.value()->success) {
      return {false, "estimator switching service call failed"};
    }
  }

  return {true, "estimator switched"};
}

//}

/* switchController() //{ */

tuple<bool, string> UAVHandler::switchController(const std::string &controller) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  std::shared_ptr<mrs_msgs::srv::String::Request> request = std::make_shared<mrs_msgs::srv::String::Request>();
  request->value                                          = controller;

  {
    auto response = sch_switch_controller_.callSync(request);

    if (!response || !response.value()->success) {
      return {false, "controller switching service call failed"};
    }
  }

  return {true, "controller switched"};
}

//}

/* switchTracker() //{ */

tuple<bool, string> UAVHandler::switchTracker(const std::string &tracker) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  std::shared_ptr<mrs_msgs::srv::String::Request> request = std::make_shared<mrs_msgs::srv::String::Request>();
  request->value                                          = tracker;

  {
    auto response = sch_switch_tracker_.callSync(request);

    if (!response || !response.value()->success) {
      return {false, "tracker switching service call failed"};
    }
  }

  return {true, "tracker switched"};
}

//}

/* setGains() //{ */

tuple<bool, string> UAVHandler::setGains(const std::string &gains) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  std::shared_ptr<mrs_msgs::srv::String::Request> request = std::make_shared<mrs_msgs::srv::String::Request>();
  request->value                                          = gains;

  {
    auto response = sch_set_gains_.callSync(request);

    if (!response || !response.value()->success) {
      return {false, "gain setting service call failed"};
    }
  }

  return {true, "gains set"};
}

//}

/* setConstraints() //{ */

tuple<bool, string> UAVHandler::setConstraints(const std::string &constraints) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  std::shared_ptr<mrs_msgs::srv::String::Request> request = std::make_shared<mrs_msgs::srv::String::Request>();
  request->value                                          = constraints;

  {
    auto response = sch_set_constraints_.callSync(request);

    if (!response || !response.value()->success) {
      return {false, "constraints setting service call failed"};
    }
  }

  return {true, "constraints set"};
}

//}

/* overrideConstraints() //{ */

tuple<bool, string> UAVHandler::overrideConstraints(const double hor_a, const double ver_a) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  std::shared_ptr<mrs_msgs::srv::ConstraintsOverride::Request> request = std::make_shared<mrs_msgs::srv::ConstraintsOverride::Request>();

  request->acceleration_horizontal = hor_a;
  request->acceleration_vertical   = ver_a;

  {
    auto response = sch_override_constraints_.callSync(request);

    if (!response || !response.value()->success) {
      return {false, "constraints override service call failed"};
    }
  }

  return {true, "constrainsts were overriden"};
}

//}

/* gotoTrajectoryStart() //{ */

tuple<bool, string> UAVHandler::gotoTrajectoryStart() {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  {
    std::shared_ptr<std_srvs::srv::Trigger::Request> request = std::make_shared<std_srvs::srv::Trigger::Request>();

    {
      auto response = sch_goto_trajectory_start_.callSync(request);

      if (!response || !response.value()->success) {
        return {false, "goto trajectory start service call failed"};
      }
    }
  }

  sleep(1.0);

  // | -------------------- check for result -------------------- |

  while (true) {

    if (!rclcpp::ok()) {
      return {false, "shut down from outside"};
    }

    if (!isFlyingNormally()) {
      return {false, "not flying normally"};
    }

    if (!hasGoal()) {
      return {true, "goal reached"};
    }

    sleep(0.01);
  }

  return {false, "reached end of the method without assertion"};
}

//}

/* startTrajectoryTracking() //{ */

tuple<bool, string> UAVHandler::startTrajectoryTracking() {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  {
    std::shared_ptr<std_srvs::srv::Trigger::Request> request = std::make_shared<std_srvs::srv::Trigger::Request>();

    {
      auto response = sch_start_trajectory_tracking_.callSync(request);

      if (!response || !response.value()->success) {
        return {false, "start trajectory tracking service call failed"};
      }
    }
  }

  sleep(1.0);

  if (sh_control_manager_diag_.getMsg()->tracker_status.tracking_trajectory) {
    return {true, "tracking trajectory"};
  }

  return {false, "failed to start trajectory tracking"};
}

//}

/* stopTrajectoryTracking() //{ */

tuple<bool, string> UAVHandler::stopTrajectoryTracking() {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  {
    std::shared_ptr<std_srvs::srv::Trigger::Request> request = std::make_shared<std_srvs::srv::Trigger::Request>();

    {
      auto response = sch_stop_trajectory_tracking_.callSync(request);

      if (!response || !response.value()->success) {
        return {false, "stop trajectory tracking service call failed"};
      }
    }
  }

  sleep(1.0);

  if (!sh_control_manager_diag_.getMsg()->tracker_status.tracking_trajectory) {
    return {true, "tracking trajectory stopped"};
  }

  return {false, "failed to stop trajectory tracking"};
}

//}

/* resumeTrajectoryTracking() //{ */

tuple<bool, string> UAVHandler::resumeTrajectoryTracking() {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  {
    std::shared_ptr<std_srvs::srv::Trigger::Request> request = std::make_shared<std_srvs::srv::Trigger::Request>();

    {
      auto response = sch_resume_trajectory_tracking_.callSync(request);

      if (!response || !response.value()->success) {
        return {false, "resume trajectory tracking service call failed"};
      }
    }
  }

  sleep(1.0);

  if (sh_control_manager_diag_.getMsg()->tracker_status.tracking_trajectory) {
    return {true, "tracking trajectory"};
  }

  return {false, "failed to resume trajectory tracking"};
}

//}

/* hover() //{ */

tuple<bool, string> UAVHandler::hover() {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  {
    std::shared_ptr<std_srvs::srv::Trigger::Request> request = std::make_shared<std_srvs::srv::Trigger::Request>();

    {
      auto response = sch_hover_.callSync(request);

      if (!response || !response.value()->success) {
        return {false, "hover service call failed"};
      }
    }
  }

  return {true, "hover triggered"};
}

//}

/* getPathSrv() //{ */

tuple<std::optional<mrs_msgs::msg::TrajectoryReference>, std::optional<Eigen::VectorXd>, string> UAVHandler::getPathSrv(const mrs_msgs::msg::Path &path_in) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return std::make_tuple<std::optional<mrs_msgs::msg::TrajectoryReference>, std::optional<Eigen::VectorXd>, string>({}, {}, std::string(std::get<1>(res)));
  }

  std::shared_ptr<mrs_msgs::srv::GetPathSrv::Request> request = std::make_shared<mrs_msgs::srv::GetPathSrv::Request>();
  request->path                                               = path_in;

  auto response = sch_get_path_.callSync(request);

  if (!response || !response.value()->success) {
    return std::make_tuple<std::optional<mrs_msgs::msg::TrajectoryReference>, std::optional<Eigen::VectorXd>, string>({}, {}, "path service call failed");
  }

  auto response_msg = *(response.value());

  Eigen::VectorXd waypoint_trajectory_idxs = Eigen::VectorXd::Zero(response_msg.waypoint_trajectory_idxs.size());

  for (size_t i = 0; i < response_msg.waypoint_trajectory_idxs.size(); i++) {
    waypoint_trajectory_idxs(i) = response_msg.waypoint_trajectory_idxs.at(i);
  }

  return {response_msg.trajectory, waypoint_trajectory_idxs, "path set"};
}

//}

/* validateReference() //{ */

tuple<bool, string> UAVHandler::validateReference(const mrs_msgs::msg::ReferenceStamped &msg) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return res;
  }

  std::shared_ptr<mrs_msgs::srv::ValidateReference::Request> request = std::make_shared<mrs_msgs::srv::ValidateReference::Request>();

  request->reference = msg;

  {
    auto response = sch_validate_reference_.callSync(request);

    if (!response) {
      return {false, "reference validation service call failed"};
    } else {
      return {response.value()->success, response.value()->message};
    }
  }
}

//}

/* transformReference() //{ */

std::tuple<bool, std::optional<std::string>, std::optional<mrs_msgs::msg::ReferenceStamped>>
UAVHandler::transformReference(const mrs_msgs::msg::ReferenceStamped &msg, std::string target_frame) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return {};
  }

  std::shared_ptr<mrs_msgs::srv::TransformReferenceSrv::Request> request = std::make_shared<mrs_msgs::srv::TransformReferenceSrv::Request>();

  request->reference = msg;
  request->frame_id  = target_frame;

  {
    auto response = sch_tranform_reference_.callSync(request);

    if (!response.value()->success) {
      return {false, "transform reference service call failed", {}};
    } else {
      return {response.value()->success, response.value()->message, response.value()->reference};
    }
  }
}

//}

/* transformPose() //{ */

std::tuple<bool, std::optional<std::string>, std::optional<geometry_msgs::msg::PoseStamped>>
UAVHandler::transformPose(const geometry_msgs::msg::PoseStamped &msg, std::string target_frame) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return {};
  }

  std::shared_ptr<mrs_msgs::srv::TransformPoseSrv::Request> request = std::make_shared<mrs_msgs::srv::TransformPoseSrv::Request>();

  request->pose     = msg;
  request->frame_id = target_frame;

  {
    auto response = sch_tranform_pose_.callSync(request);

    if (!response.value()->success) {
      return {false, "transform pose service call failed", {}};
    } else {
      return {response.value()->success, response.value()->message, response.value()->pose};
    }
  }
}

//}

/* transformVector3() //{ */

std::tuple<bool, std::optional<std::string>, std::optional<geometry_msgs::msg::Vector3Stamped>>
UAVHandler::transformVector3(const geometry_msgs::msg::Vector3Stamped &msg, std::string target_frame) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return {};
  }

  std::shared_ptr<mrs_msgs::srv::TransformVector3Srv::Request> request = std::make_shared<mrs_msgs::srv::TransformVector3Srv::Request>();

  request->vector   = msg;
  request->frame_id = target_frame;

  {
    auto response = sch_tranform_vector3_.callSync(request);

    if (!response.value()->success) {
      return {false, "transform vector service call failed", {}};
    } else {
      return {response.value()->success, response.value()->message, response.value()->vector};
    }
  }
}

//}

/* ValidateReferenceArray() //{ */

tuple<bool, std::optional<mrs_msgs::srv::ValidateReferenceArray::Response>>
UAVHandler::validateReferenceArray(const mrs_msgs::srv::ValidateReferenceArray::Request &request_in) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return {false, {}};
  }

  std::shared_ptr<mrs_msgs::srv::ValidateReferenceArray::Request> request = std::make_shared<mrs_msgs::srv::ValidateReferenceArray::Request>(request_in);

  {
    auto response = sch_validate_reference_array_.callSync(request);

    if (!response) {
      return {false, {}};
    } else {
      return {true, *(response.value())};
    }
  }
}

//}

/* sleep() //{ */

void UAVHandler::sleep(const double &duration) {

  clock_->sleep_for(std::chrono::duration<double>(duration));
}

//}

/* sleep() //{ */

void TestGeneric::sleep(const double &duration) {

  clock_->sleep_for(std::chrono::duration<double>(duration));
}

//}

// | ------------------------- getters ------------------------ |

/* getHeightAgl() //{ */

std::optional<double> UAVHandler::getHeightAgl(void) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return {};
  }

  if (sh_height_agl_.hasMsg()) {
    return {sh_height_agl_.getMsg()->value};
  } else {
    return {};
  }
}

//}

/* getCurrentConstraints() //{ */

std::optional<mrs_msgs::msg::DynamicsConstraints> UAVHandler::getCurrentConstraints(void) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return {};
  }

  if (sh_current_constraints_.hasMsg()) {
    return {*sh_current_constraints_.getMsg()};
  } else {
    return {};
  }
}

//}

/* getTrackerCmd() //{ */

std::optional<mrs_msgs::msg::TrackerCommand> UAVHandler::getTrackerCmd(void) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return {};
  }

  if (sh_tracker_cmd_.hasMsg()) {
    return {*sh_tracker_cmd_.getMsg()};
  } else {
    return {};
  }
}

//}

/* isAtPosition(const double &x, const double &y, const double &z, const double &hdg, const double &pos_tolerance, const std::string frame_id) //{ */

bool UAVHandler::isAtPosition(const double &x, const double &y, const double &z, const double &hdg, const double &pos_tolerance, const std::string frame_id) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return false;
  }

  if (!sh_uav_state_.hasMsg()) {
    return false;
  }

  auto heading = getHeading();

  if (!heading) {
    return false;
  }

  auto uav_state = sh_uav_state_.getMsg();

  // | ---- transform the input to the current control frame ---- |

  mrs_msgs::msg::ReferenceStamped ref_in;

  ref_in.header.frame_id      = frame_id == "" ? uav_state->header.frame_id : frame_id;
  ref_in.reference.position.x = x;
  ref_in.reference.position.y = y;
  ref_in.reference.position.z = z;
  ref_in.reference.heading    = hdg;

  auto ref_transformed = transformer_->transformSingle(ref_in, uav_state->header.frame_id);

  if (!ref_transformed) {
    return false;
  }

  if (abs(ref_transformed->reference.position.x - uav_state->pose.position.x) < pos_tolerance &&
      abs(ref_transformed->reference.position.y - uav_state->pose.position.y) < pos_tolerance &&
      abs(ref_transformed->reference.position.z - uav_state->pose.position.z) < pos_tolerance &&
      abs(sradians::diff(ref_transformed->reference.heading, heading.value())) < 0.2) {

    return true;

  } else {

    return false;
  }
}

//}

/* isAtPosition(const double &x, const double &y, const double &hdg, const double &pos_tolerance, const std::string frame_id) //{ */

bool UAVHandler::isAtPosition(const double &x, const double &y, const double &hdg, const double &pos_tolerance, const std::string frame_id) {

  auto res = checkPreconditions();

  if (!(std::get<0>(res))) {
    return false;
  }

  if (!sh_uav_state_.hasMsg()) {
    return false;
  }

  auto heading = getHeading();

  if (!heading) {
    return false;
  }

  auto uav_state = sh_uav_state_.getMsg();

  // | ---- transform the input to the current control frame ---- |

  mrs_msgs::msg::ReferenceStamped ref_in;

  ref_in.header.frame_id      = frame_id == "" ? uav_state->header.frame_id : frame_id;
  ref_in.reference.position.x = x;
  ref_in.reference.position.y = y;
  ref_in.reference.position.z = uav_state->pose.position.z;
  ref_in.reference.heading    = hdg;

  auto ref_transformed = transformer_->transformSingle(ref_in, uav_state->header.frame_id);

  if (!ref_transformed) {
    return false;
  }

  if (abs(ref_transformed->reference.position.x - uav_state->pose.position.x) < pos_tolerance &&
      abs(ref_transformed->reference.position.y - uav_state->pose.position.y) < pos_tolerance &&
      abs(sradians::diff(ref_transformed->reference.heading, heading.value())) < 0.2) {

    return true;

  } else {

    return false;
  }
}

//}

/* isReferenceAtPosition() //{ */

bool UAVHandler::isReferenceAtPosition(const double &x, const double &y, const double &z, const double &hdg, const double &pos_tolerance) {

  if (!sh_tracker_cmd_.hasMsg()) {
    return false;
  }

  auto tracker_cmd = sh_tracker_cmd_.getMsg();

  if (abs(x - tracker_cmd->position.x) < pos_tolerance && abs(y - tracker_cmd->position.y) < pos_tolerance &&
      abs(z - tracker_cmd->position.z) < pos_tolerance && abs(sradians::diff(hdg, tracker_cmd->heading)) < 0.2) {

    return true;

  } else {

    return false;
  }
}

//}

/* getActiveTracker() //{ */

std::string UAVHandler::getActiveTracker(void) {

  if (!sh_control_manager_diag_.getMsg()) {
    return "";
  }

  return sh_control_manager_diag_.getMsg()->active_tracker;
}

//}

/* getActiveController() //{ */

std::string UAVHandler::getActiveController(void) {

  if (!sh_control_manager_diag_.getMsg()) {
    return "";
  }

  return sh_control_manager_diag_.getMsg()->active_controller;
}

//}

/* getActiveEstimator() //{ */

std::string UAVHandler::getActiveEstimator(void) {

  if (!sh_estim_manager_diag_.getMsg()) {
    return "";
  }

  return sh_estim_manager_diag_.getMsg()->current_state_estimator;
}

//}

/* hasGoal() //{ */

bool UAVHandler::hasGoal(void) {

  if (sh_control_manager_diag_.hasMsg()) {
    return sh_control_manager_diag_.getMsg()->tracker_status.have_goal;
  } else {
    return false;
  }
}

//}

/* mrsSystemReady() //{ */

bool UAVHandler::mrsSystemReady(void) {

  bool got_control_manager_diag     = sh_control_manager_diag_.hasMsg();
  bool got_uav_manager_diag         = sh_uav_manager_diag_.hasMsg();
  bool got_gain_manager_diag        = sh_gain_manager_diag_.hasMsg();
  bool got_safety_area_manager_diag = sh_safety_area_manager_diag_.hasMsg();
  bool got_constraint_manager_diag  = sh_constraint_manager_diag_.hasMsg();
  bool got_estimation_manager_diag  = sh_estim_manager_diag_.hasMsg();
  bool got_uav_state                = sh_uav_state_.hasMsg();

  return got_control_manager_diag && got_estimation_manager_diag && got_uav_manager_diag && got_gain_manager_diag && got_constraint_manager_diag &&
         got_uav_state && got_safety_area_manager_diag;
}

//}

/* isFlyingNormally() //{ */

bool UAVHandler::isFlyingNormally(void) {

  if (sh_control_manager_diag_.hasMsg()) {
    return sh_control_manager_diag_.getMsg()->flying_normally;
  } else {
    return false;
  }
}

//}

/* isStationary() //{ */

std::optional<bool> UAVHandler::isStationary(void) {

  if (!sh_control_manager_diag_.hasMsg()) {
    return {};
  }

  auto ctrl_diag = sh_control_manager_diag_.getMsg();

  std::optional<double> speed = getSpeed();

  if (!speed) {
    return {};
  }

  if (!ctrl_diag->tracker_status.have_goal && std::abs(speed.value()) < 0.2) {
    return true;
  } else {
    return false;
  }
}

//}

/* isOutputEnabled() //{ */

bool UAVHandler::isOutputEnabled(void) {

  if (sh_control_manager_diag_.hasMsg()) {
    return sh_control_manager_diag_.getMsg()->output_enabled;
  } else {
    return false;
  }
}

//}

/* isArmed() //{ */

bool UAVHandler::isArmed(void) {

  if (sh_hw_api_status_.hasMsg()) {
    return sh_hw_api_status_.getMsg()->armed;
  } else {
    return false;
  }
}

//}

/* isInOffboard() //{ */

bool UAVHandler::isInOffboard(void) {

  if (sh_hw_api_status_.hasMsg()) {
    return sh_hw_api_status_.getMsg()->offboard;
  } else {
    return false;
  }
}

//}

/* getSpeed() //{ */

std::optional<double> UAVHandler::getSpeed(void) {

  if (!sh_speed_.hasMsg()) {
    return {};
  }

  return sh_speed_.getMsg()->value;
}

//}

/* getHeading() //{ */

std::optional<double> UAVHandler::getHeading(void) {

  if (!sh_uav_state_.hasMsg()) {
    return {};
  }

  auto uav_state = sh_uav_state_.getMsg();

  double heading;

  try {
    heading = mrs_lib::AttitudeConverter(uav_state->pose.orientation).getHeading();
  }
  catch (...) {
    return {};
  }

  return {heading};
}

//}

/* getVelocity() //{ */

std::optional<Eigen::Vector3d> UAVHandler::getVelocity(const std::string frame_id) {

  if (!sh_uav_state_.hasMsg()) {
    return {};
  }

  auto uav_state = sh_uav_state_.getMsg();

  geometry_msgs::msg::Vector3Stamped vel_world;

  vel_world.header = uav_state->header;

  vel_world.vector.x = uav_state->velocity.linear.x;
  vel_world.vector.y = uav_state->velocity.linear.y;
  vel_world.vector.z = uav_state->velocity.linear.z;

  auto res = transformer_->transformSingle(vel_world, frame_id);

  if (res) {
    return Eigen::Vector3d(res->vector.x, res->vector.y, res->vector.z);
  } else {
    return {};
  }
}

//}

} // namespace mrs_uav_testing
