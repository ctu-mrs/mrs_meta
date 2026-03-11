#include <mrs_uav_state_estimators/estimators/state/state_generic.h>

namespace mrs_point_lio_estimator_plugin
{

const char estimator_name[] = "point_lio_2";
const bool is_core_plugin   = false;

class Estimator2 : public rclcpp::Node, public mrs_uav_state_estimators::StateGeneric {
public:
  Estimator2() : rclcpp::Node(estimator_name), mrs_uav_state_estimators::StateGeneric(estimator_name, is_core_plugin) {
  }

  ~Estimator2(void) {
  }
};

}  // namespace mrs_point_lio_estimator_plugin

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mrs_point_lio_estimator_plugin::Estimator2, mrs_uav_managers::StateEstimator)
