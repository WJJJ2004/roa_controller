#include "pluginlib/class_list_macros.hpp"
#include "roa_policy_controller/policy_controller.hpp"

PLUGINLIB_EXPORT_CLASS(
  roa_policy_controller::RoaPolicyController,
  controller_interface::ControllerInterface
)