#include <mc_control/GlobalPluginMacros.h>

#include <McRtcTactileSensorPlugin/TactileSensorPlugin.h>

namespace mc_plugin
{

mc_control::GlobalPlugin::GlobalPluginConfiguration TactileSensorPlugin::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = false;
  out.should_always_run = true;
  return out;
}

void TactileSensorPlugin::init(mc_control::MCGlobalController & gc, const mc_rtc::Configuration & config)
{
  const auto & controller = gc.controller();
  const auto & robot = controller.robot();

  // Load configuration
  if(!config.has("sensorTopicName"))
  {
    mc_rtc::log::error_and_throw(
        "[mc_plugin::TactileSensorPlugin] The \"sensorTopicName\" key must be specified in the configuration.");
  }
  std::string sensorTopicName = static_cast<std::string>(config("sensorTopicName"));
  if(!config.has("forceSensorName"))
  {
    mc_rtc::log::error_and_throw(
        "[mc_plugin::TactileSensorPlugin] The \"forceSensorName\" key must be specified in the configuration.");
  }
  forceSensorName_ = static_cast<std::string>(config("forceSensorName"));
  if(!robot.hasForceSensor(forceSensorName_))
  {
    mc_rtc::log::error_and_throw("[mc_plugin::TactileSensorPlugin] No force sensor named \"{}\" exists.",
                                 forceSensorName_);
  }

  // Setup ROS
  nh_ = std::make_unique<ros::NodeHandle>();
  // Use a dedicated queue so as not to call callbacks of other modules
  nh_->setCallbackQueue(&callbackQueue_);
  sensorSub_ = nh_->subscribe<mujoco_tactile_sensor_plugin::TactileSensorData>(
      sensorTopicName, 1, &TactileSensorPlugin::sensorCallback, this);

  reset(gc);

  mc_rtc::log::success("[mc_plugin] Initialized TactileSensorPlugin.");
}

void TactileSensorPlugin::reset(mc_control::MCGlobalController & // gc
)
{
  sensorMsg_ = nullptr;
}

void TactileSensorPlugin::before(mc_control::MCGlobalController & gc)
{
  auto & controller = gc.controller();
  auto & robot = controller.robot();

  // Call ROS callback
  callbackQueue_.callAvailable(ros::WallDuration());

  // Set measured wrench
  robot.data()
      ->forceSensors.at(robot.data()->forceSensorsIndex.at(forceSensorName_))
      .wrench(sva::ForceVecd(Eigen::Vector3d(10, 0, 0),
                             Eigen::Vector3d(10, 0, 0))); // \todo Calculate wrench from sensorMsg_
}

void TactileSensorPlugin::sensorCallback(const mujoco_tactile_sensor_plugin::TactileSensorData::ConstPtr & sensorMsg)
{
  sensorMsg_ = std::make_shared<mujoco_tactile_sensor_plugin::TactileSensorData>(*sensorMsg);
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("TactileSensor", mc_plugin::TactileSensorPlugin)
