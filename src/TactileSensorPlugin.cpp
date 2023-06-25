#include <mc_control/GlobalPluginMacros.h>

#include <McRtcTactileSensorPlugin/TactileSensorPlugin.h>

namespace mc_plugin
{

TactileSensorPlugin::~TactileSensorPlugin()
{
  // \todo Set wrench zero
}

mc_control::GlobalPlugin::GlobalPluginConfiguration TactileSensorPlugin::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = false;
  out.should_always_run = true;
  return out;
}

void TactileSensorPlugin::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  std::string sensorTopicName = config("sensorTopicName", std::string("tactile_sensor"));

  // Setup ROS
  nh_ = std::make_unique<ros::NodeHandle>();
  // Use a dedicated queue so as not to call callbacks of other modules
  nh_->setCallbackQueue(&callbackQueue_);
  sensorSub_ = nh_->subscribe<mujoco_tactile_sensor_plugin::TactileSensorData>(
      sensorTopicName, 1, &TactileSensorPlugin::sensorCallback, this);
}

void TactileSensorPlugin::reset(mc_control::MCGlobalController & // controller
)
{
}

void TactileSensorPlugin::before(mc_control::MCGlobalController & controller) {}

void TactileSensorPlugin::after(mc_control::MCGlobalController & // controller
)
{
}

void TactileSensorPlugin::sensorCallback(const mujoco_tactile_sensor_plugin::TactileSensorData::ConstPtr & sensorMsg) {}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("TactileSensor", mc_plugin::TactileSensorPlugin)
