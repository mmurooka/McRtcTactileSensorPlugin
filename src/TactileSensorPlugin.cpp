#include <mc_control/GlobalPluginMacros.h>

#include <McRtcTactileSensorPlugin/TactileSensorPlugin.h>
#include <eigen_conversions/eigen_msg.h>

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
  if(!config.has("tactileSensorFrameName"))
  {
    mc_rtc::log::error_and_throw(
        "[mc_plugin::TactileSensorPlugin] The \"tactileSensorFrameName\" key must be specified in the configuration.");
  }
  tactileSensorFrameName_ = static_cast<std::string>(config("tactileSensorFrameName"));
  if(!robot.hasFrame(tactileSensorFrameName_))
  {
    mc_rtc::log::error_and_throw("[mc_plugin::TactileSensorPlugin] No frame named \"{}\" exists.",
                                 tactileSensorFrameName_);
  }
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
  auto & forceSensor = robot.data()->forceSensors.at(robot.data()->forceSensorsIndex.at(forceSensorName_));

  // Call ROS callback
  callbackQueue_.callAvailable(ros::WallDuration());

  // Set measured wrench
  sva::ForceVecd wrench = sva::ForceVecd::Zero();
  if(sensorMsg_)
  {
    // Calculate wrench by integrating tactile sensor data
    for(size_t i = 0; i < sensorMsg_->forces.size(); i++)
    {
      Eigen::Vector3d position;
      Eigen::Vector3d normal;
      tf::pointMsgToEigen(sensorMsg_->positions[i], position);
      tf::vectorMsgToEigen(sensorMsg_->normals[i], normal);
      Eigen::Vector3d force = sensorMsg_->forces[i] * normal;
      Eigen::Vector3d moment = position.cross(force);
      wrench.force() += force;
      wrench.moment() += moment;
    }

    // Transform from tactile sensor frame to force sensor frame
    sva::PTransformd tactileSensorPose = robot.frame(tactileSensorFrameName_).position();
    sva::PTransformd forceSensorPose = forceSensor.X_fsmodel_fsactual() * forceSensor.X_0_f(robot);
    sva::PTransformd tactileToForceTrans = forceSensorPose * tactileSensorPose.inv();
    wrench = tactileToForceTrans.dualMul(wrench);
  }
  forceSensor.wrench(wrench);
}

void TactileSensorPlugin::sensorCallback(const mujoco_tactile_sensor_plugin::TactileSensorData::ConstPtr & sensorMsg)
{
  sensorMsg_ = std::make_shared<mujoco_tactile_sensor_plugin::TactileSensorData>(*sensorMsg);
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("TactileSensor", mc_plugin::TactileSensorPlugin)
