#pragma once

#include <mc_control/GlobalPlugin.h>

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <mujoco_tactile_sensor_plugin/TactileSensorData.h>

namespace mc_plugin
{

/** \brief Tactile sensor plugin. */
struct TactileSensorPlugin : public mc_control::GlobalPlugin
{
public:
  /** \brief Destructor. */
  ~TactileSensorPlugin();

  /** \brief Returns the plugin configuration. */
  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  /** \brief Initialize the plugin. */
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;

  /** \brief Reset the plugin. */
  void reset(mc_control::MCGlobalController & controller) override;

  /** \brief Run before mc_control::MCGlobalController::run() */
  void before(mc_control::MCGlobalController & controller) override;

  /** \brief Run after mc_control::MCGlobalController::run() */
  void after(mc_control::MCGlobalController & controller) override;

protected:
  /** \brief ROS callback of sensor topic. */
  void sensorCallback(const mujoco_tactile_sensor_plugin::TactileSensorData::ConstPtr & sensorMsg);

protected:
  //! ROS variables
  //! @{
  std::unique_ptr<ros::NodeHandle> nh_;
  ros::CallbackQueue callbackQueue_;
  ros::Subscriber sensorSub_;
  //! @}
};

} // namespace mc_plugin
