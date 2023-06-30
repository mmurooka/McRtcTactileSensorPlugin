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
protected:
  //! Sensor information
  struct SensorInfo
  {
    //! Topic name
    std::string topicName;

    //! Frame name of tactile sensor
    std::string tactileSensorFrameName;

    //! Force sensor name
    std::string forceSensorName;
  };

public:
  /** \brief Returns the plugin configuration. */
  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  /** \brief Initialize the plugin. */
  void init(mc_control::MCGlobalController & gc, const mc_rtc::Configuration & config) override;

  /** \brief Reset the plugin. */
  void reset(mc_control::MCGlobalController & gc) override;

  /** \brief Run before mc_control::MCGlobalController::run() */
  void before(mc_control::MCGlobalController & gc) override;

  /** \brief Run after mc_control::MCGlobalController::run() */
  inline void after(mc_control::MCGlobalController & gc) override{};

protected:
  /** \brief ROS callback of sensor topic.
      \param sensorMsg sensor message
      \param sensorIdx sensor index
   */
  void sensorCallback(const mujoco_tactile_sensor_plugin::TactileSensorData::ConstPtr & sensorMsg, size_t sensorIdx);

protected:
  //! Sensor information list
  std::vector<SensorInfo> sensorInfoList_;

  //! Sensor message list
  std::vector<std::shared_ptr<mujoco_tactile_sensor_plugin::TactileSensorData>> sensorMsgList_;

  //! Original wrench list (only for logging)
  std::unordered_map<std::string, sva::ForceVecd> origWrenchList_;

  //! ROS variables
  //! @{
  std::unique_ptr<ros::NodeHandle> nh_;
  ros::CallbackQueue callbackQueue_;
  std::vector<ros::Subscriber> sensorSubList_;
  //! @}
};

} // namespace mc_plugin
