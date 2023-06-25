#pragma once

#include <mc_control/GlobalPlugin.h>

namespace mc_plugin
{

/** \brief Tactile sensor plugin. */
struct TactileSensorPlugin : public mc_control::GlobalPlugin
{
  /** \brief Destructor. */
  ~TactileSensorPlugin() override{};

  /** \brief Returns the plugin configuration. */
  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  /** \brief Initialize the plugin. */
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override{};

  /** \brief Reset the plugin. */
  void reset(mc_control::MCGlobalController & controller) override{};

  /** \brief Run before mc_control::MCGlobalController::run() */
  inline void before(mc_control::MCGlobalController &) override {}

  /** \brief Run after mc_control::MCGlobalController::run() */
  void after(mc_control::MCGlobalController & controller) override{};
};

} // namespace mc_plugin
