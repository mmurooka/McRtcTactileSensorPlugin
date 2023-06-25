#include <McRtcTactileSensorPlugin/TactileSensorPlugin.h>

namespace mc_plugin
{

/** \brief Returns the plugin configuration. */
mc_control::GlobalPlugin::GlobalPluginConfiguration TactileSensorPlugin::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = false;
  out.should_always_run = true;
  return out;
}

} // namespace mc_plugin
