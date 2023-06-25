#include <mc_control/GlobalPlugin.h>

namespace mc_plugin
{

struct TactileSensorPlugin : public mc_control::GlobalPlugin
{
  ~TactileSensorPlugin() override{};

  inline mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override
  {
    mc_control::GlobalPlugin::GlobalPluginConfiguration out;
    out.should_run_before = true;
    out.should_run_after = false;
    out.should_always_run = true;
    return out;
  }

  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override{};

  void reset(mc_control::MCGlobalController & controller) override{};

  inline void before(mc_control::MCGlobalController &) override {}

  void after(mc_control::MCGlobalController & controller) override{};
};

} // namespace mc_plugin
