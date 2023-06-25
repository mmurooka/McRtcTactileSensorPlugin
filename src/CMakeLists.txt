set(AUTOLOAD_TactileSensor_PLUGIN
    OFF
    CACHE INTERNAL "Automatically load TactileSensor plugin")
add_plugin(TactileSensor TactileSensorPlugin.cpp)
target_link_libraries(TactileSensor
                      PUBLIC PkgConfig::mujoco_tactile_sensor_plugin)
