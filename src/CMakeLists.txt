set(AUTOLOAD_TactileSensor_PLUGIN
    OFF
    CACHE INTERNAL "Automatically load TactileSensor plugin")
add_plugin(TactileSensor TactileSensorPlugin.cpp)
