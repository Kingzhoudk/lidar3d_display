#ifndef CONFIG_GFLAGS_H
#define CONFIG_GFLAGS_H_

#include <gflags/gflags.h>

// System gflags
DECLARE_string(unreal_node_name);
DECLARE_string(unreal_module_name);

DECLARE_string(unreal_adapter_config_filename);

// data file


// Canbus gflags
DECLARE_double(chassis_freq);
DECLARE_int64(min_cmd_interval);

// chassis_detail message publish
DECLARE_bool(enable_chassis_detail_pub);

// unreal test files
DECLARE_string(unreal_test_file);

//sensor name
DECLARE_string(gps_name);

//direct topic name
DECLARE_string(odometry_topic);
DECLARE_string(corr_imu_topic);

//Canbus  chassis name
DECLARE_string(canbus_node_name);

#endif
