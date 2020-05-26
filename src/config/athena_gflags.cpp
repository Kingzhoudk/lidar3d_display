#include "athena_gflags.h"

// data file
DEFINE_string(conf_file, "../src/config/athena_conf.pb.txt",
              "Default unreal conf file");

DEFINE_string(conf_file_vio, "../src/config/athena_conf_vio.pb.txt",
              "Default unreal conf file");
// Canbus gflags
DEFINE_double(chassis_freq, 100, "Chassis feedback timer frequency.");
DEFINE_int64(min_cmd_interval, 5, "Minimum control command interval in ms.");

// chassis_detail message publish
DEFINE_bool(enable_chassis_detail_pub, false, "Chassis Detail message publish");

// unreal test files
DEFINE_string(unreal_test_file, "modules/unreal/testdata/unreal_test.pb.txt",
              "unreal tester input test file, in ControlCommand pb format.");

//sensor name
DEFINE_string(gps_name, "unreal GPS", "Module name");

//direct topic name
DEFINE_string(odometry_topic,"/apollo/sensor/direct/odometry", "direct topic name");
DEFINE_string(corr_imu_topic,"/apollo/sensor/direct/corrected_imu", "direct topic name");

//Canbus  chassis name
DEFINE_string(canbus_node_name, "chassis", "The chassis module name in proto");