#include <string>
#include <vector>
bool sim_flag;

// Simulation keys 
std::string JOINT_ANGLES_KEY = "sai2::sim::painting::kuka::sensors::q";
std::string JOINT_VELOCITIES_KEY = "sai2::sim::painting::kuka::sensors::dq";
std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::sim::painting::kuka::actuators::fgc";
std::string CONTROLLER_RUNNING_KEY = "sai2::sim::painting::kuka::controller";
std::string ROBOT_SENSED_FORCE_KEY = "sai2::sim::painting::sensors::sensed_force";
std::string MASSMATRIX_KEY;
std::string CORIOLIS_KEY;


// Controller keys. (Simulation)
// Controller
std::string ROBOT_COMMAND_TORQUES_KEY = "sai2::sim::painting::kuka::actuators::fgc";  // alias for joint torques commanded key
std::string ROBOT_PROXY_KEY = "sai2::sim::painting::dual_proxy::robot_proxy";
std::string ROBOT_PROXY_ROT_KEY = "sai2::sim::painting::dual_proxy::robot_proxy_rot";
std::string HAPTIC_PROXY_KEY = "sai2::sim::painting::dual_proxy::haptic_proxy";
std::string FORCE_SPACE_DIMENSION_KEY = "sai2::sim::painting::dual_proxy::force_space_dimension";
std::string SIGMA_FORCE_KEY = "sai2::sim::painting::dual_proxy::sigma_force";
std::string ROBOT_DEFAULT_ROT_KEY = "sai2::sim::painting::dual_proxy::robot_default_rot";
std::string ROBOT_DEFAULT_POS_KEY = "sai2::sim::painting::dual_proxy::robot_default_pos";
std::string HAPTIC_DEVICE_READY_KEY = "sai2::sim::painting::dual_proxy::haptic_device_ready";

// Haptics 
std::vector<std::string> DEVICE_MAX_STIFFNESS_KEYS = {
	"sai2::ChaiHapticDevice::device0::specifications::max_stiffness",
	"sai2::ChaiHapticDevice::device1::specifications::max_stiffness",
	};
std::vector<std::string> DEVICE_MAX_DAMPING_KEYS = {
	"sai2::ChaiHapticDevice::device0::specifications::max_damping",
	"sai2::ChaiHapticDevice::device1::specifications::max_damping",
	};
std::vector<std::string> DEVICE_MAX_FORCE_KEYS = {
	"sai2::ChaiHapticDevice::device0::specifications::max_force",
	"sai2::ChaiHapticDevice::device1::specifications::max_force",
	};
// Set force and torque feedback of the haptic device
std::vector<std::string> DEVICE_COMMANDED_FORCE_KEYS = {
	"sai2::ChaiHapticDevice::device0::actuators::commanded_force",
	"sai2::ChaiHapticDevice::device1::actuators::commanded_force",
	};
std::vector<std::string> DEVICE_COMMANDED_TORQUE_KEYS = {
	"sai2::ChaiHapticDevice::device0::actuators::commanded_torque",
	"sai2::ChaiHapticDevice::device1::actuators::commanded_torque",
	};
std::vector<std::string> DEVICE_COMMANDED_GRIPPER_FORCE_KEYS = {
	"sai2::ChaiHapticDevice::device0::actuators::commanded_force_gripper",
	"sai2::ChaiHapticDevice::device1::actuators::commanded_force_gripper",
	};
// Haptic device current position and rotation
std::vector<std::string> DEVICE_POSITION_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::current_position",
	"sai2::ChaiHapticDevice::device1::sensors::current_position",
	};
std::vector<std::string> DEVICE_ROTATION_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::current_rotation",
	"sai2::ChaiHapticDevice::device1::sensors::current_rotation",
	};
std::vector<std::string> DEVICE_GRIPPER_POSITION_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::current_position_gripper",
	"sai2::ChaiHapticDevice::device1::sensors::current_position_gripper",
	};
// Haptic device current velocity
std::vector<std::string> DEVICE_TRANS_VELOCITY_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::current_trans_velocity",
	"sai2::ChaiHapticDevice::device1::sensors::current_trans_velocity",
	};
std::vector<std::string> DEVICE_ROT_VELOCITY_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::current_rot_velocity",
	"sai2::ChaiHapticDevice::device1::sensors::current_rot_velocity",
	};
std::vector<std::string> DEVICE_GRIPPER_VELOCITY_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::current_gripper_velocity",
	"sai2::ChaiHapticDevice::device1::sensors::current_gripper_velocity",
	};
std::vector<std::string> DEVICE_SENSED_FORCE_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::sensed_force",
	"sai2::ChaiHapticDevice::device1::sensors::sensed_force",
	};
std::vector<std::string> DEVICE_SENSED_TORQUE_KEYS = {
	"sai2::ChaiHapticDevice::device0::sensors::sensed_torque",
	"sai2::ChaiHapticDevice::device1::sensors::sensed_torque",
	};
