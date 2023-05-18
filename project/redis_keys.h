/**
 * @file redis_keys.h
 * @brief Contains all redis keys for simulation and control.
 * 
 */

const std::string JOINT_ANGLES_KEY = "sai2::sim::panda::sensors::q";
const std::string JOINT_VELOCITIES_KEY = "sai2::sim::panda::sensors::dq";
const std::string JOINT_TORQUES_COMMANDED_KEY = "sai2::sim::panda::actuators::fgc";
const std::string CONTROLLER_RUNNING_KEY = "sai2::sim::panda::controller";

// define keys for project
const std::string HOOP_EE_POS = "sai2::hoop::ee_pos";
const std::string HOOP_EE_VEL = "sai2::hoop::ee_vel";
const std::string SHOOTER_POWER = "sai2::shooter::power";
const std::string BALL_POS = "sai2::sim::basketball_pos";
const std::string BALL_VEL = "sai2::sim::basketball_vel";
const std::string SHOOTER_EE_POS = "sai2::shooter::ee_pos";
const std::string SHOOTER_EE_VEL = "sai2::shooter::ee_vel";
const std::string JOINT_ANGLES_KEY_SHOOTER = "sai2::sim::kuka::sensors::q";
const std::string JOINT_VELOCITIES_KEY_SHOOTER = "sai2::sim::kuka::sensors::dq";
const std::string JOINT_TORQUES_COMMANDED_KEY_SHOOTER = "sai2::sim::kuka::actuators::fgc";
const std::string HOOP_STATE_KEY = "sai2::sim::hoop::state";
const std::string SHOOTER_MODE = "sai2::sim::shooter::mode";
const std::string FUTURE_POS = "sai2::sim::basketball_future_pos";
const std::string GAME_STATE = "sai2::game_state";


