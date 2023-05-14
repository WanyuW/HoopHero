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
