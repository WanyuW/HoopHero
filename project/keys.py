

# Hoop (mmp_panda keys)
JOINT_ANGLES_KEY = "sai2::sim::panda::sensors::q"
JOINT_VELOCITIES_KEY = "sai2::sim::panda::sensors::dq"
JOINT_TORQUES_COMMANDED_KEY = "sai2::sim::panda::actuators::fgc"

HOOP_EE_POS = "sai2::hoop::ee_pos"
HOOP_EE_VEL = "sai2::hoop::ee_vel"

# Basketball keys
OBJ_JOINT_ANGLES_KEY  = "cs225a::object::ball::sensors::q"
OBJ_JOINT_VELOCITIES_KEY = "cs225a::object::ball::sensors::dq"
BALL_POS = "sai2::sim::basketball_pos"
BALL_VEL = "sai2::sim::basketball_vel"
FUTURE_POS = "sai2::sim::basketball_future_pos"

# camera keys
CAMERA_POS_KEY = "cs225a::camera::pos"
CAMERA_ORI_KEY = "cs225a::camera::ori"
CAMERA_DETECT_KEY = "cs225a::camera::detect"
CAMERA_OBJ_POS_KEY = "cs225a::camera::obj_pos"

# Simulation/Controller keys
SIMULATION_LOOP_DONE_KEY = "cs225a::simulation::done"
CONTROLLER_LOOP_DONE_KEY = "cs225a::controller::done"
CONTROLLER_RUNNING_KEY = "sai2::sim::panda::controller"

# Shooter keys
SHOOTER_POWER = "sai2::shooter::power"
SHOOTER_EE_POS = "sai2::shooter::ee_pos"
SHOOTER_EE_VEL = "sai2::shooter::ee_vel"
JOINT_ANGLES_KEY_SHOOTER = "sai2::sim::kuka::sensors::q"
JOINT_VELOCITIES_KEY_SHOOTER = "sai2::sim::kuka::sensors::dq"
SHOOTING_ANGLE = "sai2::shooter::shooting_angle"
JOINT_TORQUES_COMMANDED_KEY_SHOOTER = "sai2::sim::kuka::actuators::fgc"
SHOOTER_EE_POS_INWORLD = "sai2::shooter::ee_pos_inworld"
SHOOTER_MODE = "sai2::sim::shooter::mode"

# Game state keys
GAME_STATE = "sai2::game_state"
RESET_KEY = "sai2::sim::reset"
HOOP_STATE_KEY = "sai2::sim::hoop::state"
BALL_READY_KEY = "sai2::sim::ball_ready"
SHOOTER_READY_KEY = "sai2::sim::shooter_ready"
PREDICTION_READY_KEY = "sai2::sim::prediction_ready"
FALLING_KEY = "sai2::sim::ball_falling"
SHOOTER_MOVE = "sai2::shooter::joint_0_increment"
BALL_SHOOT_READY_KEY = "sai2::sim::basketball_shoot_ready"
SHOOTER_SET_STATE = "sai2::shooter::set_state"
RUN_KEY = "sim::run"

# Other keys
GRAVITY_KEY = "sai2::sim::world_gravity"
EE_FORCE_KEY = "cs225a::sensor::force"
EE_MOMENT_KEY = "cs225a::sensor::moment"
PRESS_START_KEY = "sim::start"
CONTINUE_KEY = "sim::continue"
SCORE = "sim::game::score"
CHECK_SPOT = "sim::game::check_spot_with_target"





