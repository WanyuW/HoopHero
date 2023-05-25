# overall redis keys
JOINT_ANGLES_KEY = "sai2::sim::panda::sensors::q"
JOINT_VELOCITIES_KEY = "sai2::sim::panda::sensors::dq"
JOINT_TORQUES_COMMANDED_KEY = "sai2::sim::panda::actuators::fgc"
CONTROLLER_RUNNING_KEY = "sai2::sim::panda::controller"
GRAVITY_KEY = "sai2::sim::world_gravity"


# hoop's info
HOOP_EE_POS = "sai2::hoop::ee_pos"
HOOP_EE_VEL = "sai2::hoop::ee_vel"
HOOP_STATE_KEY = "sai2::sim::hoop::state"

# shooter's info
SHOOTER_POWER = "sai2::shooter::power"
SHOOTER_MODE = "sai2::sim::shooter::mode"
SHOOTING_ANGLE = "sai2::shooter::shooting_angle"

# ball's info
BALL_POS = "sai2::sim::basketball_pos"
BALL_VEL = "sai2::sim::basketball_vel"
FUTURE_POS = "sai2::sim::basketball_future_pos"

# launch
GAME_STATE = "sai2::game_state"
RESET_KEY = "sai2::sim::reset"
