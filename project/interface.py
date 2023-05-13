import redis
import numpy as np
import time

if __name__ == "__main__":
    r = redis.Redis()
    r.flushall()
    
    # template
    JOINT_ANGLES_KEY = "sai2::sim::panda::sensors::q"
    JOINT_VELOCITIES_KEY = "sai2::sim::panda::sensors::dq"
    JOINT_TORQUES_COMMANDED_KEY = "sai2::sim::panda::actuators::fgc"
    CONTROLLER_RUNNING_KEY = "sai2::sim::panda::controller"
    
    # hoop's info
    HOOP_EE_POS = "sai2::hoop::ee_pos"
    HOOP_EE_VEL = "sai2::hoop::ee_vel"
    
    # shooter's info
    SHOOTER_POWER = "sai2::shooter::power"
    
    r.set(HOOP_EE_POS, "0.0, 0.0, 0.0")
    r.set(HOOP_EE_VEL, "0.0, 0.0, 0.0")
    r.set(SHOOTER_POWER, 3)
    
    GAME_STATE = True
    
    while(GAME_STATE):
        ee_pos = r.get(HOOP_EE_POS)
        print(ee_pos)
