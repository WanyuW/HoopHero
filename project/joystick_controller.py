import pygame
import redis

# importing keys
from keys import *

# setup ps5 controller
pygame.init()
pygame.joystick.init()
r = redis.Redis()
r.set(SHOOTER_MOVE, "0")
r.set(BALL_SHOOT_READY_KEY, "0")
r.set(SHOOTER_SET_STATE, "1")

# Set up the gamepad
if pygame.joystick.get_count() > 0:
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
else:
    print("No controller connected.")
    # Handle the case when no controller is connected (e.g., show an error message)

def main():
    while True:
        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                # Handle axis motion
                shooter_move_dx = joystick.get_axis(0)
                r.set(SHOOTER_MOVE, str(shooter_move_dx))
                # print(shooter_move_dx)

                power_input = float(joystick.get_axis(5))
                power = 0.5 * power_input + 0.5
                r.set(SHOOTER_POWER, str(power))

            elif event.type == pygame.JOYBUTTONDOWN:
                # Handle button press
                if event.button == 0:
                    r.set(SHOOTER_MODE, "high_arc")
                    print("high_arc")
                elif event.button == 1:
                    if r.get(SHOOTER_SET_STATE).decode() == "1":
                        r.set(BALL_SHOOT_READY_KEY, "1")
                        print("ball's ready to be shot")
                elif event.button == 2:
                    r.set(SHOOTER_MODE, "low_arc")
                    print("low_arc")
                elif event.button == 3:
                    r.set(SHOOTER_MODE, "straight")
                    print("straight")
                elif event.button == 10:
                    r.set(RESET_KEY, "1")

    # Clean up resources and exit
    pygame.joystick.quit()
    pygame.quit()

if __name__ == "__main__":
    main()
