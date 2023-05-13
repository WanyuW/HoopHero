import redis
import numpy as np
import time

# importing libraries for interface
from tkinter import *
import customtkinter

# const
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

def button_function():
    print("button pressed")

def main():
    r = redis.Redis()
    r.flushall()

    r.set(HOOP_EE_POS, "0.0, 0.0, 0.0")
    r.set(HOOP_EE_VEL, "0.0, 0.0, 0.0")
    r.set(SHOOTER_POWER, 3)

    GAME_STATE = True

    # interface template
    customtkinter.set_appearance_mode("System")  # Modes: system (default), light, dark
    customtkinter.set_default_color_theme("blue")  # Themes: blue (default), dark-blue, green

    app = customtkinter.CTk()  # create CTk window like you do with the Tk window
    app.geometry("400x240")

    # Use CTkButton instead of tkinter Button
    button = customtkinter.CTkButton(master=app, text="CTkButton", command=button_function)
    button.place(relx=0.5, rely=0.5, anchor=tkinter.CENTER)

    app.mainloop()

    while (GAME_STATE):
        ee_pos = r.get(HOOP_EE_POS)
        print(ee_pos)

if __name__ == "__main__":
    main()
