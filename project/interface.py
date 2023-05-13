import redis
import numpy as np
import time

# importing libraries for interface
import tkinter
import customtkinter

# importing keys
from keys import *

# const
# keys dict
KEYS = {
    # redis_keys template
    JOINT_ANGLES_KEY: "",
    JOINT_VELOCITIES_KEY: "",
    JOINT_TORQUES_COMMANDED_KEY: "",
    CONTROLLER_RUNNING_KEY: "",

    # hoop's info
    HOOP_EE_POS: "",
    HOOP_EE_VEL: "",

    # shooter's info
    SHOOTER_POWER: ""
}

def button_function():
    print(KEYS[JOINT_ANGLES_KEY])

def check_redis_keys(keys, r, app):
    # Retrieve the updated Redis keys using appropriate Redis commands
    for key, value in keys.items():
        value = r.get(key)
        if value is not None:
            keys[key] = value.decode()

    # Process the retrieved keys and update your application state
    #

    # Schedule the next Redis key retrieval after a certain interval
    app.after(10, check_redis_keys, keys, r, app)  # Adjust the interval as needed

def main():
    r = redis.Redis()

    # r.set(HOOP_EE_POS, "[0.0, 0.0, 0.0]")
    # r.set(HOOP_EE_VEL, "[0.0, 0.0, 0.0]")
    r.set("SHOOTER_POWER", 3)

    GAME_STATE = True

    # interface template
    customtkinter.set_appearance_mode("System")  # Modes: system (default), light, dark
    customtkinter.set_default_color_theme("blue")  # Themes: blue (default), dark-blue, green

    app = customtkinter.CTk()  # create CTk window like you do with the Tk window
    app.geometry("400x240")

    # Start the Redis key retrieval loop
    check_redis_keys(KEYS, r, app)
    # passes = {}   # passed values

    # Use CTkButton instead of tkinter Button
    button_title = "JOINT_ANGLES_KEY"
    button = customtkinter.CTkButton(master=app, text=button_title, command=button_function)
    button.place(relx=0.5, rely=0.5, anchor=tkinter.CENTER)

    app.mainloop()

if __name__ == "__main__":
    main()
