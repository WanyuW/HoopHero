import redis
import numpy as np
import time

# importing libraries for interface
import tkinter as tk
import customtkinter as ctk

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
    SHOOTER_POWER: "",

    # ball's info
    BALL_POS: "",
    BALL_VEL: ""
}

# global variable
# Define the button as a global variable
button = None

def button_function():
    print(KEYS[BALL_POS])
    # Access the button object using the global keyword
    global button

    # Modify the button's attributes
    button.configure(text=KEYS[HOOP_EE_POS])
    button.update()

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
    # declaim the global var
    global button

    r = redis.Redis()

    # r.set(HOOP_EE_POS, "[0.0, 0.0, 0.0]")
    # r.set(HOOP_EE_VEL, "[0.0, 0.0, 0.0]")
    r.set("SHOOTER_POWER", 3)

    # GAME_STATE = True

    # interface template
    ctk.set_appearance_mode("System")  # Modes: system (default), light, dark
    ctk.set_default_color_theme("customized_theme.json")
    # ctk.set_default_color_theme("dark-blue")  # Themes: blue (default), dark-blue, green

    app = ctk.CTk()  # create CTk window like you do with the Tk window
    app.geometry("1440x900")
    app.title("HoopHero")

    # Start the Redis key retrieval loop
    check_redis_keys(KEYS, r, app)
    # passes = {}   # passed values

    # Use CTkButton instead of tkinter Button
    button_title = KEYS[JOINT_ANGLES_KEY]
    button = ctk.CTkButton(master=app, text=button_title, command=button_function)
    button.place(relx=0.5, rely=0.5, anchor=tk.CENTER)

    app.mainloop()

if __name__ == "__main__":
    main()
