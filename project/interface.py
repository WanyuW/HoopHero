import random

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
power = None
r = redis.Redis()
start_time = None
end_time = None
time_flag = 0
check_var = None


def button_function():
    global power
    power_progress = random.random()
    r.set(SHOOTER_POWER, power_progress)
    power.set(float(KEYS[SHOOTER_POWER]))
    power.update()
    print(KEYS[SHOOTER_POWER], power_progress)


def check_redis_keys(keys, app):
    # Retrieve the updated Redis keys using appropriate Redis commands
    for key, value in keys.items():
        value = r.get(key)
        if value is not None:
            keys[key] = value.decode()

    # Process the retrieved keys and update your application state
    #

    # Schedule the next Redis key retrieval after a certain interval
    app.after(10, check_redis_keys, keys, app)  # Adjust the interval as needed


def on_keydown(event):
    global start_time
    global time_flag
    if event.char == ' ':
        start_time = time.time()
        time_flag = 1
        # print(start_time)


def on_keyup(event):
    global end_time
    global start_time
    global time_flag
    if event.char == 'a':
        if start_time is not None:
            end_time = time.time()
            duration = end_time - start_time
            # print(start_time, duration, end_time)
            power_progress = duration/10
            power.set(power_progress)
            r.set(SHOOTER_POWER, power_progress)
            power.update()
            # print(power_progress, KEYS[SHOOTER_POWER], power.get())
            time_flag = 0
            start_time = None


def checkbox_event():
    global check_var
    print("checkbox toggled, current value:", check_var.get())


def main():
    # declaim the global var
    global button
    global power

    # r.set(HOOP_EE_POS, "[0.0, 0.0, 0.0]")
    # r.set(HOOP_EE_VEL, "[0.0, 0.0, 0.0]")
    r.set(SHOOTER_POWER, 0.5)

    # GAME_STATE = True

    # interface template
    ctk.set_appearance_mode("System")  # Modes: system (default), light, dark
    ctk.set_default_color_theme("customized_theme.json")
    # ctk.set_default_color_theme("dark-blue")  # Themes: blue (default), dark-blue, green

    app = ctk.CTk()  # create CTk window like you do with the Tk window
    app.geometry("1440x900")
    app.title("HoopHero")

    # Start the Redis key retrieval loop
    check_redis_keys(KEYS, app)

    # Use CTkButton instead of tkinter Button
    button_title = KEYS[JOINT_ANGLES_KEY]
    button = ctk.CTkButton(master=app, text=button_title, command=button_function)
    button.place(relx=0.5, rely=0.5, anchor=tk.CENTER)

    # head title
    title = ctk.CTkLabel(master=app, font=("Berlin Sans FB Demi", 30), text="Player Panel")
    title.place(relx=0.5, rely=0.05, anchor=tk.CENTER)

    # power module
    power_label = ctk.CTkLabel(master=app, font=('Berlin Sans FB Demi', 40), text="Power")
    power_label.place(relx=0.2, rely=0.15, anchor=tk.CENTER)
    power = ctk.CTkProgressBar(master=app, orientation="horizontal", mode="determinate", width=800, height=50,
                               border_color='black', border_width=2, progress_color="#1f538d")
    power.set(float(KEYS[SHOOTER_POWER]))
    power.place(relx=0.3, rely=0.15, anchor=tk.W)

    # mode module
    mode_label = ctk.CTkLabel(master=app, font=('Berlin Sans FB Demi', 40), text="Mode")
    mode_label.place(relx=0.2, rely=0.25, anchor=tk.CENTER)
    check_var = ctk.StringVar(value="on")
    checkbox1 = ctk.CTkCheckBox(app, text="Straignt", font=('Calibri', 30), command=checkbox_event,
                                         variable=check_var, onvalue="on", offvalue="off")
    checkbox1.place(relx=0.3, rely=0.25, anchor=tk.W)

    app.bind('<KeyPress>', on_keydown)
    app.bind('<KeyRelease>', on_keyup)

    app.mainloop()

if __name__ == "__main__":
    main()
