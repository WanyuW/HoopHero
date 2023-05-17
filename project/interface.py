import random

import redis
import numpy as np
import time

# importing libraries for interface
import tkinter as tk
import customtkinter as ctk
from PIL import Image, ImageTk

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
    HOOP_STATE_KEY: "",

    # shooter's info
    SHOOTER_POWER: "",
    SHOOTER_MODE: "",

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
mode_var = None
switch1 = None
switch2 = None
switch3 = None


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


def switch_event1():
    global mode_var
    global switch1
    global switch2
    global switch3
    switch1.select()
    switch2.deselect()
    switch3.deselect()
    mode_var = ctk.StringVar(value="straight")
    r.set(SHOOTER_MODE, "straight")
    # print("switch toggled, current value:", mode_var.get(), r.get(SHOOTER_MODE).decode())


def switch_event2():
    global mode_var
    global switch1
    global switch2
    global switch3
    switch1.deselect()
    switch2.select()
    switch3.deselect()
    mode_var = ctk.StringVar(value="low_arc")
    r.set(SHOOTER_MODE, "low_arc")
    # print("switch toggled, current value:", mode_var.get(), r.get(SHOOTER_MODE).decode())


def switch_event3():
    global mode_var
    global switch1
    global switch2
    global switch3
    switch1.deselect()
    switch2.deselect()
    switch3.select()
    mode_var = ctk.StringVar(value="high_arc")
    r.set(SHOOTER_MODE, "high_arc")
    # print("switch toggled, current value:", mode_var.get(), r.get(SHOOTER_MODE).decode())


def main():
    # declaim the global var
    global button
    global power
    global mode_var
    global switch1
    global switch2
    global switch3

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

    # # Load the background image
    # image = Image.open("basketball_court.jpeg")
    # image = image.resize((1440, 900), Image.LANCZOS)  # Resize the image to fit the window
    # photo = ImageTk.PhotoImage(image)  # Create a Tkinter-compatible photo image from the PIL image
    # # Create a Canvas widget and place it in the window
    # canvas = tk.Canvas(app, width=1440, height=900)
    # canvas.pack()
    # # Draw the image on the canvas
    # canvas.create_image(0, 0, anchor=tk.NW, image=photo)

    # # set the background image
    # app.bg_image = ctk.CTkImage(Image.open("basketball_court.jpeg"), size=(1440, 900))
    # app.bg_image_label = ctk.CTkLabel(app, image=app.bg_image)
    # app.bg_image_label.grid(row=0, column=0)

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
    power_frame = ctk.CTkFrame(master=app, width=1200, height=100)
    power_frame.place(relx=0.5, rely=0.1, anchor=tk.N)
    power_label = ctk.CTkLabel(master=power_frame, font=('Berlin Sans FB Demi', 40), text="Power")
    power_label.place(relx=0.05, rely=0.5, anchor=tk.W)
    power = ctk.CTkProgressBar(master=power_frame, orientation="horizontal", mode="determinate", width=900, height=50,
                               border_color='black', border_width=2, progress_color="#1f538d")
    power.set(float(KEYS[SHOOTER_POWER]))
    power.place(relx=0.2, rely=0.5, anchor=tk.W)

    # mode module
    mode_frame = ctk.CTkFrame(master=app, width=1200, height=100)
    mode_frame.place(relx=0.5, rely=0.25, anchor=tk.N)
    mode_label = ctk.CTkLabel(master=mode_frame, font=('Berlin Sans FB Demi', 40), text="Mode")
    mode_label.place(relx=0.05, rely=0.5, anchor=tk.W)
    mode_var = ctk.StringVar(value="mode")
    switch1 = ctk.CTkSwitch(mode_frame, text=" Straight", font=('Helvetica', 30), command=switch_event1,
                            variable=mode_var, onvalue="straight", switch_width=70, switch_height=30)
    switch1.place(relx=0.2, rely=0.5, anchor=tk.W)
    switch2 = ctk.CTkSwitch(mode_frame, text=" Low arc", font=('Helvetica', 30), command=switch_event2,
                            variable=mode_var, onvalue="low_arc", switch_width=70, switch_height=30)
    switch2.place(relx=0.5, rely=0.5, anchor=tk.W)
    switch3 = ctk.CTkSwitch(mode_frame, text=" High arc", font=('Helvetica', 30), command=switch_event3,
                            variable=mode_var, onvalue="high_arc", switch_width=70, switch_height=30)
    switch3.place(relx=0.8, rely=0.5, anchor=tk.W)
    switch1.select()

    # Preview module
    preview_frame = ctk.CTkFrame(master=app, width=1200, height=450)
    preview_frame.place(relx=0.5, rely=0.4, anchor=tk.N)
    preview_label = ctk.CTkLabel(master=preview_frame, font=('Berlin Sans FB Demi', 40), text="Shooting Preview")
    preview_label.place(relx=0.05, rely=0.11, anchor=tk.W)


    app.bind('<KeyPress>', on_keydown)
    app.bind('<KeyRelease>', on_keyup)

    app.mainloop()

if __name__ == "__main__":
    main()
