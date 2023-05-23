import random

import redis
import time
import subprocess
import math
import os
import signal

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
    GAME_STATE: "",

    # hoop's info
    HOOP_EE_POS: "",
    HOOP_EE_VEL: "",
    HOOP_STATE_KEY: "",

    # shooter's info
    SHOOTER_POWER: "",
    SHOOTER_MODE: "",
    SHOOTING_ANGLE: "",

    # ball's info
    BALL_POS: "",
    BALL_VEL: ""
}

# global variable
# Define the button as a global variable
app = None
power = None
r = redis.Redis()
r.flushall()
start_time = None
end_time = None
time_flag = 0
mode_var = None
switch1 = None
switch2 = None
switch3 = None
start_frame = None
main_frame = None
wind_canvas = None
wind_line = None
wind_text = None
angle_canvas = None
angle_line = None
angle_text = None
shooting_angle_input = ""


def button_function():
    global app
    global start_frame
    global main_frame
    start_frame.place_forget()  # remove login frame
    print("pressed")
    main_frame.place(relx=0.5, rely=0.5, anchor=tk.CENTER)  # show main frame
    print(r.get(GAME_STATE).decode())

    # simviz_process = subprocess.Popen(["./simviz"])
    # simviz_pid = simviz_process.pid
    # # print("Simulation launched with PID:", simviz_pid)
    #
    # # Function to handle Ctrl+C interruption
    # def ctrl_c(signal, frame):
    #     simviz_process.terminate()
    #     simviz_process.wait()
    #     print("Simulation terminated")
    #     exit(0)
    #
    # # Set the signal handler
    # signal.signal(signal.SIGINT, ctrl_c)


def check_redis_keys(keys, app):
    # Retrieve the updated Redis keys using appropriate Redis commands
    global wind_canvas, wind_line, wind_text
    global angle_canvas, angle_line, angle_text

    for key, value in keys.items():
        if r.exists(key):
            value = r.get(key)
            if value is not None:
                keys[key] = value.decode()

    # Process the retrieved keys and update your application state
    shooting_angle = int(r.get(SHOOTING_ANGLE))  # takes in degrees
    shooting_radian = math.radians(shooting_angle)
    arrow_length = 75
    angle_canvas.delete(angle_line)
    angle_line = angle_canvas.create_line(176, 126, 176 - arrow_length * math.sin(shooting_radian),
                                          126 - arrow_length * math.cos(shooting_radian), fill="#990000",
                                          width=10, arrow="last", arrowshape=(15, 15, 5))
    angle_canvas.delete(angle_text)
    angle_text = angle_canvas.create_text(176, 260, anchor=tk.CENTER, text="Shooting angle = " + str(shooting_angle)
                                                                           + u"\u00b0",
                                          font=('Berlin Sans FB Demi', 35), fill="gray84")
    angle_canvas.update()

    # Schedule the next Redis key retrieval after a certain interval
    app.after(10, check_redis_keys, keys, app)  # Adjust the interval as needed


def on_keydown(event):
    global start_time
    global time_flag
    global wind_canvas, wind_line, wind_text
    if event.char == ' ':
        start_time = time.time()
        time_flag = 1
        # print(start_time)

    # random wind
    if event.char == 'r':
        wind_scale = random.random() * 5
        wind_angle = random.random() * math.pi * 2
        arrow_length = 75
        wind_canvas.delete(wind_line)
        wind_line = wind_canvas.create_line(176, 126, 176 - arrow_length * wind_scale / 5 * math.sin(wind_angle),
                                            126 - arrow_length * wind_scale / 5 * math.cos(wind_angle), fill="#990000",
                                            width=10, arrow="last", arrowshape=(15, 15, 5))
        wind_canvas.delete(wind_text)
        wind_text = wind_canvas.create_text(176, 260, anchor=tk.CENTER, text="Wind scale = {:.2f}".format(wind_scale),
                                            font=('Berlin Sans FB Demi', 35), fill="gray84")
        wind_canvas.update()


def on_keyup(event):
    global end_time
    global start_time
    global time_flag
    if event.char == 'a':
        if start_time is not None:
            end_time = time.time()
            duration = end_time - start_time
            # print(start_time, duration, end_time)
            power_progress = duration / 10
            power.set(power_progress)
            r.set(SHOOTER_POWER, str(power_progress))
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
    mode_var = ctk.StringVar(value="0")
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
    mode_var = ctk.StringVar(value="1")
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
    mode_var = ctk.StringVar(value="2")
    r.set(SHOOTER_MODE, "high_arc")
    # print("switch toggled, current value:", mode_var.get(), r.get(SHOOTER_MODE).decode())


def launch_function():
    r.set(GAME_STATE, "1")
    print(r.get(GAME_STATE).decode())
    subprocess.Popen(["./controller"])  # Launch the controller process in the background

    # Save the controller process ID to a file
    with open("controller_pid.txt", "w") as pid_file:
        pid_file.write(str(os.getpid()))


def mouse_click(event):
    global shooting_angle_input
    # Check if the mouse click event occurred within the desired area on the canvas
    if (event.x - 175) ** 2 + (event.y - 125) ** 2 <= 200 * 200:
        # Create a new window
        window = ctk.CTkToplevel(app)
        window.geometry("250x150")
        window.title("Shooting angle editor")

        # Create an entrybox in the new window
        entrybox = ctk.CTkEntry(window, width=200, height=100, placeholder_text=KEYS[SHOOTING_ANGLE] + u"\u00b0",
                                font=('Berlin Sans FB Demi', 40))
        entrybox.place(relx=0.5, rely=0.5, anchor=tk.CENTER)

        def handle_enter_key(event):
            value = entrybox.get()  # Get the entered value from the textbox
            r.set(SHOOTING_ANGLE, str(value))
            print("Entered value:", r.get(SHOOTING_ANGLE))  # Print the entered value

            window.destroy()  # Close the window

            global angle_canvas, angle_line, angle_text

            # Process the retrieved keys and update your application state
            shooting_angle = int(r.get(SHOOTING_ANGLE))  # takes in degrees
            shooting_radian = math.radians(shooting_angle)
            arrow_length = 75
            angle_canvas.delete(angle_line)
            angle_line = angle_canvas.create_line(176, 126, 176 - arrow_length * math.sin(shooting_radian),
                                                  126 - arrow_length * math.cos(shooting_radian), fill="#990000",
                                                  width=10, arrow="last", arrowshape=(15, 15, 5))
            angle_canvas.delete(angle_text)
            angle_text = angle_canvas.create_text(176, 260, anchor=tk.CENTER,
                                                  text="Shooting angle = " + str(shooting_angle)
                                                       + u"\u00b0",
                                                  font=('Berlin Sans FB Demi', 35), fill="gray84")
            angle_canvas.update()

        entrybox.bind("<Return>", handle_enter_key)  # Bind the Enter key event to handle_enter_key

        window.mainloop()


def main():
    # declaim the global var
    global app
    global power
    global mode_var
    global switch1
    global switch2
    global switch3
    global start_frame
    global main_frame
    global wind_canvas, wind_line, wind_text
    global angle_canvas, angle_line, angle_text

    r.set(SHOOTER_POWER, "0.5")
    r.set(GAME_STATE, "0")
    r.set(SHOOTER_MODE, "straight")
    r.set(SHOOTING_ANGLE, "0")

    # interface template
    ctk.set_appearance_mode("dark")  # Modes: system (default), light, dark
    ctk.set_default_color_theme("customized_theme.json")
    # ctk.set_default_color_theme("dark-blue")  # Themes: blue (default), dark-blue, green

    app = ctk.CTk()  # create CTk window like you do with the Tk window
    app.geometry("1440x750")
    app.title("HoopHero")

    # create main frame
    main_frame = ctk.CTkFrame(app, corner_radius=0)
    # main_frame.place(relx=0.5, rely=0.5, anchor=tk.CENTER)

    # set the background image
    app.bg_image = ctk.CTkImage(Image.open("basketball_court.jpg"), size=(1440, 750))
    app.bg_image_label = ctk.CTkLabel(main_frame, image=app.bg_image)
    app.bg_image_label.grid(row=0, column=0)

    # power module
    power_frame = ctk.CTkFrame(master=main_frame, width=1200, height=120)
    # power_frame = ctk.CTkFrame(master=app, width=1200, height=100)
    power_frame.place(relx=0.5, rely=0.1, anchor=tk.N)
    power_label = ctk.CTkLabel(master=power_frame, font=('Berlin Sans FB Demi', 40), text="Power")
    power_label.place(relx=0.05, rely=0.5, anchor=tk.W)
    power = ctk.CTkProgressBar(master=power_frame, orientation="horizontal", mode="determinate", width=900, height=50,
                               border_color='black', border_width=2, progress_color="#1f538d")
    power.set(float(r.get(SHOOTER_POWER)))
    power.place(relx=0.2, rely=0.5, anchor=tk.W)

    # mode module
    mode_frame = ctk.CTkFrame(master=main_frame, width=1200, height=120)
    # mode_frame = ctk.CTkFrame(master=app, width=1200, height=100)
    mode_frame.place(relx=0.5, rely=0.25, anchor=tk.N)
    mode_label = ctk.CTkLabel(master=mode_frame, font=('Berlin Sans FB Demi', 40), text="Mode")
    mode_label.place(relx=0.05, rely=0.35, anchor=tk.W)
    mode_var = ctk.StringVar(value="mode")
    switch1 = ctk.CTkSwitch(mode_frame, text=" Straight", font=('Helvetica', 28), command=switch_event1,
                            variable=mode_var, onvalue="straight", switch_width=70, switch_height=30)
    switch1.place(relx=0.2, rely=0.35, anchor=tk.W)
    switch2 = ctk.CTkSwitch(mode_frame, text=" Low arc", font=('Helvetica', 28), command=switch_event2,
                            variable=mode_var, onvalue="low_arc", switch_width=70, switch_height=30)
    switch2.place(relx=0.5, rely=0.35, anchor=tk.W)
    switch3 = ctk.CTkSwitch(mode_frame, text=" High arc", font=('Helvetica', 28), command=switch_event3,
                            variable=mode_var, onvalue="high_arc", switch_width=70, switch_height=30)
    switch3.place(relx=0.8, rely=0.35, anchor=tk.W)
    switch1.select()
    # r.set(SHOOTER_MODE, "straight")

    # Preview module
    preview_frame = ctk.CTkFrame(master=main_frame, width=1200, height=400)
    preview_frame.place(relx=0.5, rely=0.4, anchor=tk.N)
    preview_label = ctk.CTkLabel(master=preview_frame, font=('Berlin Sans FB Demi', 40), text="Shooting Preview")
    preview_label.place(relx=0.05, rely=0.05, anchor=tk.W)
    # use canvas
    # wind module
    wind_canvas = tk.Canvas(app, width=350, height=300, background="gray16", highlightthickness=0)
    wind_canvas.place(relx=0.28, rely=0.7, anchor=tk.CENTER)
    wind_image = Image.open("wind.png")
    wind_image = wind_image.resize((200, 200), Image.LANCZOS)  # Resize the image to fit the window
    wind_photo = ImageTk.PhotoImage(wind_image)  # Create a Tkinter-compatible photo image from the PIL image
    wind_canvas.create_image(175, 125, anchor=tk.CENTER, image=wind_photo)
    wind_canvas.create_oval(165, 115, 185, 135, fill="#990000", outline="#990000")
    wind_line = wind_canvas.create_line(176, 126, 176, 50, fill="#990000", width=10, arrow="last",
                                        arrowshape=(15, 15, 5))
    wind_text = wind_canvas.create_text(176, 260, anchor=tk.CENTER, text="Wind scale: 0",
                                        font=('Berlin Sans FB Demi', 35),
                                        fill="gray84")

    # angle module
    angle_canvas = tk.Canvas(app, width=350, height=300, background="gray16", highlightthickness=0)
    angle_canvas.place(relx=0.72, rely=0.7, anchor=tk.CENTER)
    angle_image = Image.open("angle.png")
    angle_image = angle_image.resize((200, 200), Image.LANCZOS)  # Resize the image to fit the window
    angle_photo = ImageTk.PhotoImage(angle_image)  # Create a Tkinter-compatible photo image from the PIL image
    angle_canvas.create_image(175, 125, anchor=tk.CENTER, image=angle_photo)
    angle_canvas.create_oval(165, 115, 185, 135, fill="#990000", outline="#990000")
    shooting_angle = int(r.get(SHOOTING_ANGLE))  # takes in degrees
    shooting_radian = math.radians(shooting_angle)
    arrow_length = 75
    angle_line = angle_canvas.create_line(176, 126, 176 - arrow_length * math.sin(shooting_radian),
                                          126 - arrow_length * math.cos(shooting_radian), fill="#990000",
                                          width=10, arrow="last", arrowshape=(15, 15, 5))
    angle_text = angle_canvas.create_text(176, 260, anchor=tk.CENTER, text="Shooting angle = " + str(shooting_angle)
                                                                           + u"\u00b0",
                                          font=('Berlin Sans FB Demi', 35), fill="gray84")

    # launch controller
    launch_button_title = "Launch"
    launch_button = ctk.CTkButton(master=preview_frame, text=launch_button_title, command=launch_function,
                                  font=('Berlin Sans FB Demi', 50), border_spacing=10, fg_color="#414141",
                                  hover_color="#2f2f2f")
    launch_button.place(relx=0.5, rely=0.5, anchor=tk.CENTER)

    # create login frame
    start_frame = ctk.CTkFrame(app, corner_radius=0, width=1440, height=750, fg_color="#0a053f")
    start_frame.place(relx=0.5, rely=0.5, anchor=tk.CENTER)
    game_title = ctk.CTkLabel(start_frame, text="HOOPHERO", font=('Blox (BRK)', 275), text_color="#aba7e8")
    game_title.place(relx=0.5, rely=0.4, anchor=tk.CENTER)
    button_title = "PRESS START"
    button = ctk.CTkButton(master=start_frame, text=button_title, command=button_function,
                           font=('Berlin Sans FB Demi', 80), border_spacing=20, text_color="#8885b7",
                           bg_color="#0a053f", hover_color="#0a053f", fg_color="#0a053f")
    button.place(relx=0.5, rely=0.75, anchor=tk.CENTER)

    # Start the Redis key retrieval loop
    check_redis_keys(KEYS, app)

    app.bind('<KeyPress>', on_keydown)
    app.bind('<KeyRelease>', on_keyup)

    # Bind the mouse click event to handle_mouse_click
    angle_canvas.bind("<Button-1>", mouse_click)

    app.mainloop()


if __name__ == "__main__":
    main()
