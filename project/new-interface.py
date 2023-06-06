import redis
import time
import subprocess
import math
import signal
import pygame.mixer
from pygame.mixer import Sound


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
    GRAVITY_KEY: "",

    # hoop's info
    HOOP_EE_POS: "",
    HOOP_EE_VEL: "",
    HOOP_STATE_KEY: "",

    # shooter's info
    SHOOTER_POWER: "",
    SHOOTER_MODE: "",
    SHOOTING_ANGLE: "",
    SHOOTER_EE_POS_INWORLD: "",

    # ball's info
    BALL_POS: "",
    BALL_VEL: "",

    # launch
    GAME_STATE: "",
    RESET_KEY: "",
    RUN_KEY: ""
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
intro_frame = None
angle_canvas = None
angle_line = None
angle_text = None
angle_label = None
shooting_angle_input = ""
launch_button = None
power_counter = 0
joystick_process = None

def run():
    global joystick_process
    simviz_process = subprocess.Popen(["./simviz"])
    # simviz_pid = simviz_process.pid

    time.sleep(4)

    r.set(GAME_STATE, "1")
    # print(r.get(GAME_STATE).decode())
    launch_process = subprocess.Popen(["./controller"])  # Launch the controller process in the background

    # Function to handle Ctrl+C interruption
    def ctrl_c(signal, frame):
        simviz_process.terminate()
        simviz_process.wait()
        print("Simulation terminated")
        launch_process.terminate()
        launch_process.wait()
        print("Controller terminated")
        joystick_process.terminate()
        joystick_process.wait()
        exit(0)

    # Set the signal handler
    signal.signal(signal.SIGINT, ctrl_c)

def play_sound(sound_file):
    sound = Sound(sound_file)
    sound.play()

def button_function():
    global app
    global intro_frame
    global main_frame
    intro_frame.place_forget()  # remove login frame
    print("pressed")
    play_sound("button-3.wav")
    main_frame.place(relx=0.5, rely=0.5, anchor=tk.CENTER)  # show main frame

    r.set(RUN_KEY, "1")
    r.set(CONTINUE_KEY, "1")

def intro_button_function():
    global app
    global start_frame
    global intro_frame
    start_frame.place_forget()  # remove login frame
    intro_frame.place(relx=0.5, rely=0.5, anchor=tk.CENTER)  # show main frame
    r.set(PRESS_START_KEY, "1")
    play_sound("button-3.wav")


def check_redis_keys(keys, app):
    # Retrieve the updated Redis keys using appropriate Redis commands
    global angle_canvas, angle_line, angle_text, angle_label
    global power
    global switch1
    global switch2
    global switch3

    for key, value in keys.items():
        if r.exists(key):
            value = r.get(key)
            if value is not None:
                keys[key] = value.decode()

    # Process the retrieved keys and update your application state
    shooting_angle = int(float(r.get(SHOOTING_ANGLE)))  # takes in degrees
    shooting_radian = math.radians(shooting_angle)
    arrow_length = 45
    angle_canvas.delete(angle_line)
    angle_line = angle_canvas.create_line(75, 75, 75 - arrow_length * math.sin(shooting_radian),
                                          75 - arrow_length * math.cos(shooting_radian), fill="#990000",
                                          width=7, arrow="last", arrowshape=(15, 15, 5))
    angle_text = str(shooting_angle) + u"\u00b0"
    angle_label.configure(text=angle_text)
    angle_canvas.update()
    angle_label.update()

    if KEYS[RUN_KEY] == "1":
        print(r.get(RUN_KEY))
        run()
        r.set(RUN_KEY, "0")

    power_progress = r.get(SHOOTER_POWER).decode()
    power.set(float(power_progress))
    power.update()

    shooter_mode = r.get(SHOOTER_MODE).decode()
    if shooter_mode == 'low_arc':
        switch_event2()
    elif shooter_mode == 'high_arc':
        switch_event3()
    else:
        switch_event1()

    # Schedule the next Redis key retrieval after a certain interval
    app.after(1, check_redis_keys, keys, app)  # Adjust the interval as needed


def on_keydown(event):
    if event.char == ' ':
        if r.get(PRESS_START_KEY).decode() == '0':
            intro_button_function()
        elif r.get(CONTINUE_KEY).decode() == '0':
            button_function()


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


def launch_function():
    r.set(RESET_KEY, "1")


def main():
    pygame.mixer.init()
    pygame.mixer.music.load("bgm.mp3")
    pygame.mixer.music.play(-1)
    # declaim the global var
    global app
    global power
    global mode_var
    global switch1
    global switch2
    global switch3
    global start_frame
    global main_frame
    global angle_canvas, angle_line, angle_text, angle_label
    global intro_frame
    global joystick_process

    # init keys
    r.set(SHOOTER_POWER, "0.5")
    r.set(GAME_STATE, "0")
    r.set(SHOOTER_MODE, "straight")
    r.set(SHOOTING_ANGLE, "0")
    r.set(GRAVITY_KEY, str([0.0, 0.0, -9.81]))
    r.set(PRESS_START_KEY, "0")
    r.set(CONTINUE_KEY, "0")

    # Launch the joystick_controller.py script
    joystick_process = subprocess.Popen(["python3", "joystick_controller.py"])

    # interface template
    ctk.set_appearance_mode("dark")  # Modes: system (default), light, dark
    ctk.set_default_color_theme("customized_theme.json")
    ctk.set_widget_scaling(0.5)  # widget dimensions and text size
    ctk.set_window_scaling(0.5)  # window geometry dimensions

    app = ctk.CTk()

    app.geometry("1980x1020+0+0")
    app.title("HoopHero")

    # create main frame
    main_frame = ctk.CTkFrame(app, corner_radius=0)

    # set the background image
    app.bg_image = ctk.CTkImage(Image.open("11.png"), size=(1980, 1020))
    app.bg_image_label = ctk.CTkLabel(main_frame, image=app.bg_image, text="")
    app.bg_image_label.grid(row=0, column=0)

    # power module
    power = ctk.CTkProgressBar(master=main_frame, orientation="horizontal", mode="determinate", width=900, height=80,
                               border_color='#3a301e', border_width=5, progress_color="#e24795", bg_color="#2e1b5b", fg_color="#837189")
    power.set(float(r.get(SHOOTER_POWER)))
    power.place(relx=0.33, rely=0.275, anchor=tk.W)

    # mode module
    mode_var = ctk.StringVar(value="mode")
    switch1 = ctk.CTkSwitch(main_frame, command=switch_event1, bg_color="#2e1b5b", fg_color="#837189", text="", progress_color="#e24795",
                            variable=mode_var, onvalue="straight", switch_width=140, switch_height=55, border_width=5, border_color='#3a301e')
    switch1.place(relx=0.4, rely=0.375, anchor=tk.W)
    switch2 = ctk.CTkSwitch(main_frame, command=switch_event2, bg_color="#2e1b5b", fg_color="#837189", text="", progress_color="#e24795",
                            variable=mode_var, onvalue="low_arc", switch_width=140, switch_height=55, border_width=5, border_color='#3a301e')
    switch2.place(relx=0.555, rely=0.375, anchor=tk.W)
    switch3 = ctk.CTkSwitch(main_frame, command=switch_event3, bg_color="#2e1b5b", fg_color="#837189", text="", progress_color="#e24795",
                            variable=mode_var, onvalue="high_arc", switch_width=140, switch_height=55, border_width=5, border_color='#3a301e')
    switch3.place(relx=0.71, rely=0.375, anchor=tk.W)
    switch1.select()

    # angle module
    angle_canvas = ctk.CTkCanvas(main_frame, width=150, height=150, background="#2e1b5b", highlightthickness=0)
    angle_canvas.place(relx=0.5, rely=0.65, anchor=tk.CENTER)
    angle_image = Image.open("wind.png")
    angle_image = angle_image.resize((125, 125), Image.LANCZOS)  # Resize the image to fit the window
    angle_photo = ImageTk.PhotoImage(angle_image)  # Create a Tkinter-compatible photo image from the PIL image
    angle_canvas.create_image(75, 75, anchor=tk.CENTER, image=angle_photo)
    angle_canvas.create_oval(68, 68, 82, 82, fill="#990000", outline="#990000")
    shooting_angle = int(r.get(SHOOTING_ANGLE))  # takes in degrees
    shooting_radian = math.radians(shooting_angle)
    arrow_length = 45
    angle_line = angle_canvas.create_line(75, 75, 75 - arrow_length * math.sin(shooting_radian),
                                          75 - arrow_length * math.cos(shooting_radian), fill="#990000",
                                          width=7, arrow="last", arrowshape=(15, 15, 5))
    angle_text = str(shooting_angle) + u"\u00b0"
    angle_label = ctk.CTkLabel(main_frame, text=angle_text, font=('Berlin Sans FB Demi', 60), bg_color="#2e1b5b", text_color="white")
    angle_label.place(relx=0.25, rely=0.7, anchor=tk.CENTER)

    # create start frame
    start_frame = ctk.CTkFrame(app, corner_radius=0)
    start_frame.place(relx=0.5, rely=0.5, anchor=tk.CENTER)

    # set the background image
    app.bg_image = ctk.CTkImage(Image.open("1.png"), size=(1980, 1020))
    app.bg_image_label = ctk.CTkLabel(start_frame, text=" ", image=app.bg_image)
    app.bg_image_label.grid(row=0, column=0)

    intro_button_title = "PRESS START"
    intro_button = ctk.CTkButton(master=start_frame, text=intro_button_title, command=intro_button_function,
                           font=('Arcadegamerfont', 60), border_spacing=2, text_color="#0a053f",
                           bg_color="#e24795", hover_color="#e24795", fg_color="#e24795")
    intro_button.place(relx=0.5, rely=0.655, anchor=tk.CENTER)

    # create introduction frame
    intro_frame = ctk.CTkFrame(app, corner_radius=0)

    # set the intro image
    app.intro_image = ctk.CTkImage(Image.open("2.png"), size=(1980, 1020))
    app.intro_image_label = ctk.CTkLabel(intro_frame, text=" ", image=app.intro_image)
    app.intro_image_label.grid(row=0, column=0)

    button_title = " "
    button = ctk.CTkButton(master=intro_frame, text=button_title, command=button_function,
                           font=('Arcadegamerfont', 10), border_spacing=2, text_color="#db00d5", height=100,
                           bg_color="#db00d5", hover_color="#db00d5", fg_color="#db00d5", width=70)
    button.place(relx=0.92, rely=0.28, anchor=tk.CENTER)

    # Start the Redis key retrieval loop
    check_redis_keys(KEYS, app)

    app.bind('<KeyPress>', on_keydown)
    app.mainloop()


if __name__ == "__main__":
    main()
