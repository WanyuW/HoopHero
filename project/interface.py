import redis
import numpy as np
import time

# importing libraries for interface
import tkinter
import customtkinter

# const
# keys dict
KEYS = {
    # redis_keys template
    "JOINT_ANGLES_KEY": {"sai2::sim::panda::sensors::q": None},
    "JOINT_VELOCITIES_KEY": {"sai2::sim::panda::sensors::dq": None},
    "JOINT_TORQUES_COMMANDED_KEY": {"sai2::sim::panda::actuators::fgc": None},
    "CONTROLLER_RUNNING_KEY": {"sai2::sim::panda::controller": None},

    # hoop's info
    "HOOP_EE_POS": {"sai2::hoop::ee_pos": None},
    "HOOP_EE_VEL": {"sai2::hoop::ee_vel": None},

    # shooter's info
    "SHOOTER_POWER": {"sai2::shooter::power": None}
}

def button_function():
    print("button pressed")

def check_redis_keys(keys, r, app):
    # Retrieve the updated Redis keys using appropriate Redis commands
    for key, name in keys:
        for r_key, value in name:
            value = r.get(r_key)

    # Process the retrieved keys and update your application state
    # ...

    # Schedule the next Redis key retrieval after a certain interval
    app.after(1000, check_redis_keys)  # Adjust the interval as needed

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
    button_title = str(KEYS["JOINT_ANGLES_KEY"]["sai2::sim::panda::sensors::q"])

    # Use CTkButton instead of tkinter Button
    button = customtkinter.CTkButton(master=app, text=button_title, command=button_function)
    button.place(relx=0.5, rely=0.5, anchor=tkinter.CENTER)

    app.mainloop()

if __name__ == "__main__":
    main()
