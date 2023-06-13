# HoopHero
CS225A project
## Denpendencies
in order to run the simulation please download the following:
1. font: blox-brk
2. run "pip3 install screeninfo" on the terminal
3. run "pip3 install pillow"
4. install pygame "pip3 intall pygame"
5. install Enjoyable to map controller button to keyboard via https://yukkurigames.com/enjoyable/

## Ps5 controller instruction:
1. direction button on the left map to W, A, S, D to zoom in, move left, zoom out, and move left.
2. press right joystick to map left mouse click
3. move the right joystick left and right to map UP and DOWN to move camera height
4. move the right joystick while pressing it can rotate the camera view.
5. move the left joystick left or right to controller the shooter shooting angle.
6. circle button map to launch.
7. triangle button map to straigh shooting gesture.
8. square button map to low_arc shooting gesture.
9. x button map to high_arc shooting gesture.
10. left menu button map to ESC to kill the simulation.

## Default settings for robot controller
1. shooting angle: 0 degree.
2. shooting power factor: 0
3. shooting gesture mode: straight

## ENJOY AND HAVE FUN!

In the main directory, create a build directory and build from that folder:
```
mkdir build
cd build
cmake .. && make -j4
```
Make sure you have connected with the controller. Then go to the bin folder, and run the following command:
```
python3 new-interface.py
```
