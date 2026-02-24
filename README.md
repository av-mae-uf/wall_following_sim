# wall_following_sim
Code to begin the wall following problem in simulation.  The vehicle controller code is not included.

## Installation/Build
Create a development workspace by typing the following when in your home directory. Replace the text within the <> to be whatever you want.
```bash
mkdir -p <your_dev_ws>/src
```
Move into the **src** directory.
```bash
cd <your_dev_ws>/src
```

Download the repository using
```bash
git clone https://github.com/av-mae-uf/wall_following_sim.git
```
The gps_nav package uses the ackermann_msgs package.  Add the following:
```bash
sudo apt install ros-humble-ackermann-msgs
```
Change your directory to your root workspace directory (*~/<your_dev_ws>*) and build the workspace.
```bash
colcon build
``` 
Source your workspace with
```bash
source install/setup.bash
```
## How to run simulation
Note: If running on a Raspberry Pi 5 (Ubuntu 24.04) enter the following as an additional line to your .bashrc file:
```bash
export QT_QPA_PLATFORM="xcb"
```
This is required in order for the rviz2 simulator to be able to open a graphic window.
Source the .bashrc file.  Note: It will be sourced every time you open a new terminal window from here on.

Run the command below to see the simulation at work.
```bash
ros2 launch gps_nav simulation_demo.launch.py
``` 
