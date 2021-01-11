# Rover Project

Repository for AEMBOT's 2020 rover written using ROS and Python

# Installing ROS

**I would highly recommend using either a machine dual booted with Ubuntu or a VM (If dual booting make sure the ping returns a reasonably low ping ~15ms or lower)**

This project makes use of [ROS Noetic](http://wiki.ros.org/noetic/Installation) which can be found on the ROS website or at that link 
Follow the install instructions for Ubuntu.

After the install you need to add a few lines to your .bashrc file (This allows certain commands to run when a new terminal is opened)
1. After running a catkin_make to build the project, you will need to add the `setup.bash` file for the RoverProject to the `.bashrc`.
To do this you simply need to open the .bashrc file and add the line `source <RoverProject Parent Directory>/RoverProject/devel/setup.bash`
2. Next you need to add the `src` folder from within each of the rover packages to the PYTHONPATH.
To do this you need to add the following line **ABOVE** the setup.bash calls:<br>
``
export PYTHONPATH="<RoverProject Parent Directory>/RoverProject/src/rover_main/src":<RoverProject Parent Directory>/RoverProject/src/rover_client/src"
``

You will also need to add the rover's hostname/ip combo to your hosts file so that you can accsess the roscore running on the rover

# Tutorials

While the concept of how ROS works isn't that hard to understand and begin to use. The management of nodes, messages, services, etc. can be a bit of a pain. 

Therefore, 
**If you don't know what you are doing with the CMake or Package files DO NOT TOUCH.**
One can break these files very easily, and since they are used to build the project... It won't build.

That being said the [ROS Wiki](http://wiki.ros.org) and it's [Tutorial Series (Mainly just the Publisher/Subscriber ones and the explanations of the ROS paradigms)](http://wiki.ros.org/ROS/Tutorials) are great places to start. If you run into issues (you will)... **google it**. Someone has likely had a similar issue on the [ROS Forms](https://answers.ros.org/questions/). Please try to troubleshoot issues yourself before asking. 

# Useful Resources

[ROS Project Structuring](http://www.artificialhumancompanions.com/structure-python-based-ros-package/)
[ROS Wiki](http://wiki.ros.org)
[ROS Forms](https://answers.ros.org/questions/)

# Workspace Setup Suggestions

1. I would recommend setting up several virtual desktops to swap between to allow for more workspace
