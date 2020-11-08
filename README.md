# Rover Project

Repository for AEMBOT's 2020 rover written using ROS and Python

# Installing ROS

**I would highly recommend using a machine dual booted with Ubuntu. However with that said make sure that machine returns a reasonably low ping**
If it its average is > 300ms check in windows, if that is lower install for Windows

This project makes use of [ROS Noetic](http://wiki.ros.org/noetic/Installation) which can be found on the ROS website or at that link

After the install add the following to an environment variable named PYTHONPATH
1. C:\opt\workspaces\RoverProject\src\rover_client\src
2. C:\opt\workspaces\RoverProject\src\rover_main\src

You will also need to add the rover's hostname/ip combo to your hosts file

# Tutorials

While the concept of how ROS works isn't that hard to understand and begin to use. The management of modes, messages, services, etc. can be a bit of a pain. 

Therefore, 
**If you don't know what you are doing with the CMake or Package files DO NOT TOUCH.**
One can break these files very easily, and since they are used to build the project... It won't build.

That being said the [ROS Wiki](http://wiki.ros.org) and its [Tutorial Series](http://wiki.ros.org/ROS/Tutorials) are great places to start. If you run into issues (you will)... google it. Someone has likely had a similar issue on the [ROS Forms](https://answers.ros.org/questions/).

You can also always ask me (Will) for help.

# Useful Resources

[ROS Project Structuring](http://www.artificialhumancompanions.com/structure-python-based-ros-package/)

# Workspace Setup Suggestions

1. I would recommend setting up several virtual desktops to swap between to allow for more workspace
