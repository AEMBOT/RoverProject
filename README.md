# Rover Project

Repository for AEMBOT's 2020 rover written using ROS and Python

# Installing ROS

**I would highly recomend using a machine dual booted with Ubuntu if you actually want to test any of your modules off rover**
There are experimental windows builds but your milage may vary

This project makes use of [ROS Noetic](http://wiki.ros.org/noetic/Installation) which can be found on the ROS website or at that link

# Tutorials

While the concept of how ROS works isn't that hard to understand and begin to use. The management of modes, messages, services, etc. can be a bit of a pain. 

Therefore, 
**If you don't know what you are doing with the CMake or Package files DO NOT TOUCH.**
One can break these files very easily, and since they are used to build the project... It won't build.

This project also utilizes Python 3 as Python 2 is dead, for the shebang lines to execute properly you need to chage the python3 executable in /usr/bin/ to python

That being said the [ROS Wiki](http://wiki.ros.org) and its [Tutorial Series](http://wiki.ros.org/ROS/Tutorials) are great places to start. If you run into issues (you will)... google it. Someone has likely had a similar issue on the [ROS Forms](https://answers.ros.org/questions/).

You can also always ask me (Will) for help.

# Useful Resources

[ROS Project Structuring](http://www.artificialhumancompanions.com/structure-python-based-ros-package/)
