#include "ros/ros.h"

#include "rover_main/controllerMap.h"


/**
 *  Called whenever the controller topic stream is updated
 */
void controllerStreamCallback(const rover_main::controllerMap msg){
    ROS_INFO("Controller State: [%f]", (float)msg.Joystick_LeftX);
}

int main(int argc, char **argv){

    // Creates a new node to listen for new updates to the control stream
    ros::init(argc, argv, "controller_listener");

    // Used to handle all subscribers and publishers
    ros::NodeHandle n;

    ROS_INFO("Hello");

    ros::Subscriber sub = n.subscribe("rover_control_stream", 1000, controllerStreamCallback, ros::TransportHints().udp());

    ros::spin();

    return 0;
}