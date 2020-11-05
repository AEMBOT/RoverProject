#include "ros/ros.h"

#include "rover_client/ControlService.h"
#include "rover_main/controllerMap.h"
#include <string>

int main(int argc, char **argv){

    //Create a new node to publish data from the ControllerService to a global topic stream
    ros::init(argc, argv, "controller_info_publisher");


    //Create a new NodeHandle to handle publishing/subscribing
    ros::NodeHandle n;

    // Create service caller to get information from the ControllerService, persistance is true to prevent unnecessary latency as this service is used constantly
    ros::ServiceClient controller_service = n.serviceClient<rover_client::ControlService>("controller_service", true);

    // Wait for the service to be up
    controller_service.waitForExistence();

    // Variable to store the return values of the controller_service
    rover_client::ControlService srv;

    // Advertise the "rover_control_stream" topic with a queue size of 1000, using an update rate of 250hz
    ros::Publisher controller_publisher = n.advertise<rover_main::controllerMap>("rover_control_stream", 1000);
    ros::Rate update_rate(250);

    // While ROS is running constantly call the service to get the new controller values
    while (ros::ok()){

        // If the call is succsessful publish the data to the topic
        if(controller_service.call(srv)){
            controller_publisher.publish(srv.response.controllerState);
        }

        // If not announce that the call failed and exit
        else{
            ROS_INFO("Failed to call ControllerService");
            return 1;
        }

        // Sleep to keep in time with the update_rate
        update_rate.sleep();
    }
   
    // If all is well return 0 on exit
    return 0;
}