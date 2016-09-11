#include <ros.h>
#include <std_msgs/RobotCmd.h>
#include <string>

int main() {
    ros::NodeHandle  nh;
    std_msgs::RobotCmd cmd_msg;

    ros::Publisher control_walking("control_walking", &cmd_msg);

    nh.initNode();
    nh.advertise(control_walking);

    //string cmd, inarg;

    while (1) {
        // Read user inputs
        cin>>cmd_msg.command>>cmd_msg.arg;

        // Send the command
        control_walking.publish( &cmd_msg );
        nh.spinOnce();
        // Stall for 1s
        sleep(1000);
    }
}