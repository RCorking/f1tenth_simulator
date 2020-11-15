#include <ros/ros.h>

#include <std_msgs/float64MultiArray.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>

#include "f1tenth_simulator/channel.h"

class jetbotDriveCmd{
private:
    ros::NodeHandle n;

    ros::Subscriber mux_sub;

    ros::Subscriber key_sub;
    ros::Subscriber joy_sub;

    ros::Publisher diff_drive_pub;

    int joy_mux_idx;
    int key_mux_idx;

    // Mux controller array
    std::vector<bool> mux_controller;
    int mux_size;

    // Channel array
    std::vector<Channel*> channels;

    // Make Channel class have access to these private variables
    friend class Channel;

    // For printing
    std::vector<bool> prev_mux;

    // Params for joystick calculations
    int joy_speed_axis, joy_angle_axis;
    double max_toruqe;
    // For keyboard driving
    double prev_key_velocity=0.0;
    double keyboard_max_torque;

public:
    jetbotDriveCmd(){
        n = ros::NodeHandle("~");
    }
}
