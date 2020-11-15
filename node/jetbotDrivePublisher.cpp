#include <ros/ros.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>

#include "f1tenth_simulator/channel.h"

class jetbotDriveCmd
{
private:
    ros::NodeHandle n;

    //ros::Subscriber mux_sub;

    ros::Subscriber key_sub;
    //ros::Subscriber joy_sub;

    ros::Publisher diff_drive_pub;

    //int joy_mux_idx;
    //int key_mux_idx;

    // Mux controller array
    //std::vector<bool> mux_controller;
    //int mux_size;

    // Channel array
    //std::vector<Channel *> channels;

    // Make Channel class have access to these private variables
    //friend class Channel;

    // For printing
    //std::vector<bool> prev_mux;

    // Params for joystick calculations
    //int joy_speed_axis, joy_angle_axis;
    //double max_toruqe = 1;
    // For keyboard driving
    double prev_key_velocity = 0.0;
    double keyboard_max_torque = 1;

public:
    jetbotDriveCmd()
    {
        n = ros::NodeHandle("~");

        std::string diff_drive_topic, mux_topic, joy_topic, key_topic n.getParam("diff_drive_topic", diff_drive_topic);
        //n.getParam("joy_topic", joy_topic);
        //n.getParam("mux_topic", mux_topic);
        //n.getParam("keyboard_topic", key_topic);

        diff_drive_pub = n.advertise<std_msgs::float64MultiArray>(diff_drive_topic, 10);

        //mux_sub = n.subscribe(mux_topic, 1, &jetbotDriveCmd::mux_callback, this);

        // Start subscribers to listen to joy and keyboard messages
        //joy_sub = n.subscribe(joy_topic, 1, &jetbotDriveCmd::joy_callback, this);
        key_sub = n.subscribe(key_topic, 1, &jetbotDriveCmd::key_callback, this);

        //n.getParam("joy_mux_idx", joy_mux_idx);
        //n.getParam("key_mux_idx", key_mux_idx);

        // get params for joystick calculations
        //n.getParam("joy_speed_axis", joy_speed_axis);
        //n.getParam("joy_angle_axis", joy_angle_axis);
        //n.getParam("mux_size", mux_size);

        // initialize mux controller
        /**mux_controller.reserve(mux_size);
        prev_mux.reserve(mux_size);
        for (int i = 0; i < mux_size; i++)
        {
            mux_controller[i] = false;
            prev_mux[i] = false;
        }

        channels = std::vector<Channel *>();**/
    }

    /**void add_channel(std::string channel_name, std::string drive_topic, int mux_idx_)
    {
        Channel *new_channel = new Channel(channel_name, diff_drive_topic, mux_idx_, this);
        channels.push_back(new_channel);
    }**/

    void publish_to_diff_drive(double rightWheelTrq, leftWheelTrq)
    {
        std_msgs::float64MultiArray diffDriveMsg;
        diffDriveMsg.data[0] = rightWheelTrq;
        diffDriveMsg.data[1] = leftWheelTrq;

        diff_drive_pub.publish(diffDriveMsg);
    }
    /**void mux_callback(const std_msgs::Int32MultiArray &msg)
    {
        for (int i = 0; i < mux_size; i++)
        {
            mus_controller[i] = bool(msg.data[i]);
        }
        // Prints the mux whenever it is changed
        bool changed = false;
        // checks if nothing is on
        bool anything_on = false;
        for (int i = 0; i < mux_size; i++)
        {
            changed = changed || (mux_controller[i] != prev_mux[i]);
            anything_on = anything_on || mux_controller[i];
        }
        if (changed)
        {
            std::cout << "MUX: " << std::endl;
            for (int i = 0; i < mux_size; i++)
            {
                std::cout << mux_controller[i] << std::endl;
                prev_mux[i] = mux_controller[i];
            }
            std::cout << std::endl;
        }
        if (!anything_on)
        {
            // if no mux channel is active, halt the car
            publish_to_drive(0.0, 0.0);
        }
    }**/
    void key_callback(const std::msgs:String & msg){
        double leftWheelTrq;
        double rightWheelTrq;

        bool publish = true;

        if (msg.data == "w"){
            leftWheelTrq = 1.0;
            rightWheelTrq = 1.0;
        
        }else if(msg.data=="s"){
            leftWheelTrq = -1.0;
            rightWheelTrq = -1.0;

        }else if(msg.data == "a"){
            leftWheelTrq = -1.0;
            rightWheelTrq = 1.0;

        }else if(msg.data == "d") {
            leftWheelTrq = 1.0;
            rightWheelTrq = -1.0;
        }else if (msg.data ==" "){
            leftWheelTrq = 0.0;
            rigthWheelTrq = 0.0;
        }else {
            publish = false;
        }
        if (publish){
            publish_to_diff_drive(rightWheelTrq , leftWheelTrq);

        }
    }
   
}

