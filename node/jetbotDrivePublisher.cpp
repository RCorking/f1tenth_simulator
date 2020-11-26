#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>

class jetbotDriveCmd
{
private:
    ros::NodeHandle n;


    ros::Subscriber key_sub;

    ros::Publisher diff_drive_pub;

    double prev_key_velocity = 0.0;
    double keyboard_max_torque = 1;

public:
    jetbotDriveCmd()
    {
        n = ros::NodeHandle("~");

        std::string diff_drive_topic, mux_topic, joy_topic, key_topic;
	    n.getParam("diff_drive_topic", diff_drive_topic);
        n.getParam("keyboard_topic", key_topic);

        diff_drive_pub = n.advertise<std_msgs::Float64MultiArray>(diff_drive_topic, 10);

        key_sub = n.subscribe(key_topic, 1, &jetbotDriveCmd::key_callback, this);

    }


    void publish_to_diff_drive(double rightWheelTrq,double leftWheelTrq)
    {
        std_msgs::Float64MultiArray diffDriveMsg;
	diffDriveMsg.data.clear();
	diffDriveMsg.data.push_back(rightWheelTrq);
	diffDriveMsg.data.push_back(leftWheelTrq);
        diff_drive_pub.publish(diffDriveMsg);
    }

    void key_callback(const std_msgs::String & msg){
        ROS_INFO("I'm in key_callback");
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
            rightWheelTrq = 0.0;
        }else {
            publish = false;
        }
        if (publish){
            publish_to_diff_drive(rightWheelTrq , leftWheelTrq);

        }
	ROS_INFO("end of key_callback");
    }
   
};
int main(int argc, char ** argv){
  ros::init(argc, argv, "jetbotDriveCmd");
  jetbotDriveCmd jetDriver;
  ros::spin();
  return 0;
  }


