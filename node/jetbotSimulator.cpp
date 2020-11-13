#include <ros/ros.h>

// interactive marker
#include <interactive_markers/interactive_marker_server.h>

#include <tf2/impl/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int32MultiArray.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include "f1tenth_simulator/pose_2d.hpp"
#include "f1tenth_simulator/ackermann_kinematics.hpp"

#include "f1tenth_simulator/car_state.hpp"
#include "f1tenth_simulator/car_params.hpp"
#include "f1tenth_simulator/precompute.hpp"
#include "f1tenth_simulator/jetbotDynamics.hpp"

#include <iostream>
#include <math.h>

using namespace racecar_simulator;

class jetbotSimulator{
private:
     // A ROS node
    ros::NodeHandle n;

    // The transformation frames used
    std::string map_frame, base_frame, scan_frame;

    // obstacle states (1D index) and parameters
    std::vector<int> added_obs;
    // listen for clicked point for adding obstacles
    ros::Subscriber obs_sub;
    int obstacle_size;

    // interactive markers' server
    interactive_markers::InteractiveMarkerServer im_server;

    // The car state and parameters
    twoWheelBotState jetbotState;
    double previous_seconds;
    double max_wheel_speed;
    double max_wheel_torque;
    double desired_speed, desired_curvature;
    double accel;
    double angularVleocityThreshold;
    double rightWheelTorqueCommand;
    double leftWheelTorqueCommand;
    twoWheelBotParameters jetbotParameters;

    // For publishing transformations
    tf2_ros::TransformBroadcaster br;

    // A timer to update the pose
    ros::Timer update_pose_timer;

    // Listen for drive commands
    ros::Subscriber drive_sub;

    // Listen for a map
    ros::Subscriber map_sub;
    bool map_exists = false;

    // Listen for updates to the pose
    ros::Subscriber pose_sub;
    ros::Subscriber pose_rviz_sub;

    // Publish a scan, odometry, and imu data
    bool broadcast_transform;
    bool pub_gt_pose;
    ros::Publisher pose_pub;
    ros::Publisher odom_pub;

    // publisher for map with obstacles
    ros::Publisher map_pub;

    // for obstacle collision
    int map_width, map_height;
    double map_resolution, origin_x, origin_y;

    // safety margin for collisions
    double thresh;
    double speed_clip_diff;

    // pi
    const double PI = 3.1415;

    // for collision check
    bool TTC = false;
    double ttc_threshold;

public:

    jetbotSimulator(): im_server("jetbot_sim"){
        n = ros::NodeHandle("~");

        jetbotState = {.x=0, .y=0, .theta=0, .velocity=0, .angular_velocity=0, .leftWheelSpeed=0, .rigthWheelSpeed=0, .std_dyn=false};
        accel = 0.0;
        desired_speed = 0.0;
        desire_curvature = 0.0;
        previous_seconds = ros::Time::now().toSec();

        // Get the topic names
        std::string drive_topic, map_topic, pose_topic, gt_pose_topic, 
        pose_rviz_topic, odom_topic, diff_drive_topic;
        n.getParam("drive_topic", drive_topic);
        n.getParam("map_topic", map_topic);    
        n.getParam("pose_topic", pose_topic);
        n.getParam("odom_topic", odom_topic);
        n.getParam("pose_rviz_topic", pose_rviz_topic);
        n.getParam("ground_truth_pose_topic", gt_pose_topic);
        n.getParam("diff_drive_topic", diff_drive_topic);

        // Get the transformation frame names
        n.getParam("map_frame", map_frame);
        n.getParam("base_frame", base_frame);

        //Fetch car parameters
        n.getParam("update_pose_rate", update_pose_rate);
        n.getParam("jetBot_max_wheel_speed", max_wheel_speed);
        n.getParam("jetBot_max_wheel_torque", max_wheel_torque);
        n.getParam("jetBot_mas", jetBotParameters.mass);
        n.getParam("jetBot_wheel_radius", jetBotParameters.wheelRadius);
        n.getParam("jetBot_length" , jetBotParameters.length);
        n.getParam("jetBot_width", jetBotParameters.track);
        n.getParam("jetbot_wheel_damping" , jetBotParameters.wheelDampingFactor);
        jetBotParameters.I_z=(1.0/12.0)*(jetBotParameters.mass)*(jetBotParameters.track^2 + jetBotParameters.length^2);   

        // Determine if we should broadcast
        n.getParam("broadcast_transform", broadcast_transform);
        n.getParam("publish_ground_truth_pose", pub_gt_pose);

        // Get obstacle size parameter
        n.getParam("obstacle_size", obstacle_size);

        // Make a publisher for odometry messages
        odom_pub = n.advertise<nav_msgs::Odometry>(odom_topic, 1);

        // Make a publisher for publishing map with obstacles
        map_pub = n.advertise<nav_msgs::OccupancyGrid>("/map", 1);

        // Make a publisher for ground truth pose
        pose_pub = n.advertise<geometry_msgs::PoseStamped>(gt_pose_topic, 1);

        // Start a timer to output the pose
        update_pose_timer = n.createTimer(ros::Duration(update_pose_rate), &jetBotSimulator::update_pose, this);

        // Start a subscriber to listen to drive commands
        drive_sub = n.subscribe(diff_drive_topic, 1, &jetBotSimulator::drive_callback, this);

        // Start a subscriber to listen to new maps
        //map_sub = n.subscribe(map_topic, 1, &jetBotSimulator::map_callback, this);

        // Start a subscriber to listen to pose messages
        pose_sub = n.subscribe(pose_topic, 1, &jetBotSimulator::pose_callback, this);
        pose_rviz_sub = n.subscribe(pose_rviz_topic, 1, &jetBotSimulator::pose_rviz_callback, this);

        // obstacle subscriber
        obs_sub = n.subscribe("/clicked_point", 1, &jetBotSimulator::obs_callback, this);

        // get collision safety margin
        n.getParam("coll_threshold", thresh);
        n.getParam("ttc_threshold", ttc_threshold);
        n.getParam("angular_velocity_threshold" , angularVelocityThreshold);

        // OBSTACLE BUTTON:
        // wait for one map message to get the map data array
        boost::shared_ptr<nav_msgs::OccupancyGrid const> map_ptr;
        nav_msgs::OccupancyGrid map_msg;
        map_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map");
        if (map_ptr != NULL) {
            map_msg = *map_ptr;
        }
        original_map = map_msg;
        current_map = map_msg;
        std::vector<int8_t> map_data_raw = map_msg.data;
        std::vector<int> map_data(map_data_raw.begin(), map_data_raw.end());

        map_width = map_msg.info.width;
        map_height = map_msg.info.height;
        origin_x = map_msg.info.origin.position.x;
        origin_y = map_msg.info.origin.position.y;
        map_resolution = map_msg.info.resolution;

        // create button for clearing obstacles
        visualization_msgs::InteractiveMarker clear_obs_button;
        clear_obs_button.header.frame_id = "map";
        // clear_obs_button.pose.position.x = origin_x+(1/3)*map_width*map_resolution;
        // clear_obs_button.pose.position.y = origin_y+(1/3)*map_height*map_resolution;
        // TODO: find better positioning of buttons
        clear_obs_button.pose.position.x = 0;
        clear_obs_button.pose.position.y = -5;
        clear_obs_button.scale = 1;
        clear_obs_button.name = "clear_obstacles";
        clear_obs_button.description = "Clear Obstacles\n(Left Click)";
        visualization_msgs::InteractiveMarkerControl clear_obs_control;
        clear_obs_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;
        clear_obs_control.name = "clear_obstacles_control";
        
        // make a box for the button
        visualization_msgs::Marker clear_obs_marker;
        clear_obs_marker.type = visualization_msgs::Marker::CUBE;
        clear_obs_marker.scale.x = clear_obs_button.scale*0.45;
        clear_obs_marker.scale.y = clear_obs_button.scale*0.65;
        clear_obs_marker.scale.z = clear_obs_button.scale*0.45;
        clear_obs_marker.color.r = 0.0;
        clear_obs_marker.color.g = 1.0;
        clear_obs_marker.color.b = 0.0;
        clear_obs_marker.color.a = 1.0;

        clear_obs_control.markers.push_back(clear_obs_marker);
        clear_obs_control.always_visible = true;
        clear_obs_button.controls.push_back(clear_obs_control);

        im_server.insert(clear_obs_button);
        im_server.setCallback(clear_obs_button.name, boost::bind(&jetBotSimulator::clear_obstacles, this, _1));

        im_server.applyChanges();

        ROS_INFO("Simulator constructed.");
    
    }

    void update_pose(const ros::TimerEvenet&){

        // Update the pose 
        ros::Time timestamp = ros::Time::now();
        double current_seconds = timestamp.toSec();
        jetbotState = jetbotDynamics::update(
            jetbotState,
            rightWheelTorqueCommand,
            leftWheelTorqueCommand,
            jetbotParameters,
            angularVelocityThreshold,
            current_seconds - previous_seconds);
            
        jetBotState.leftWheelSpeed = std::min(std::max(jetbotState.leftWheelSpeed , -max_wheel_speed),max_wheel_speed);
        jetBotState.rightWheelSpeed = std::min(std::max(jetbotState.rightWheelSpeed , -max_wheel_speed),max_wheel_speed);
        previous_seconds = current_seconds;

        /// Publish the pose as a transformation
        pub_pose_transform(timestamp);

        /// Make an odom message as well and publish it
        pub_odom(timestamp);

    
    }
    ///---------PUBLISHIG HELPER FUNCTIONS-------
    void pub_pose_transform(ros::Time timestamp) {
            // Convert the pose into a transformation
            geometry_msgs::Transform t;
            t.translation.x = jetbotState.x;
            t.translation.y = jetbotState.y;
            tf2::Quaternion quat;
            quat.setEuler(0., 0., jetbotState.theta);
            t.rotation.x = quat.x();
            t.rotation.y = quat.y();
            t.rotation.z = quat.z();
            t.rotation.w = quat.w();

            // publish ground truth pose
            geometry_msgs::PoseStamped ps;
            ps.header.frame_id = "/map";
            ps.pose.position.x = jetbotState.x;
            ps.pose.position.y = jetbotState.y;
            ps.pose.orientation.x = quat.x();
            ps.pose.orientation.y = quat.y();
            ps.pose.orientation.z = quat.z();
            ps.pose.orientation.w = quat.w();

            // Add a header to the transformation
            geometry_msgs::TransformStamped ts;
            ts.transform = t;
            ts.header.stamp = timestamp;
            ts.header.frame_id = map_frame;
            ts.child_frame_id = base_frame;

            // Publish them
            if (broadcast_transform) {
                br.sendTransform(ts);
            }
            if (pub_gt_pose) {
                pose_pub.publish(ps);
            }
    }

    void pub_odom(ros::Time timestamp) {
            // Make an odom message and publish it
            nav_msgs::Odometry odom;
            odom.header.stamp = timestamp;
            odom.header.frame_id = map_frame;
            odom.child_frame_id = base_frame;
            odom.pose.pose.position.x = jetbotState.x;
            odom.pose.pose.position.y = jetbotState.y;
            tf2::Quaternion quat;
            quat.setEuler(0., 0., jetbotState.theta);
            odom.pose.pose.orientation.x = quat.x();
            odom.pose.pose.orientation.y = quat.y();
            odom.pose.pose.orientation.z = quat.z();
            odom.pose.pose.orientation.w = quat.w();
            odom.twist.twist.linear.x = jetbotState.velocity;
            odom.twist.twist.angular.z = jetbotState.angular_velocity;
            odom_pub.publish(odom);
    }
        //--Callback Functions-----//
    void drive_callback(const diffDriveMsgs::motorCmd &msg){
            rightWheelTorqueCommand = msg.rightWheelTorque;
            leftWheelTorqueCommand = msg.leftWheelTorque;
            rightWheelTorqueCommand = (std::min(std::max(rightWheelTorqueCommand, -1.0),1.0))*
                                        max_wheel_torque;
            leftWheelTorqueCommand = (std::min(std::max(leftWheelTorqueCommand, -1),1))*
                                        max_wheel_torque;        
    }
    void pose_callback(const geometry_msgs::PoseStamped & msg) {
        state.x = msg.pose.position.x;
        state.y = msg.pose.position.y;
        geometry_msgs::Quaternion q = msg.pose.orientation;
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        state.theta = tf2::impl::getYaw(quat);
    }
    void pose_rviz_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msg) {
        geometry_msgs::PoseStamped temp_pose;
        temp_pose.header = msg->header;
        temp_pose.pose = msg->pose.pose;
        pose_callback(temp_pose);
    }
    void clear_obstacles(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
        bool clear_obs_clicked = false;
        if (feedback->event_type == 3) {
            clear_obs_clicked = true;
        }
        if (clear_obs_clicked) {
            ROS_INFO("Clearing obstacles.");
            current_map = original_map;
            map_pub.publish(current_map);

            clear_obs_clicked = false;
        }
    }

    


};
int main(argc, char ** argv){
    ros::init(argc, argv, "jetBotSimulator");
    jetbotSimulator jetSim;
    ros::spin();
    return 0;

}