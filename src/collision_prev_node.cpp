#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "collision_prev_package/constants.h"
#include <typeinfo>

#include <stdlib.h>


geometry_msgs::Twist current_velocity;
std::vector<float> ranges(270, 0);

void AdjustVelocity(const geometry_msgs::Twist& msg)
{
    current_velocity = geometry_msgs::Twist(msg);
    ROS_INFO("Desired Velocity: linear=(%f, %f, %f) angular=(%f, %f, %f)",
        current_velocity.linear.x, current_velocity.linear.y, current_velocity.linear.z,
        current_velocity.angular.x, current_velocity.angular.y, current_velocity.angular.z);

    int forward = static_cast<int>(ranges.size() / 2);
    for (int i = forward - ANGLE_MARGIN; i < forward + ANGLE_MARGIN; i++)
    {
        if (ranges[i] < MIN_LASER_READING) {
            current_velocity.linear.x = 0;
            current_velocity.linear.y = 0;
            current_velocity.linear.z = 0;
            break;
        }
    }
}

void ReadLaser(const sensor_msgs::LaserScan& msg)
{
    ROS_INFO("Reading Laser ranges (%d), forward distance: %f",
        msg.ranges.size(), msg.ranges[static_cast<int>(ceil(msg.ranges.size() / 2))]);
    ranges = std::vector<float>(msg.ranges);
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
    std::string robot_name("Robot0");
    int opt;
    while ((opt = getopt(argc, (argv), "nld:")) != -1) {
        switch(opt) {
            case 'n':
                robot_name = optarg;
                break;
            case 'l':
                laser_topic = optarg;
                break;
            case 'd':
                input_vel_topic = optarg;
                break;
            default:
                printf("The -%c is not a recognized parameter\n", opt);
                break;
        }
    }

    // Initialize ROS
    ros::init(argc, argv, robot_name + "_collsion_prevention");    

    // Create Access point to communications to ROS
    ros::NodeHandle n;

    // To get correct scope additions
    robot_name += "/";

    ROS_INFO("Subscribing to '%s' for lasers", robot_name + laser_topic);
    // Tell ROS this node will subscribe to robot's laser target
    ros::Subscriber laser_subscriber = n.subscribe(robot_name + laser_topic, 1000, ReadLaser);

    ROS_INFO("Subscribing to '%s' for input velocity", robot_name + input_vel_topic);
    // Tell ROS this node will subscribe to desired velocity topic
    ros::Subscriber vel_subscriber = n.subscribe(robot_name + input_vel_topic, 1000, AdjustVelocity);

    ROS_INFO("Publishing to '%s' for output velocity", robot_name + output_vel_topic);
    // Tell ROS this node will publish to robot's output topic
    ros::Publisher vel_publisher = n.advertise<geometry_msgs::Twist>(robot_name + output_vel_topic, 1000);

    ros::Rate loop_rate(10);

    while(ros::ok()) {
        ROS_INFO("Adjusted Velocity: linear=(%f, %f, %f) angular=(%f, %f, %f)",
            current_velocity.linear.x, current_velocity.linear.y, current_velocity.linear.z,
            current_velocity.angular.x, current_velocity.angular.y, current_velocity.angular.z);
        vel_publisher.publish(current_velocity);
        ros::spinOnce();
        loop_rate.sleep();
    }

    /**
     * ros::spin() will enter a loop, pumping callbacks. With this version, all
     * callbacks will be called from within this thread (the main one). ros::spin()
     * wil exit when Ctrl-C is pressed, or the node is shutdown by the master.
     */
    // ros::spin();

    return 0;
}