#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "my_rb1_ros/Rotate.h"
#include <cmath>

ros::Publisher cmd_pub;
ros::Subscriber odom_sub;

double current_yaw = 0.0;

double angle_difference(double target, double current)
{
    double diff = target - current;
    while (diff > M_PI)
        diff -= 2.0 * M_PI;
    while (diff < -M_PI)
        diff += 2.0 * M_PI;
    return diff;
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    double x = msg->pose.pose.orientation.x;
    double y = msg->pose.pose.orientation.y;
    double z = msg->pose.pose.orientation.z;
    double w = msg->pose.pose.orientation.w;

    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    current_yaw = atan2(siny_cosp, cosy_cosp);
}

bool rotate_callback(my_rb1_ros::Rotate::Request &req,
                     my_rb1_ros::Rotate::Response &res)
{
    double target_angle = req.degrees * M_PI / 180.0;
    double initial_yaw = current_yaw;
    double final_yaw = initial_yaw + target_angle;

    ROS_INFO("Service Requested: Rotate %d degrees", req.degrees);

    geometry_msgs::Twist vel_msg;
    vel_msg.angular.z = (target_angle > 0) ? 0.5 : -0.5;

    ros::Rate rate(10);
    bool rotation_complete = false;

    while (ros::ok())
    {
        ros::spinOnce();

        double error = angle_difference(final_yaw, current_yaw);

        if (fabs(error) < 0.01) // Threshold of ~0.57 degrees
        {
            vel_msg.angular.z = 0.0;
            cmd_pub.publish(vel_msg);
            rotation_complete = true;
            break;
        }

        cmd_pub.publish(vel_msg);
        rate.sleep();
    }

    if (rotation_complete)
    {
        res.result = "Rotation completed successfully.";
        ROS_INFO("Rotation of %d degrees completed successfully.", req.degrees);
    }
    else
    {
        res.result = "Rotation failed.";
        ROS_WARN("Rotation of %d degrees failed.", req.degrees);
    }

    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rotate_service_server");
    ros::NodeHandle nh;

    cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    odom_sub = nh.subscribe("odom", 50, odomCallback);

    ros::ServiceServer service = nh.advertiseService("rotate_robot", rotate_callback);
    ROS_INFO("Service /rotate_robot Ready");

    ros::spin();

    return 0;
}
