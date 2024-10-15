#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "my_rb1_ros/Rotate.h"
#include <cmath>

class RotateService
{
public:
    RotateService()
    {
        cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        odom_sub = nh.subscribe("odom", 50, &RotateService::odomCallback, this);
        service = nh.advertiseService("rotate_robot", &RotateService::rotateCallback, this);
        ROS_INFO("Service /rotate_robot Ready");
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

    bool rotateCallback(my_rb1_ros::Rotate::Request &req, my_rb1_ros::Rotate::Response &res)
    {
        double target_angle = req.degrees * M_PI / 180.0;
        double initial_yaw = current_yaw;
        double accumulated_rotation = 0.0;

        ROS_INFO("Service Requested: Rotate %d degrees", req.degrees);

        geometry_msgs::Twist vel_msg;
        ros::Rate rate(10);
        bool rotation_complete = false;

        while (ros::ok())
        {
            ros::spinOnce();

            double delta_yaw = angleDifference(current_yaw, initial_yaw);
            accumulated_rotation += delta_yaw;
            initial_yaw = current_yaw;

            if (fabs(accumulated_rotation) >= fabs(target_angle))
            {
                vel_msg.angular.z = 0.0;
                cmd_pub.publish(vel_msg);
                rotation_complete = true;
                break;
            }

            vel_msg.angular.z = (target_angle > 0) ? 0.5 : -0.5;
            cmd_pub.publish(vel_msg);

            rate.sleep();
        }

        if (rotation_complete)
        {
            res.result = "Rotation completed successfully.";
            ROS_INFO("Rotation of %d degrees completed successfully.", req.degrees);
        }

        return true;
    }

private:
    ros::NodeHandle nh;
    ros::Publisher cmd_pub;
    ros::Subscriber odom_sub;
    ros::ServiceServer service;

    double current_yaw = 0.0;
    double angleDifference(double target, double current)
    {
        double diff = target - current;
        while (diff > M_PI)
            diff -= 2.0 * M_PI;
        while (diff < -M_PI)
            diff += 2.0 * M_PI;
        return diff;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rotate_service_server");
    RotateService rotateService;
    ros::spin();

    return 0;
}
