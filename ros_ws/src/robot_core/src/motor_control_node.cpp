#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>

ros::Publisher left_motor_pub;
ros::Publisher right_motor_pub;

void twist_callback(const geometry_msgs::Twist::ConstPtr& msg) {
    double linear_velocity = msg->linear.x;
    double angular_velocity = msg->angular.z;

    // Calculate motor speeds
    int left_motor_speed = static_cast<int>((linear_velocity - angular_velocity) * 255);
    int right_motor_speed = static_cast<int>((linear_velocity + angular_velocity) * 255);

    // Limit motor speeds to the range [-255, 255]
    left_motor_speed = std::max(std::min(left_motor_speed, 255), -255);
    right_motor_speed = std::max(std::min(right_motor_speed, 255), -255);

    // Publish motor commands
    std_msgs::Int16 left_motor_msg;
    std_msgs::Int16 right_motor_msg;
    left_motor_msg.data = left_motor_speed;
    right_motor_msg.data = right_motor_speed;
    left_motor_pub.publish(left_motor_msg);
    right_motor_pub.publish(right_motor_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "motor_control_node");
    ros::NodeHandle nh;

    left_motor_pub = nh.advertise<std_msgs::Int16>("left_motor", 10);
    right_motor_pub = nh.advertise<std_msgs::Int16>("right_motor", 10);

    ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 10, twist_callback);

    ROS_INFO("Twist to motors node initialized");

    ros::spin();

    return 0;
}