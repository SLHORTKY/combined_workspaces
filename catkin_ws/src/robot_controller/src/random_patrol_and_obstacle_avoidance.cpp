#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <random>
#include <cmath>

ros::Publisher cmd_vel_pub;
ros::Publisher goal_pub;

// Define a random number generator for generating random patrol points
std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<> dis_x(-10.0, 10.0); // x range
std::uniform_real_distribution<> dis_y(-10.0, 10.0); // y range

// Function to generate random patrol waypoint within a bounded area
geometry_msgs::PoseStamped generate_random_waypoint() {
    geometry_msgs::PoseStamped waypoint;
    waypoint.pose.position.x = dis_x(gen);
    waypoint.pose.position.y = dis_y(gen);
    waypoint.pose.position.z = 0.0;
    waypoint.pose.orientation.w = 1.0; // No rotation (facing forward)

    return waypoint;
}

// Function to process laser scan data and check for obstacles
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
    bool obstacle_detected = false;

    // Check laser scan ranges for obstacles (e.g., if distance is less than 1 meter)
    for (const float& range : scan->ranges) {
        if (range < 1.0) {  // Obstacle detected if range is less than 1 meter
            obstacle_detected = true;
            break;
        }
    }

    // Robot behavior based on obstacle detection
    geometry_msgs::Twist cmd_vel_msg;
    if (obstacle_detected) {
        cmd_vel_msg.linear.x = 0.0; // Stop robot
        cmd_vel_msg.angular.z = 0.5; // Turn to avoid obstacle
        ROS_WARN("Obstacle detected! Turning to avoid.");
    } else {
        cmd_vel_msg.linear.x = 0.5;  // Move forward
        cmd_vel_msg.angular.z = 0.0; // Go straight
    }

    // Publish the velocity command
    cmd_vel_pub.publish(cmd_vel_msg);
}

// Main function to handle patrol and obstacle avoidance
int main(int argc, char **argv) {
    ros::init(argc, argv, "random_patrol_with_avoidance");
    ros::NodeHandle nh;

    // Publisher for commanding robot to move to a goal
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    // Publisher to send velocity commands to the robot
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // Subscribe to LiDAR scan data
    ros::Subscriber laser_sub = nh.subscribe("/scan", 10, laserScanCallback);

    ros::Rate loop_rate(0.5);  // 0.5 Hz (1 waypoint every 2 seconds)

    while (ros::ok()) {
        // Generate random waypoint
        geometry_msgs::PoseStamped waypoint = generate_random_waypoint();

        // Publish the goal to the robot to start patrolling
        goal_pub.publish(waypoint);
        ROS_INFO("Patrolling to: (%.2f, %.2f)", waypoint.pose.position.x, waypoint.pose.position.y);

        // Wait for a while before generating the next waypoint (5 seconds)
        ros::Duration(5.0).sleep();  // Stay at waypoint for 5 seconds

        ros::spinOnce();
    }

    return 0;
}

