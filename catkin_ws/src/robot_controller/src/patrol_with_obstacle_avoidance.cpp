#include <iostream>
#include <vector>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <thread>
#include <chrono>

// Include ROS headers
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// Patrol duty points (predefined but randomized)
struct PatrolPoint {
    double x;
    double y;
};

class RobotPatrol {
public:
    RobotPatrol(ros::NodeHandle &nh) : nh_(nh), obstacle_detected_(false), dynamic_obstacle_wait_(false) {
        // Set up publishers and subscribers
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/sim_p3at/cmd_vel", 10);
        pointcloud_sub_ = nh_.subscribe("/lidar_points", 10, &RobotPatrol::pointCloudCallback, this);
        odom_sub_ = nh_.subscribe("/odometry/filtered", 10, &RobotPatrol::odomCallback, this);

        // Random patrol points initialization
        initializePatrolPoints();
    }

    void startPatrolling() {
        while (ros::ok()) {
            PatrolPoint next_point = selectRandomPatrolPoint();
            moveToTarget(next_point);
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber pointcloud_sub_;
    ros::Subscriber odom_sub_;

    std::vector<PatrolPoint> patrol_points_;
    PatrolPoint current_position_;
    bool obstacle_detected_;
    bool dynamic_obstacle_wait_;

    void initializePatrolPoints() {
        srand(time(0));
        for (int i = 0; i < 5; ++i) {
            patrol_points_.push_back({randomCoordinate(), randomCoordinate()});
        }
    }

    double randomCoordinate() {
        return static_cast<double>(rand() % 20 - 10); // Random range [-10, 10]
    }

    PatrolPoint selectRandomPatrolPoint() {
        int index = rand() % patrol_points_.size();
        return patrol_points_[index];
    }

    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud);

        obstacle_detected_ = false;

        for (const auto &point : cloud.points) {
            // Consider points within a certain range as obstacles
            if (std::hypot(point.x, point.y) < 1.0 && std::abs(point.z) < 1.0) { // Threshold distance
                obstacle_detected_ = true;
                break;
            }
        }
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
        current_position_.x = msg->pose.pose.position.x;
        current_position_.y = msg->pose.pose.position.y;
    }

    void moveToTarget(const PatrolPoint &target) {
        geometry_msgs::Twist cmd_vel;
        double target_distance = calculateDistance(target);
        double tolerance = 0.5;

        while (ros::ok() && target_distance > tolerance) {
            ros::spinOnce();

            if (obstacle_detected_) {
                if (dynamic_obstacle_wait_) {
                    waitForObstacle();
                } else {
                    avoidObstacle();
                }
            } else {
                cmd_vel.linear.x = 0.5; // Move forward
                cmd_vel.angular.z = calculateAngularSpeed(target);
                cmd_vel_pub_.publish(cmd_vel);
            }

            target_distance = calculateDistance(target);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        stopRobot();
    }

    double calculateDistance(const PatrolPoint &target) {
        return std::sqrt(std::pow(target.x - current_position_.x, 2) +
                         std::pow(target.y - current_position_.y, 2));
    }

    double calculateAngularSpeed(const PatrolPoint &target) {
        double angle_to_target = std::atan2(target.y - current_position_.y, target.x - current_position_.x);
        return angle_to_target * 0.5; // Proportional control
    }

    void waitForObstacle() {
        ROS_INFO("Waiting for obstacle to disappear...");
        while (obstacle_detected_ && ros::ok()) {
            ros::spinOnce();
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    void avoidObstacle() {
        ROS_INFO("Avoiding obstacle...");
        geometry_msgs::Twist cmd_vel;

        // Simple reactive avoidance by turning
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.5; // Turn in place
        cmd_vel_pub_.publish(cmd_vel);

        // Continue moving forward after avoiding
        std::this_thread::sleep_for(std::chrono::seconds(2));
        cmd_vel.angular.z = 0.0;
        cmd_vel.linear.x = 0.5;
        cmd_vel_pub_.publish(cmd_vel);
    }

    void stopRobot() {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub_.publish(cmd_vel);
        ROS_INFO("Target reached!");
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_patrol");
    ros::NodeHandle nh;

    RobotPatrol patrol(nh);
    patrol.startPatrolling();

    return 0;
}

