#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>

class TeleopRobot {
public:
    TeleopRobot() : linear_velocity_(0.0), angular_velocity_(0.0), running_(true) {
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/sim_p3at/cmd_vel", 10);

        // Set terminal to raw mode for key input
        tcgetattr(STDIN_FILENO, &original_termios_);
        struct termios raw = original_termios_;
        raw.c_lflag &= ~(ECHO | ICANON);
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    }

    ~TeleopRobot() {
        // Restore terminal to original settings
        tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
    }

    void run() {
        ros::Rate rate(10); // 10 Hz loop rate
        while (ros::ok() && running_) {
            char key = getKey();

            processKey(key);

            // Gradual stop if no key is pressed
            if (key == 0) {
                gradualStop();
            }

            publishVelocity();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    geometry_msgs::Twist move_cmd_;
    double linear_velocity_;
    double angular_velocity_;
    bool running_;
    struct termios original_termios_;

    char getKey() {
        char key = 0;
        struct timeval timeout;
        fd_set readfds;

        FD_ZERO(&readfds);
        FD_SET(STDIN_FILENO, &readfds);

        // Set timeout for non-blocking input
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000; // 0.1 seconds

        if (select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &timeout) > 0) {
            read(STDIN_FILENO, &key, 1);
        }

        return key;
    }

    void processKey(char key) {
        switch (key) {
            case 'w': linear_velocity_ = 0.5; break; // Move forward
            case 's': linear_velocity_ = -0.5; break; // Move backward
            case 'a': angular_velocity_ = 0.5; break; // Turn left
            case 'd': angular_velocity_ = -0.5; break; // Turn right
            case 'q': angular_velocity_ = 0.0; break; // Stop turning
            case 'e': angular_velocity_ = 0.0; break; // Stop turning
            case 'x': 
                linear_velocity_ = 0.0; 
                angular_velocity_ = 0.0; 
                running_ = false; 
                break; // Exit
            default: break;
        }
    }

    void gradualStop() {
        const double deceleration_rate = 0.05; // Adjust for smoother stops

        if (linear_velocity_ > 0.0) linear_velocity_ -= deceleration_rate;
        else if (linear_velocity_ < 0.0) linear_velocity_ += deceleration_rate;

        if (angular_velocity_ > 0.0) angular_velocity_ -= deceleration_rate;
        else if (angular_velocity_ < 0.0) angular_velocity_ += deceleration_rate;
    }

    void publishVelocity() {
        move_cmd_.linear.x = linear_velocity_;
        move_cmd_.angular.z = angular_velocity_;
        cmd_vel_pub_.publish(move_cmd_);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "teleop_robot");

    TeleopRobot teleop_robot;
    try {
        teleop_robot.run();
    } catch (const std::exception& e) {
        ROS_ERROR("Error: %s", e.what());
    }

    std::cout << "\nTeleop control terminated." << std::endl;
    return 0;
}
