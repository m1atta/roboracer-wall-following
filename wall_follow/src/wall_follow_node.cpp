#include "rclcpp/rclcpp.hpp"
#include <string>
#include <cmath>       // added: std::isfinite, std::cos, std::sin, std::atan2
#include <algorithm>   // added: std::clamp, std::min
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class WallFollow : public rclcpp::Node {

public:
    WallFollow() : Node("wall_follow_node")
    {
        // Subscribe to the LiDAR and publish drive commands
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidarscan_topic, rclcpp::SensorDataQoS(),
            std::bind(&WallFollow::scan_callback, this, std::placeholders::_1));

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            drive_topic, 10);
    }

private:
    // PID CONTROL PARAMS
    double kp = 1.0;   // how hard to steer per metre of error
    double kd = 0.1;   // damps wobble (reacts to how fast the error changes)
    double ki = 0.0;   // fixes slow drift; usually 0 or tiny
    double servo_offset = 0.0;
    double prev_error = 0.0;
    double error = 0.0;
    double integral = 0.0;

    // Topics
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    // Other tuning values (hardcoded because the autograder passes no parameter file)
    double desired_dist = 1.0;      // metres to keep from the LEFT wall
    double lookahead = 1.0;         // L in the handout, metres
    double theta_deg = 50.0;        // angle between beam a and beam b
    double max_steer = 0.4189;      // ~24 degrees, the car's steering limit
    double front_open_dist = 3.0;   // front beam longer than this = open ahead (doorway, not corner)
    double max_speed = 1.5;         // top speed on straights

    // Scan info saved each callback so get_range() can turn an angle into an array index
    double angle_min = 0.0;
    double angle_increment = 0.0;
    int num_ranges = 0;
    double range_max = 30.0;
    double prev_time = -1.0;
    double dt = 0.025;

    double get_range(float* range_data, double angle)
    {
        /*
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle
        */

        // Angle -> array index (0 rad = straight ahead, + = left)
        int idx = static_cast<int>(std::round((angle - angle_min) / angle_increment));
        idx = std::clamp(idx, 0, num_ranges - 1);

        // If that beam is NaN/inf, use the nearest valid neighbour within 3 beams
        for (int offset = 0; offset <= 3; ++offset) {
            for (int i : {idx - offset, idx + offset}) {
                if (i >= 0 && i < num_ranges && std::isfinite(range_data[i])) {
                    return range_data[i];
                }
            }
        }
        return range_max;  // nothing valid nearby: treat as far away
    }

    double get_error(float* range_data, double dist)
    {
        /*
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        */

        const double theta = theta_deg * M_PI / 180.0;

        // Left wall: beam b points straight left (+90 deg), beam a is theta forward of it
        const double b = get_range(range_data, M_PI / 2.0);
        const double a = get_range(range_data, M_PI / 2.0 - theta);

        // Handout formulas
        const double alpha = std::atan2(a * std::cos(theta) - b, a * std::sin(theta));
        const double d_t = b * std::cos(alpha);
        double d_future = d_t + lookahead * std::sin(alpha);

        // Doorway vs corner: both make the side wall "disappear".
        // Corner = wall ahead. Doorway = open ahead -> don't let the gap pull us sideways.
        const double front = get_range(range_data, 0.0);
        if (front > front_open_dist) {
            d_future = std::min(d_future, dist + 0.5);
        }

        // Positive error = too far from the left wall
        return d_future - dist;
    }

    void pid_control(double error, double velocity)
    {
        /*
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        */
        double angle = 0.0;

        // PID: P = current error, I = sum of past error, D = how fast error is changing
        integral = std::clamp(integral + error * dt, -1.0, 1.0);
        const double derivative = (error - prev_error) / dt;
        prev_error = error;

        // Positive steering = left in ROS. Too far from the left wall (error > 0) -> steer left.
        angle = kp * error + ki * integral + kd * derivative + servo_offset;
        angle = std::clamp(angle, -max_steer, max_steer);

        // Slow down in turns (handout values), using velocity as the top speed
        const double angle_deg = std::abs(angle) * 180.0 / M_PI;
        if (angle_deg >= 20.0) {
            velocity = 0.5;
        } else if (angle_deg >= 10.0) {
            velocity = 1.0;
        }

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.header.stamp = this->now();
        drive_msg.drive.steering_angle = angle;
        drive_msg.drive.speed = velocity;
        drive_pub_->publish(drive_msg);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        /*
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        */
        // Save scan info for get_range()
        angle_min = scan_msg->angle_min;
        angle_increment = scan_msg->angle_increment;
        num_ranges = static_cast<int>(scan_msg->ranges.size());
        range_max = scan_msg->range_max;

        // Time since last scan (for the I and D terms)
        const double t = rclcpp::Time(scan_msg->header.stamp).seconds();
        if (prev_time > 0.0 && t - prev_time > 0.0 && t - prev_time < 0.5) {
            dt = t - prev_time;
        }
        prev_time = t;

        // The template's functions take float*; we only read the data, so const_cast is safe here
        float* ranges = const_cast<float*>(scan_msg->ranges.data());

        double error = get_error(ranges, desired_dist);
        double velocity = max_speed;  // pid_control lowers this in turns
        pid_control(error, velocity);
    }

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}