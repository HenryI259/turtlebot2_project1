#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "kobuki_msgs/BumperEvent.h"
#include "sensor_msgs/LaserScan.h"
#include <random>

using namespace std;

class ExplorerRobot {
private:
    // Node Handler
    ros::NodeHandle nh;

    // Publisher to TurtleBot2 /mobile_base/commands/velocity
    ros::Publisher pub;
    
    // Subscribers
    ros::Subscriber odom_sub;
    ros::Subscriber teleop_sub;
    ros::Subscriber bumper_sub;
    ros::Subscriber lidar_sub;

    // Rate that the robot updates
    ros::Rate rate;

    // Position
    double pos_x;
    double pos_y;
    double angle;

    // Robot speed recieved from keyboard
    double keyboard_linear;
    double keyboard_angular;

    // Bumper info
    int bumper_side;
    int bumper_state;

    // Lidar Distances
    double right_avg;
    double left_avg;

    // Robot speeds
    double linear_speed = 0.3;
    double angular_speed = 1;

    // Gets the postion
    void getPosition(const nav_msgs::Odometry::ConstPtr& msg) {
        pos_x = msg->pose.pose.position.x;
        pos_y = msg->pose.pose.position.y;
        geometry_msgs::Quaternion q = msg->pose.pose.orientation;
        // Calculate angle
        angle = atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    // Gets the keyboard inputs
    void getInputs(const geometry_msgs::Twist::ConstPtr& msg) {
        keyboard_linear = msg->linear.x;
        keyboard_angular = msg->angular.z;
    }

    // Gets the bumper info
    void getBumper(const kobuki_msgs::BumperEvent::ConstPtr& msg) {
        bumper_side = msg->bumper;
        bumper_state = msg->state;
    }

    void getLidar(const sensor_msgs::LaserScan::ConstPtr& msg) {
        int size = msg->ranges.size();
        int mid = size/2;

        double right_sum = 0;
        double left_sum = 0;
        int right_count = 0;
        int left_count = 0;

        // Find averages for both sides
        for (int i = 0; i < size; i++) {
            double dis = msg->ranges[i];
            if (dis < msg->range_max && dis > msg->range_min) {
                if (i > mid) {
                    left_sum += dis;
                    left_count++;
                }
                else {
                    right_sum += dis;
                    right_count++;
                }
            }
        }

        left_avg = (left_count > 0) ? left_sum / left_count : msg->range_max;
        right_avg = (right_count > 0) ? right_sum / right_count : msg->range_max;
    }

    double correctAngle(double a) {
        while (a > 3.14) {
            a -= 6.28;
        }
        while (a < -3.14) {
            a += 6.28;
        }
        return a;
    }

public:
    ExplorerRobot() : rate(60) {}

    void init() {
        pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
        odom_sub = nh.subscribe("/odom", 10, &ExplorerRobot::getPosition, this);
        teleop_sub = nh.subscribe("/my_teleop_node/cmd_vel", 10, &ExplorerRobot::getInputs, this);
        bumper_sub = nh.subscribe("/mobile_base/events/bumper", 10, &ExplorerRobot::getBumper, this);
        lidar_sub = nh.subscribe("/scan", 10, &ExplorerRobot::getLidar, this);
    }

    // move function
    void move() {
        double linear_wire;
        double angular_wire;

        double start_x = pos_x;
        double start_y = pos_y;
        double target_angle = angle;

        bool uninterrupted_turn = false;

        // init random generator
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> distrib(-0.262, 0.262);

        // main loop
        while (ros::ok()) {
            // default movement
            linear_wire = linear_speed;
            angular_wire = 0;

            // robot by random amount after traveling 1 ft
            if (sqrt((pos_x-start_x)*(pos_x-start_x) + (pos_y-start_y)*(pos_y-start_y)) > 0.3048 && !uninterrupted_turn) {
                target_angle = correctAngle(angle + distrib(gen));

                start_x = pos_x;
                start_y = pos_y;
            }

            // avoid obstacles
            if (left_avg < 0.7 && right_avg < 0.7 && !uninterrupted_turn) {
                target_angle = correctAngle(angle-3.14);
                printf("Input: %lf, Output: %lf\n", angle, target_angle);
                uninterrupted_turn = true;
            }
            else if (left_avg < 0.6 && !uninterrupted_turn) {
                target_angle = correctAngle(angle-1.05);
                uninterrupted_turn = true;
            }
            else if (right_avg < 0.6 && !uninterrupted_turn) {
                target_angle = correctAngle(angle+1.05);
                uninterrupted_turn = true;
            }

            if (uninterrupted_turn) {
                linear_wire = 0;
            }

            if (abs(target_angle-angle) > 0.05 && abs(abs(target_angle-angle)-6.28) > 0.05) {
                if ((angle > target_angle && angle-target_angle < 3.14) || (angle < target_angle && angle-target_angle < -3.14)) {
                    angular_wire = -angular_speed;
                }
                else {
                    angular_wire = angular_speed;
                }
            }
            else {
                uninterrupted_turn = false;
            }

            // override movement with the keyboard
            if (abs(keyboard_linear) > 0.01 || abs(keyboard_angular) > 0.01) {
                linear_wire = keyboard_linear;
                angular_wire = keyboard_angular;
                target_angle = angle;
                uninterrupted_turn = false;
            }

            // halt the robot when a collision is detected
            if (bumper_state) {
                linear_wire = 0;
            }
            

            // Publish the message to the robot
            geometry_msgs::Twist vel_msg;

            vel_msg.linear.x = linear_wire;   
            vel_msg.angular.z = angular_wire;  
            
            pub.publish(vel_msg);

            // Retrieve data from the topics
            ros::spinOnce();
            rate.sleep();
        }
    }

};
 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_turtlebot2");
    
    ExplorerRobot robot;

    robot.init();
    robot.move();

    return 0;
}

//roslaunch turtlebot2_project1 room_hallway_world.launch
