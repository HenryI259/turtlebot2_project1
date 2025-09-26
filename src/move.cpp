#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "kobuki_msgs/BumperEvent.h"
#include <random>

using namespace std;

// Position
double pos_x;
double pos_y;

// Robot speed recieved from keyboard
double keyboard_linear;
double keyboard_angular;

// Bumper info
int bumper_side;
int bumper_state;

double speed = 0.2;

// Gets the postion
void getPosition(const nav_msgs::Odometry::ConstPtr& msg) {
    pos_x = msg->pose.pose.position.x;
    pos_y = msg->pose.pose.position.y;
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

// move function
void move(ros::Publisher pub, ros::Rate rate) {
    double linear_wire;
    double angular_wire;

    double start_x = pos_x;
    double start_y = pos_y;

    // init random generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib(-15, 15);

    // main loop
    while (ros::ok()) {
        // default movement
        linear_wire = speed;
        angular_wire = 0;

        // robot by random amount after traveling 1 ft
        if (sqrt((pos_x-start_x)*(pos_x-start_x) + (pos_y-start_y)*(pos_y-start_y)) > 0.3048) {
            angular_wire = distrib(gen);

            start_x = pos_x;
            start_y = pos_y;
        }

        //TODO Implement collision avoiding

        // override movement with the keyboard
        if (abs(keyboard_linear) > 0.01 || abs(keyboard_angular) > 0.01) {
            linear_wire = keyboard_linear;
            angular_wire = keyboard_angular;
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
 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_turtlebot2");
    ros::NodeHandle nh;

    // Publisher to TurtleBot2 /mobile_base/commands/velocity
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
    
    // Subscribers
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, getPosition);
    ros::Subscriber teleop_sub = nh.subscribe("/my_teleop_node/cmd_vel", 10, getInputs);
    ros::Subscriber bumper_sub = nh.subscribe("/mobile_base/events/bumper", 10, getBumper);

    ros::Rate rate(10);

    move(pub, rate);

    return 0;
}

//roslaunch turtlebot2_project1 room_hallway_world.launch
