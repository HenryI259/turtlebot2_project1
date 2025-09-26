#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <random>

double pos_x;
double pos_y;

void getPosition(const nav_msgs::Odometry::ConstPtr& msg) {
    pos_x = msg->pose.pose.position.x;
    pos_y = msg->pose.pose.position.y;
}

struct moter_wire_t {
    int suppression;
    int linear_value;
    int angular_value;
};

struct moter_wire_t drive_wire;

void drive_randomly(ros::Publisher pub, ros::Rate rate) {
    double linear_wire;
    double angular_wire;

    double start_x = pos_x;
    double start_y = pos_y;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> distrib(-15, 15);

    while (ros::ok()) {
        linear_wire = 0.2;
        angular_wire = 0;

        if (sqrt((pos_x-start_x)*(pos_x-start_x) + (pos_y-start_y)*(pos_y-start_y)) > 0.3048) {
            angular_wire = distrib(gen);

            start_x = pos_x;
            start_y = pos_y;
        }

        if (drive_wire.suppression) {
            linear_wire = drive_wire.linear_value;
            angular_wire = drive_wire.angular_value;
        }

        geometry_msgs::Twist vel_msg;

        vel_msg.linear.x = linear_wire;   // forward speed
        vel_msg.angular.z = angular_wire;  //turning speed
        
        pub.publish(vel_msg);
        ros::spinOnce();
        rate.sleep();
    }
    
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_turtlebot2");
    ros::NodeHandle nh;

    // Publisher to TurtleBot2 cmd_vel_mux
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, getPosition);


    ros::Rate rate(10); // 10 Hz

    drive_randomly(pub, rate);

    return 0;
}

//roslaunch turtlebot2_project1 room_hallway_world.launch
