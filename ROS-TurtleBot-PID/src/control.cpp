
/*
Used for teaching controller design for Turtlebot3
Lantao Liu
ISE, Indiana University
*/

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/Empty.h>
#include <sstream>
#include "geometry.h"
#include "pid.h"

#include <cmath>
#include "Potential.h"
#include "Util.h"
#include "tb3_control/CoordinateGoal.h"

// global vars
tf::Point odom_pos;    // odometry position (x, y, z)
double odom_yaw;    	 // odometry orientation (yaw)
double odom_v, odom_w; // odometry linear and angular speeds

// ROS Topic Publishers
ros::Publisher cmd_vel_pub;
ros::Publisher marker_pub;

// ROS Topic Subscribers
ros::Subscriber odom_sub;
ros::Subscriber tsp_sub;

double maxSpeed = 0.5;
double distanceConst = 0.5;
double dt = 0.1, maxT = M_PI, minT = -M_PI, Kp = 0.3, Ki = 0.05, Kd = 0.01;
double dtS = 0.1, maxS = maxSpeed, minS = 0.0, KpS = 0.08, KiS = 0.01, KdS = 0.005;


double getDistance(Point &p1, Point &p2);

void odomCallback(const nav_msgs::Odometry odom_msg) 
{
    tf::pointMsgToTF(odom_msg.pose.pose.position, odom_pos);
    odom_yaw = tf::getYaw(odom_msg.pose.pose.orientation);

    //update observed linear and angular speeds (real speeds published from simulation)
    odom_v = odom_msg.twist.twist.linear.x;
    odom_w = odom_msg.twist.twist.angular.z;

    //display on terminal screen
    //ROS_INFO("Position: (%f, %f); Yaw: %f", odom_pos.x(), odom_pos.y(), odom_yaw);
}

void tspCallback(const tb3_control::CoordinateGoal tsp_msg){

}

int main(int argc, char **argv) {

    ros::init(argc, argv, "control");
    ros::NodeHandle n("~");
    tf::TransformListener m_listener;
    tf::StampedTransform transform;

    cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    odom_sub = n.subscribe("odom", 10, odomCallback);
    tsp_sub = n.subscribe("tsp/output/goals", 10, tspCallback);
    ros::Rate loop_rate(10); // ros spins 10 frames per second

    //we use geometry_msgs::twist to specify linear and angular speeds (v, w) which also denote our control inputs to pass to turtlebot
    geometry_msgs::Twist tw_msg;


    //trajectory details are here
    Geometry geometry;

    double angleError = 0.0;
    double speedError = 0.0;

    int frame_count = 0;
    PID pidTheta = PID(dt, maxT, minT, Kp, Kd, Ki);
    PID pidVelocity = PID(dtS, maxS, minS, KpS, KdS, KiS);

		double goal_x = 4;
		double goal_y = 3;

		double goal_yaw = ComputeGoalYaw(3.2, 1.7, goal_x, goal_y) + M_PI / 2;
		ROS_INFO("%f", goal_yaw);
    
		// tw_msg.angular.z = 0.5 * (Odom_yaw - 3.14/4);
    while (ros::ok()) {

        
				double secs =ros::Time::now().toSec();

				int control_ret = ControlVelocities(
						tw_msg.angular.z, 
						tw_msg.linear.x, 
						goal_yaw, 
						odom_yaw, 
						odom_pos.x(), 
						odom_pos.y(),
						goal_x,
						goal_y
						);

        ROS_INFO("[%i] w [%f], x [%f], y [%f]", control_ret, odom_yaw, odom_pos.x(), odom_pos.y());

        cmd_vel_pub.publish(tw_msg);

        frame_count++;
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;

}


/*
 * This function calculate euclidean distance between two given points
 * Remove sqrt if performance hungry
 **/
double getDistance(Point &p1, Point &p2) {
    return sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2));
}


