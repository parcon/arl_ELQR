/*
    Daman Bareiss
    Algorithmic Robotics Lab @ University of Utah
    
    This node runs the turtle bot for the extended LQR experiments.
*/

#include <ros/ros.h>
#include "tf/transform_listener.h"
#include <geometry_msgs/Vector3.h>
#include "geometry_msgs/TransformStamped.h"
#include "ros/time.h"
#include <std_msgs/Int16.h>
#include <Eigen/Dense>
#include <sensor_msgs/Joy.h>

#define yaw 0
#define pitch 1
#define roll 2
#define thrust 3

#define u_dim 2
#define x_dim 3

using namespace Eigen;

// LQR matrices
std::vector<MatrixXf> Lt; 	// FROM JUR
std::vector<VectorXf> ellt;	// FROM JUR

// State vector
VectorXf x(x_dim);

// Control input
VectorXf u(u_dim);

// Mo-Cap position
geometry_msgs::Vector3 curr_pos;

// Published u value
std_msgs::Int16 left_dummy, right_dummy;
std::vector<int> u_out;

// Joystick stuff for test drives
double joy_x_,joy_y_;
double joy_x,joy_y;

// Joystick callback
void joy_callback(const sensor_msgs::Joy& joy_msg_in)
{
	joy_x_ = joy_msg_in.axes[1];
	joy_y_ = joy_msg_in.axes[0];
}

// Merge new joystick messages
void merge_new_msgs(void)
{
	joy_x = joy_x_;
	joy_y = joy_y_;
}

// VERY simple tank mixer
std::vector<int> mixer(double joy_x, double joy_y)
{
	double max = 500.0;
	double min = -500.0;
	double left, right;
	double joy_dead = 0.1;
	if (fabs(joy_x)<joy_dead) {joy_x =0;}
	if (fabs(joy_y)<joy_dead) {joy_y =0;}
	left = 500.0/1024.0*joy_x + 100.0/1024.0*joy_y;
	right = 500.0/1024.0*joy_x - 100.0/1024.0*joy_y;
	if (left > max) {left = max;}
	if (left < min) {left = min;}
	if (right > max) {right = max;}
	if (right < min) {right = min;}
	std::vector<int> out(2);
	out[0] = (int) left;
	out[1] = (int) right;
	return out;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "eLQR");
	ros::NodeHandle node;
	
	ros::Subscriber joy_sub;
	ros::Publisher left_pub = node.advertise<std_msgs::Int16>("left",1);
	ros::Publisher right_pub = node.advertise<std_msgs::Int16>("right",1);
	
	joy_sub = node.subscribe("joy",1,joy_callback);
	
	tf::TransformListener listener;
	tf::StampedTransform stamped;
	
	ros::Rate loop_rate(4);
	
	int t = 0;
	
	while(ros::ok())
	{
		
		// Merge joystick msgs
		merge_new_msgs();
		
		// Get mo-cap information
        	listener.lookupTransform("/robot", "/optitrak",  ros::Time(0), stamped);
		curr_pos.x = stamped.getOrigin().getX();
		curr_pos.y = stamped.getOrigin().getY();
		curr_pos.z = stamped.getOrigin().getZ();
		double euler_angles[3];
		btMatrix3x3(stamped.getRotation()).getRPY(euler_angles[roll], euler_angles[pitch], euler_angles[yaw]);

		// Send joystick information
		u_out = mixer(joy_x,joy_y);
		
		// Send u to driver some-how, for now std::vector of in16
		left_dummy.data = u_out[0];
		right_dummy.data = u_out[1];
		left_pub.publish(left_dummy);
		right_pub.publish(right_dummy);
		
		// Send/Receive ROS topics
		ros::spinOnce();
		
		/* Use LQR, need in loop to match up to time-steps
		x[0] = curr_pos.x;
		x[1] = curr_pos.y;
		x[2] = euler_angles[yaw]; // have to get yaw from positive x-axis in world
		u = Lt[t]*x + ellt[t];
		u_out[0] = u[0];
		u_out[1] = u[1];
		u_pub.publish(u_out);
		t += 1;
		ros::spinOnce();
		loop_rate.sleep();
		*/
	}
}
		
		
			
