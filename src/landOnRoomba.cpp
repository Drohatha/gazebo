#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <cmath>
#include <control_test_nodes/setpoint_msg_defines.h>

/*Testcase:   */




// Defining constants
constexpr float PI_HALF = 1.570796; 
constexpr float tracking_speed = 2.7; //m/s
constexpr float descend_speed = 0.7; //m/s
constexpr float delta_p = 1.5; // Tuning parameter 

uint16_t velocity_control = IGNORE_PX | IGNORE_PY | IGNORE_PZ |
	                    IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ | 
	                    IGNORE_YAW_RATE;

uint16_t position_control = IGNORE_VX | IGNORE_VY | IGNORE_VZ |
	                    IGNORE_AFX | IGNORE_AFY | IGNORE_AFZ | 
	                    IGNORE_YAW_RATE;



ros::Publisher setpoint_pub; 

mavros_msgs::PositionTarget setpoint;

geometry_msgs::PoseStamped roomba_pose; 
geometry_msgs::PoseStamped quad_pose; 
geometry_msgs::PoseStamped distance_from_target; 
geometry_msgs::TwistStamped roomba_vel; 

auto& position_roomba = roomba_pose.pose.position;
auto& position_quad = quad_pose.pose.position; 
auto& distance = distance_from_target.pose.position; 
auto& velocity_roomba = roomba_vel.twist.linear; 

int state; 

enum{
	takeoff, 
	landOnRoomba
};

void roombaInterception(){

	distance.x = position_roomba.x - position_quad.x; 
	distance.y = position_roomba.y - position_quad.y; 
	distance.z = position_roomba.z - position_quad.z; 

	float inner_product = pow(distance.x,2) + pow(distance.y, 2) + pow(distance.z,2);
	float interception_gain = tracking_speed/sqrt(pow(delta_p,2) + inner_product); 

	setpoint.velocity.x = interception_gain * distance.x + velocity_roomba.x;
	setpoint.velocity.y = interception_gain * distance.y + velocity_roomba.y;
	setpoint.velocity.z = interception_gain * distance.z + velocity_roomba.z; 

	setpoint.yaw = -PI_HALF; 
	setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
	setpoint.header.frame_id = "fcu"; 
	setpoint.type_mask = velocity_control; 

	setpoint_pub.publish(setpoint); 
}

void quadCallback(const geometry_msgs::PoseStamped& input) {
    quad_pose = input;
}

void roombaCallback(const geometry_msgs::PoseStamped& input) {
	geometry_msgs::PoseStamped temp = roomba_pose; 
    
    roomba_pose = input;
    roomba_pose.pose.position.y = -input.pose.position.z;
    roomba_pose.pose.position.z =  input.pose.position.y;


    float time = roomba_pose.header.stamp.toSec() - temp.header.stamp.toSec();  

    //Calculate roomba velocity
    velocity_roomba.x = (position_roomba.x - temp.pose.position.x)/time;
    velocity_roomba.y = (position_roomba.y - temp.pose.position.y)/time;
    velocity_roomba.z = (position_roomba.z - temp.pose.position.z)/time; // This one should be zero and can be removed.	
}

bool closeEnough(float x, float y, float z){
	if(abs(position_quad.x - x) < 0.1 && abs(position_quad.y - y) < 0.1 && abs(position_quad.z - z) < 0.1 ){
		float temp2 = position_quad.z - z; 
		float temp = abs(temp2); 
		ROS_INFO("I am true %f and: %f and abs: %f and z -z: %f", position_quad.z, z, temp, temp2 );  
		return true;
	}else{
		ROS_INFO("I am false %f and: %f ", position_quad.z, z); 
		return false; 
	}
}

void takeOff(float x, float y, float z){
	setpoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
	setpoint.type_mask = position_control;
	setpoint.position.z = 1;
	setpoint.position.x = 0;
	setpoint.position.y = 0;				
	setpoint.header.frame_id = "fcu";
	setpoint.yaw = -PI_HALF;
	setpoint_pub.publish(setpoint); 
}


int main(int argc, char **argv){
	ros::init(argc, argv,"landOnRoomba"); 
	ros::NodeHandle n; 

	ROS_INFO_STREAM("Hello, ROS!"); 

	ros::Subscriber quad_sub            = n.subscribe("/mavros/local_position/pose", 100, quadCallback);
	ros::Subscriber roomba_sub          = n.subscribe("vrpn_client_node/roomba/pose", 100, roombaCallback);
	setpoint_pub 						= n.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100); 

	ros::Rate loop_rate(30); 

	while(ros::ok()){

		switch (state){
			case takeoff:
				takeOff(0,0,1); 
				if(closeEnough(0,0,1)){
					state = landOnRoomba; 
				}
				break; 
			case landOnRoomba:
				roombaInterception();
				if(closeEnough(position_roomba.x, position_roomba.y, position_roomba.z)){
					state = takeoff; 
				}
				break; 
		}


		ros::spinOnce();
		
		loop_rate.sleep();
	}
}