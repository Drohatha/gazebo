#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>


geometry_msgs::PoseStamped roomba_pose; 
auto& position_roomba = roomba_pose.pose.position;



int main(int argc, char **argv){
	ros::init(argc, argv,"roombaRepeater"); 
	ros::NodeHandle n;

	ros::Publisher roomba_pub = n.advertise<geometry_msgs::PoseStamped>("vrpn_client_node/roomba/pose", 100); 

	ros::Rate loop_rate(30); 

	// The motioncapture dont have same coordinate system as the quad
	position_roomba.x = 1.0;  // In quad coor = x
	position_roomba.y = 0.0;  // in quad coor = -z
	position_roomba.z = -1.0;  // In quad coor = -y 

	while(ros::ok()){
		roomba_pose.header.stamp = ros::Time::now(); 
		roomba_pub.publish(roomba_pose); 

		loop_rate.sleep();
	}
}