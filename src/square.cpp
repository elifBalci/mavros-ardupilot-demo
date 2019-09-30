#include <cstdlib>
#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointPush.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <stdlib.h>

//Current position of the drone
double x_position;
double y_position;
double z_position;

void position_call_back(const geometry_msgs::PoseStamped::ConstPtr &msg){
    geometry_msgs::PoseStamped position = *msg;
    x_position = position.pose.position.x;
    y_position = position.pose.position.y;
    z_position = position.pose.position.z;
}

bool is_greater_then_threshold(geometry_msgs::PoseStamped destination){
	//Is drone close enough to the target
    double distance = sqrt(pow(destination.pose.position.x - x_position, 2) + pow(destination.pose.position.y - y_position, 2));
    return distance> 0.8; //0.8 is just a threshold
}

//function that make drone move by publishing destination.
void travel(geometry_msgs::PoseStamped destination, ros::Publisher local_pos_pub, ros::Publisher next_pos_pub  ){
    while(is_greater_then_threshold(destination)){
        next_pos_pub.publish(destination.pose.position);
        local_pos_pub.publish(destination);
        sleep(1);
    }
    return;
}

int main(int argc, char **argv){
    int rate = 20;
    ros::init(argc, argv, "mavros_takeoff");
    ros::NodeHandle n;
    ros::Rate r(rate);
    //start callbacks
    ros::AsyncSpinner spinner(1);
    spinner.start();

    /*********** GUIDED ***********/
    ros::ServiceClient cl = n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    mavros_msgs::SetMode srv_setMode;
    srv_setMode.request.base_mode = 0;
    srv_setMode.request.custom_mode = "GUIDED";
    if(cl.call(srv_setMode)){
        ROS_INFO("setmode send ok %d value:", srv_setMode.response.mode_sent);
    }else{
        ROS_ERROR("Failed SetMode");
        return -1;
    }

    /***********ARM***************/
    ros::ServiceClient arming_cl = n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    mavros_msgs::CommandBool srv;
    srv.request.value = true;
    if(arming_cl.call(srv)){
        ROS_INFO("ARM send ok %d", srv.response.success);
    }else{
        ROS_ERROR("Failed arming or disarming");
    }

    /**********TAKE OFF***********/
    ros::ServiceClient takeoff_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
    mavros_msgs::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = 10;
    srv_takeoff.request.latitude = 0;
    srv_takeoff.request.longitude = 0;
    srv_takeoff.request.min_pitch = 0;
    srv_takeoff.request.yaw = 0;
    if(takeoff_cl.call(srv_takeoff)){
        ROS_INFO("srv_takeoff send ok %d", srv_takeoff.response.success);
    }else{
        ROS_ERROR("Failed Takeoff");
    }

    /************TASK****************/
    sleep(10);
    ros::Publisher local_pos_pub = n.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1000);
    ros::Publisher next_pos_pub = n.advertise<geometry_msgs::Point>("/drone/next_waypoint", 1000);
    ros::Subscriber position_sub = n.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, position_call_back);
    
    //Declare destination, change it when reached.
    geometry_msgs::PoseStamped destination;
    destination.pose.position.x = 3;
    destination.pose.position.y = 0;
    destination.pose.position.z = 10;
    travel(destination, local_pos_pub, next_pos_pub);
	//second point of the square 
    destination.pose.position.x = 3;
    destination.pose.position.y = 3;
    destination.pose.position.z = 10;
    travel(destination, local_pos_pub, next_pos_pub);
    //third point of the square
    destination.pose.position.x = 0;
    destination.pose.position.y = 3;
    destination.pose.position.z = 10;
    travel(destination, local_pos_pub, next_pos_pub);
    //final point 
    destination.pose.position.x = 0;
    destination.pose.position.y = 0;
    destination.pose.position.z = 10;
    travel(destination, local_pos_pub, next_pos_pub);
    sleep(3);

    /***********LAND***********/
    ros::ServiceClient land_cl = n.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    mavros_msgs::CommandTOL srv_land;
    srv_land.request.altitude = 10;
    srv_land.request.latitude = 0;
    srv_land.request.longitude = 0;
    srv_land.request.min_pitch = 0;
    srv_land.request.yaw = 0;
    if(land_cl.call(srv_land)){
        ROS_INFO("srv_land send ok %d", srv_land.response.success);
    }else{
        ROS_ERROR("Failed Land");
    }

    return 0;
}

