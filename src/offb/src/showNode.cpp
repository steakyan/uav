#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/StreamRate.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/StatusText.h>
#include <mavros_msgs/VehicleInfo.h>
#include <mavros_msgs/VehicleInfoGet.h>
#include <mavros_msgs/MessageInterval.h>
#include <mavros_msgs/ADSBVehicle.h>

mavros_msgs::ADSBVehicle temp;

void cb(const mavros_msgs::ADSBVehicle::ConstPtr& msg){
    temp = *msg;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe<mavros_msgs::ADSBVehicle>
            ("/mavros/adsb/vehicle",10,&cb);

    ROS_INFO("%f %f %f",temp.latitude,temp.longitude,temp.altitude);
    ROS_INFO("%f %f %f",temp.heading,temp.hor_velocity,temp.ver_velocity);
    ROS_INFO("%f %f", temp.altitude_type,temp.emitter_type);
    
    return 0;
}
