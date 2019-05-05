/**
*    @file myAvoid.cpp
*    calculate dist and slow down
*/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <pthread.h>
#include <cstring>
#include <stdio.h>
#include <stdlib.h>
using namespace std;

int NUM_UAV = 5;
double[][] dists = new double[NUM_UAV][3];

class Flyer;

struct FlyerBag{
    Flyer* pThis;
};

class Flyer{
    const static int BORDER = 5;
    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped cur_local_pos;
    ros::NodeHandle nh;
    string uav_name;
    int uav_num;

public:
    Flyer(){

    }

    ~Flyer(){

    }

    Flyer(string name,int num){
        this->uav_name = name;
        this->uav_num = num;
        srand((unsigned)time(NULL));
    }

    void state_cb(const mavros_msgs::State::ConstPtr& msg){
        current_state = *msg;
    }

    void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
        //TODO        
        cur_local_pos = *msg;
        dists[uav_num][0] = cur_local_pos.pose.position.x;
        dists[uav_num][1] = cur_local_pos.pose.position.y;
        dists[uav_num][2] = cur_local_pos.pose.position.z;
    }

    static void* run(void* args){
        Flyer* pThis = ((FlyerBag*)args)->pThis;

        ros::Subscriber state_sub = pThis->nh.subscribe<mavros_msgs::State>
            (uav_name + "/mavros/state", 10, &Flyer::state_cb,pThis);
        ros::Subscriber local_pos_sub = pThis->nh.subscribe<geometry_msgs::PoseStamped>
            (uav_name + "/mavros/local_position/pose",10,&Flyer::local_pos_cb,pThis);


        ros::Publisher set_pos_pub = pThis->nh.advertise<geometry_msgs::PoseStamped>
            (uav_name + "/mavros/setpoint_position/local", 10);
        ros::Publisher set_vel_pub = pThis->nh.advertise<geometry_msgs::TwistStamped>
            (uav_name + "/mavros/setpoint_velocity/cmd_vel",10);


        ros::ServiceClient arming_client = pThis->nh.serviceClient<mavros_msgs::CommandBool>
            (uav_name + "/mavros/cmd/arming");
        ros::ServiceClient set_mode_client = pThis->nh.serviceClient<mavros_msgs::SetMode>
            (uav_name + "/mavros/set_mode");

        //the setpoint publishing rate MUST be faster than 2Hz
        ros::Rate rate(20.0);

        // wait for FCU connection
        while(ros::ok() && !pThis->current_state.connected){
            ros::spinOnce();
            rate.sleep();
        }

        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = rand() % BORDER + 1;
        pose.pose.position.y = rand() % BORDER + 1;
        pose.pose.position.z = rand() % BORDER + 1;
        //send a few setpoints before starting
        for(int i = 100; ros::ok() && i > 0; --i){
            set_pos_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }

        mavros_msgs::SetMode offb_set_mode;
        offb_set_mode.request.custom_mode = "OFFBOARD";

        mavros_msgs::CommandBool arm_cmd;
        arm_cmd.request.value = true;

        ros::Time last_request = ros::Time::now();
        ros::Time pos_last_req = ros::Time::now();
        while(ros::ok()){
            if( pThis->current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent){
                    ROS_INFO("Flyer offboard enabled");
                }
                last_request = ros::Time::now();
            } else {
                if( !pThis->current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))){
                    if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success){
                        ROS_INFO("Flyer vehicle armed");
                    }
                    last_request = ros::Time::now();
                }
            }

            if( abs(pThis->cur_local_pos.pose.position.x - pose.pose.position.x) < 0.1 &&
                abs(pThis->cur_local_pos.pose.position.y - pose.pose.position.y) < 0.1 && 
                abs(pThis->cur_local_pos.pose.position.z - HEIGHT) < 0.01 &&
                ros::Time::now() - pos_last_req > ros::Duration(5.0) ){
                ROS_INFO("%s change pos", pThis->uav_name.c_str());
                pos_last_req = ros::Time::now();                
                pose.pose.position.x = rand() % BORDER + 1;
                pose.pose.position.y = rand() % BORDER + 1;
                pose.pose.position.z = rand() % BORDER + 1;
            }

            for(int i = 0; i<NUM_UAV;i++){
                if(i != uav_num && getDist(i,uav_num) < 0.1){
                    geometry_msgs::TwistStamped vel;
                    vel.twist.linear.x = 0;
                    vel.twist.linear.y = 0;
                    vel.twist.linear.z = 0;
                    set_vel_pub.publish(vel);
                }
            }else{
                set_pos_pub.publish(pose);
            }

            ros::spinOnce();
            rate.sleep();
        }
    }

    void excute(ros::NodeHandle nh){
        this->nh = nh;
        pthread_t tid;
        FlyerBag *lb = new FlyerBag();
        lb->pThis = this;        

        int ret = pthread_create(&tid,NULL,Flyer::run,(void *)lb);
        if(ret != 0){
            ROS_INFO("pthread Flyer create error");
        }else{
            ROS_INFO("pthread Flyer create ok");
        }
    }
};


int main(int args, char **argv){
    ros::init(argc,argv,"myAvoid_node");
    ros::NodeHandle nh;

    Flyer* flyer = new Flyer();    
    flyer->excute(nh); 
}