
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <pthread.h>
#include <cstring>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

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

public:
    Flyer(){

    }

    ~Flyer(){

    }

    Flyer(string name){
        this->uav_name = name;
        srand((unsigned)time(NULL));
    }

    void state_cb(const mavros_msgs::State::ConstPtr& msg){
        current_state = *msg;
    }

    void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
        cur_local_pos = *msg;
    }

    static void* run(void* args){
        Flyer* pThis = ((FlyerBag*)args)->pThis;

        ros::Subscriber state_sub = pThis->nh.subscribe<mavros_msgs::State>
            ("/" + pThis->uav_name + "/mavros/state", 10, &Flyer::state_cb,pThis);
        ros::Publisher set_pos_pub = pThis->nh.advertise<geometry_msgs::PoseStamped>
            ("/" + pThis->uav_name + "/mavros/setpoint_position/local", 10);
        ros::Subscriber local_pos_sub = pThis->nh.subscribe<geometry_msgs::PoseStamped>
            ("/" + pThis->uav_name + "/mavros/local_position/pose",10,&Flyer::local_pos_cb,pThis);
        ros::ServiceClient arming_client = pThis->nh.serviceClient<mavros_msgs::CommandBool>
            ("/" + pThis->uav_name + "/mavros/cmd/arming");
        ros::ServiceClient set_mode_client = pThis->nh.serviceClient<mavros_msgs::SetMode>
            ("/" + pThis->uav_name + "/mavros/set_mode");

        //the setpoint publishing rate MUST be faster than 2Hz
        ros::Rate rate(20.0);

        // wait for FCU connection
        while(ros::ok() && !pThis->current_state.connected){
            ros::spinOnce();
            rate.sleep();
        }

        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = rand() % BORDER;
        pose.pose.position.y = rand() % BORDER;
        pose.pose.position.z = rand() % BORDER;

        //send a few setpoints before starting
        for(int i = 17; ros::ok() && i > 0; --i){
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

            if(  abs(pThis->cur_local_pos.pose.position.x - pose.pose.position.x) < 0.2 &&
                abs(pThis->cur_local_pos.pose.position.y - pose.pose.position.y) < 0.2 && 
                abs(pThis->cur_local_pos.pose.position.z - pose.pose.position.z) < 0.2 &&
                ros::Time::now() - pos_last_req > ros::Duration(1.0) ){
                ROS_INFO("%s change pos", pThis->uav_name.c_str());
                pos_last_req = ros::Time::now();                
                pose.pose.position.x = rand() % BORDER;
                pose.pose.position.y = rand() % BORDER;
                pose.pose.position.z = rand() % BORDER;
            }
            // ROS_INFO("%s %f %f %f ", pThis->uav_name.c_str(),abs(pThis->cur_local_pos.pose.position.x - pose.pose.position.x),
            //     abs(pThis->cur_local_pos.pose.position.y - pose.pose.position.y),abs(pThis->cur_local_pos.pose.position.z - pose.pose.position.z));
            set_pos_pub.publish(pose);

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



int main(int argc, char **argv){
    ros::init(argc, argv, "flyerRandom_node");
    ros::NodeHandle nh;

    int num_uav = 3;

    for(int i = 1;i<=num_uav;i++){
        stringstream ss;
        ss << i;
        string uav_name = "uav" + ss.str();
        Flyer *fi = new Flyer(uav_name);
        fi->excute(nh);
    }    

    pthread_exit(NULL);
    return 0;
}
