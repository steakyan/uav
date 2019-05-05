/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
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

class Leader;
class Follower;

struct LeaderBag{
    Leader* pThis;
};

struct FollowerBag{
    Follower* pThis;
};

class Leader{
    const static int RADIUS = 10;
    const static int HEIGHT = 2;
    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped cur_local_pos;
    ros::NodeHandle nh;

public:
    void state_cb(const mavros_msgs::State::ConstPtr& msg){
        current_state = *msg;
    }

    void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
        cur_local_pos = *msg;
    }

    static void* run(void* args){
        Leader* pThis = ((LeaderBag*)args)->pThis;

        ros::Subscriber state_sub = pThis->nh.subscribe<mavros_msgs::State>
            ("/uav1/mavros/state", 10, &Leader::state_cb,pThis);
        ros::Publisher set_pos_pub = pThis->nh.advertise<geometry_msgs::PoseStamped>
            ("/uav1/mavros/setpoint_position/local", 10);
        ros::Subscriber local_pos_sub = pThis->nh.subscribe<geometry_msgs::PoseStamped>
            ("/uav1/mavros/local_position/pose",10,&Leader::local_pos_cb,pThis);
        ros::ServiceClient arming_client = pThis->nh.serviceClient<mavros_msgs::CommandBool>
            ("/uav1/mavros/cmd/arming");
        ros::ServiceClient set_mode_client = pThis->nh.serviceClient<mavros_msgs::SetMode>
            ("/uav1/mavros/set_mode");

        //the setpoint publishing rate MUST be faster than 2Hz
        ros::Rate rate(20.0);

        // wait for FCU connection
        while(ros::ok() && !pThis->current_state.connected){
            ros::spinOnce();
            rate.sleep();
        }

        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = RADIUS;
        pose.pose.position.y = RADIUS;
        pose.pose.position.z = HEIGHT;
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

        int xs[] = {4,0,-4,0};
        int ys[] = {0,4,0,-4};
        int index = 0;
        int radians = 0;

        ros::Time last_request = ros::Time::now();
        ros::Time pos_last_req = ros::Time::now();
        while(ros::ok()){
            if( pThis->current_state.mode != "OFFBOARD" &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent){
                    ROS_INFO("leader offboard enabled");
                }
                last_request = ros::Time::now();
            } else {
                if( !pThis->current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))){
                    if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success){
                        ROS_INFO("leader vehicle armed");
                    }
                    last_request = ros::Time::now();
                }
            }

            if( abs(pThis->cur_local_pos.pose.position.x - pose.pose.position.x) < 0.1 &&
                abs(pThis->cur_local_pos.pose.position.y - pose.pose.position.y) < 0.1 && 
                abs(pThis->cur_local_pos.pose.position.z - HEIGHT) < 0.01 &&
                ros::Time::now() - pos_last_req > ros::Duration(5.0) ){
                //ROS_INFO("\npos_cur : %f %f\npos_before: %f %f",cur_local_pos.pose.position.x,cur_local_pos.pose.position.y,pose.pose.position.x,pose.pose.position.y);
                pos_last_req = ros::Time::now();
                // ROS_INFO("reached %d",index);
                index++;
                pose.pose.position.x = xs[index % 4];
                pose.pose.position.y = ys[index % 4];
                // pose.pose.position.x = cos(radians) * 10;
                // pose.pose.position.y = sin(radians) * 10;
                radians += 5;
            }
            set_pos_pub.publish(pose);

            ros::spinOnce();
            rate.sleep();
        }
    }

    void excute(ros::NodeHandle nh){
        this->nh = nh;
        pthread_t tid;
        LeaderBag *lb = new LeaderBag();
        lb->pThis = this;        

        int ret = pthread_create(&tid,NULL,Leader::run,(void *)lb);
        if(ret != 0){
            ROS_INFO("pthread leader create error");
        }else{
            ROS_INFO("pthread leader create ok");
        }
    }
};

class Follower{
    const static int RADIUS = 5;
    const static int HEIGHT = 5;
    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped cur_local_pos;
    geometry_msgs::PoseStamped cur_leader_pos;
    ros::NodeHandle nh;
    string uav_name;

public:
    Follower(){

    }

    ~Follower(){

    }

    Follower(string name){
        this->uav_name = name;
    }

    void state_cb(const mavros_msgs::State::ConstPtr& msg){
        current_state = *msg;
    }

    void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
        cur_local_pos = *msg;
    }

    void leader_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
        cur_leader_pos = *msg;
    }

    static void* run(void* args){
        Follower* pThis = ((FollowerBag*)args)->pThis;

        ros::Subscriber state_sub = pThis->nh.subscribe<mavros_msgs::State>
            (pThis->uav_name + "/mavros/state", 10, &Follower::state_cb,pThis);
        ros::Publisher set_pos_pub = pThis->nh.advertise<geometry_msgs::PoseStamped>
            (pThis->uav_name + "/mavros/setpoint_position/local", 10);
        ros::Subscriber local_pos_sub = pThis->nh.subscribe<geometry_msgs::PoseStamped>
            (pThis->uav_name + "/mavros/local_position/pose",10,&Follower::local_pos_cb,pThis);
        ros::ServiceClient arming_client = pThis->nh.serviceClient<mavros_msgs::CommandBool>
            (pThis->uav_name + "/mavros/cmd/arming");
        ros::ServiceClient set_mode_client = pThis->nh.serviceClient<mavros_msgs::SetMode>
            (pThis->uav_name + "/mavros/set_mode");
        ros::Subscriber leader_pos_sub = pThis->nh.subscribe<geometry_msgs::PoseStamped>
            ("/uav1/mavros/local_position/pose",10,&Follower::leader_pos_cb,pThis);

        //the setpoint publishing rate MUST be faster than 2Hz
        ros::Rate rate(20.0);

        // wait for FCU connection
        while(ros::ok() && !pThis->current_state.connected){
            ros::spinOnce();
            rate.sleep();
        }

        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = pThis->cur_leader_pos.pose.position.x;
        pose.pose.position.y = pThis->cur_leader_pos.pose.position.y;
        pose.pose.position.z = pThis->cur_leader_pos.pose.position.z;

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
                    ROS_INFO("follow offboard enabled");
                }
                last_request = ros::Time::now();
            } else {
                if( !pThis->current_state.armed &&
                    (ros::Time::now() - last_request > ros::Duration(5.0))){
                    if( arming_client.call(arm_cmd) &&
                        arm_cmd.response.success){
                        ROS_INFO("follow vehicle armed");
                    }
                    last_request = ros::Time::now();
                }
            }


            pose.pose.position.x = pThis->cur_leader_pos.pose.position.x;
            pose.pose.position.y = pThis->cur_leader_pos.pose.position.y;
            pose.pose.position.z = pThis->cur_leader_pos.pose.position.z;
            set_pos_pub.publish(pose);

            ros::spinOnce();
            rate.sleep();
        }
    }

    void excute(ros::NodeHandle nh){
        this->nh = nh;
        pthread_t tid;
        FollowerBag *lb = new FollowerBag();
        lb->pThis = this;        

        int ret = pthread_create(&tid,NULL,Follower::run,(void *)lb);
        if(ret != 0){
            ROS_INFO("pthread follow create error");
        }else{
            ROS_INFO("pthread follow create ok");
        }
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    Leader* leader = new Leader();    
    leader->excute(nh);    

    Follower* follow_one = new Follower("/uav2/");
    follow_one->excute(nh);

    Follower* follow_two = new Follower("/uav3/");
    follow_two->excute(nh);

    pthread_exit(NULL);
    return 0;
}
