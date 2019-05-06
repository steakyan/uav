#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <pthread.h>
#include <cstring>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cmath>
using namespace std;

class Leader;
class Follower;

const int NUM_UAV = 2;
double dists[NUM_UAV][3];
double home[NUM_UAV][3] = {{0,0,0},{2,0,0}};

double getDist(int i , int j){
    double ret = 0;
    for(int index = 0;index < 3; index++){
        ret += (dists[i][index] - dists[j][index]) * (dists[i][index] - dists[j][index]);
    }
    return sqrt(ret);
}

struct LeaderBag{
    Leader* pThis;
};

struct FollowerBag{
    Follower* pThis;
};

class Leader{
public:

    const static int RADIUS = 5;
    const static int HEIGHT = 2;
    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped cur_local_pos;
    ros::NodeHandle nh;
    string uav_name;
    int uav_num;

    Leader(){

    }

    ~Leader(){

    }

    Leader(string name,int num){
        this->uav_name = name;
        this->uav_num = num;
    }

    void state_cb(const mavros_msgs::State::ConstPtr& msg){
        current_state = *msg;
    }

    void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
        cur_local_pos = *msg;
        dists[uav_num][0] = cur_local_pos.pose.position.x + home[uav_num][0];
        dists[uav_num][1] = cur_local_pos.pose.position.y + home[uav_num][1];
        dists[uav_num][2] = cur_local_pos.pose.position.z + home[uav_num][2];
    }

    static void* run(void* args){
        Leader* pThis = ((LeaderBag*)args)->pThis;

        ros::Subscriber state_sub = pThis->nh.subscribe<mavros_msgs::State>
            (pThis->uav_name + "/mavros/state", 10, &Leader::state_cb,pThis);
        ros::Publisher set_pos_pub = pThis->nh.advertise<geometry_msgs::PoseStamped>
            (pThis->uav_name + "/mavros/setpoint_position/local", 10);
        ros::Subscriber local_pos_sub = pThis->nh.subscribe<geometry_msgs::PoseStamped>
            (pThis->uav_name + "/mavros/local_position/pose",10,&Leader::local_pos_cb,pThis);
        ros::ServiceClient arming_client = pThis->nh.serviceClient<mavros_msgs::CommandBool>
            (pThis->uav_name + "/mavros/cmd/arming");
        ros::ServiceClient set_mode_client = pThis->nh.serviceClient<mavros_msgs::SetMode>
            (pThis->uav_name + "/mavros/set_mode");

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

        ros::Time last_request = ros::Time::now();
        ros::Time pos_last_req = ros::Time::now();
        bool flag = false;
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

            if( abs(pThis->cur_local_pos.pose.position.x - pose.pose.position.x) < 0.2 &&
                abs(pThis->cur_local_pos.pose.position.y - pose.pose.position.y) < 0.2 && 
                abs(pThis->cur_local_pos.pose.position.z - HEIGHT) < 0.2 &&
                ros::Time::now() - pos_last_req > ros::Duration(5.0) && !flag){
                pos_last_req = ros::Time::now();
                index++;
                pose.pose.position.x = xs[index % 4];
                pose.pose.position.y = ys[index % 4];
            }

            for(int i = 0 ;i<NUM_UAV;i++){                   
                if(i != pThis->uav_num && getDist(i , pThis->uav_num) < 1){
                    flag = true;
                    ROS_INFO("too close");
                    break;
                }
            }

            if(flag){                
                pose.pose.position.x = pThis->cur_local_pos.pose.position.x;
                pose.pose.position.y = pThis->cur_local_pos.pose.position.y;
                pose.pose.position.z = pThis->cur_local_pos.pose.position.z + 1;
            }
            set_pos_pub.publish(pose);

            // set_pos_pub.publish(pose);

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
    const static int HEIGHT = 2;
    mavros_msgs::State current_state;
    geometry_msgs::PoseStamped cur_local_pos;
    geometry_msgs::PoseStamped cur_leader_pos;
    ros::NodeHandle nh;
    string uav_name;
    int uav_num;
    Leader* leader;

public:
    Follower(){

    }

    ~Follower(){

    }

    Follower(string name, Leader* leader,int num){
        this->uav_name = name;
        this->leader = leader;
        this->uav_num = num;
    }

    void state_cb(const mavros_msgs::State::ConstPtr& msg){
        current_state = *msg;
    }

    void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
        cur_local_pos = *msg;
        dists[uav_num][0] = cur_local_pos.pose.position.x + home[uav_num][0];
        dists[uav_num][1] = cur_local_pos.pose.position.y + home[uav_num][1];
        dists[uav_num][2] = cur_local_pos.pose.position.z + home[uav_num][2];
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
            (pThis->leader->uav_name + "/mavros/local_position/pose",10,&Follower::leader_pos_cb,pThis);

        //the setpoint publishing rate MUST be faster than 2Hz
        ros::Rate rate(20.0);

        // wait for FCU connection
        while(ros::ok() && !pThis->current_state.connected){
            ros::spinOnce();
            rate.sleep();
        }

        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = pThis->cur_leader_pos.pose.position.x - home[pThis->uav_num][0];
        pose.pose.position.y = pThis->cur_leader_pos.pose.position.y - home[pThis->uav_num][1];
        pose.pose.position.z = pThis->cur_leader_pos.pose.position.z - home[pThis->uav_num][2];

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
        bool flag = false;
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

            for(int i = 0 ;i<NUM_UAV;i++){                   
                if(i != pThis->uav_num && getDist(i , pThis->uav_num) < 1){
                    flag = true;
                    ROS_INFO("too close");
                    break;
                }
            }

            if(flag){                
                pose.pose.position.x = pThis->cur_local_pos.pose.position.x;
                pose.pose.position.y = pThis->cur_local_pos.pose.position.y;
                pose.pose.position.z = pThis->cur_local_pos.pose.position.z + 1;
            }
            else{
                pose.pose.position.x = pThis->cur_leader_pos.pose.position.x - home[pThis->uav_num][0];
                pose.pose.position.y = pThis->cur_leader_pos.pose.position.y - home[pThis->uav_num][1];
                pose.pose.position.z = pThis->cur_leader_pos.pose.position.z - home[pThis->uav_num][2];            
            }
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

    Leader* leader = new Leader("/uav1",0);    
    leader->excute(nh);    

    Follower* follow_one = new Follower("/uav2",leader,1);
    follow_one->excute(nh);

    // Follower* follow_two = new Follower("/uav3",leader,2);
    // follow_two->excute(nh);

    pthread_exit(NULL);
    return 0;
}
