//fly to 1m for 5s then land
#include<iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/String.h>

mavros_msgs::State current_state;
int land_mode = 0;
int start_mode =0;
geometry_msgs::Pose start_pose;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void joyCallBack( const std_msgs::String::ConstPtr& str){ 
	// call back
	if(str->data == "d"||str->data=="D"){
		land_mode = 1;
		}
    else if(str->data == "a"||str->data=="A")
    {
        start_mode = 1;
        //start_pose = local_position.pose;
    }
	std::cout<<"str.data : "<< str->data <<std::endl;
	}

geometry_msgs::PoseStamped local_position;
void position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_position = *msg;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_landing");
    ros::NodeHandle nh;
    ROS_INFO("start landing node\n");

    bool is_reached_height = false;
 	
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber position_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, position_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
	
	ros::Subscriber joy_sub = nh.subscribe("/keys",10,&joyCallBack);	// subscribe
    
    double max_height = 2.0;
    
 
    ros::Rate rate(20.0);
 
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("CONNECTED.");
    geometry_msgs::PoseStamped pose;//姿态控制
    //pose.pose.position.x = 0;
    //pose.pose.position.y = 0;
    pose.pose.position.x = local_position.pose.position.x;
    pose.pose.position.y = local_position.pose.position.y;
    pose.pose.position.z = max_height;
    
    geometry_msgs::TwistStamped vel;//速度控制
 
    for(int i = 10; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        //rate.sleep();
    }
 
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
 
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    
    int land_cnt=0;
 
    ros::Time last_request = ros::Time::now();
 	
 	//起飞
    while(ros::ok())
    {
		if(land_mode == 1)
		{
			ROS_INFO("keyboard interrupt");
			break;
		}

        if( current_state.mode != "OFFBOARD")
        {
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
           	last_request = ros::Time::now();
       	}
        else if( !current_state.armed )
        {
            if(start_mode==0)
            {
                ROS_INFO("wait for start,press a");
                ros::spinOnce();
                rate.sleep();
                continue;
            }
            else if( arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
                pose.pose.position.x = local_position.pose.position.x;
                pose.pose.position.y = local_position.pose.position.y;
                pose.pose.position.z = max_height;
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }
        else if(!is_reached_height&&fabs( local_position.pose.position.z - max_height )<0.15)
        {// judge if reached target height
            is_reached_height = true;
            last_request = ros::Time::now();
        }

        // after reach target height for 5s, land
        else if(  is_reached_height && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            ROS_INFO("tracking landing start");
            break;
        }
		//ROS_INFO("local_pos: x %f, z %f",local_position.pose.position.x,local_position.pose.position.z);
        local_pos_pub.publish(pose);
 
        ros::spinOnce();
        rate.sleep();
    }
   
    
    offb_set_mode.request.custom_mode = "AUTO.LAND";
    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
    {
        ROS_INFO("land enabled");
        last_request = ros::Time::now();
    }
 
    return 0;
}
