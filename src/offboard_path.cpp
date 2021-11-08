#include<iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>

ros::Publisher marker_pub;
mavros_msgs::State current_state;
int land_mode = 0;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void joyCallBack( const std_msgs::String::ConstPtr& str){ 
	// call back
	if(str->data == "d"||str->data=="D"){
		land_mode = 1;
		}
	std::cout<<"str.data : "<< str->data <<std::endl;
	}

geometry_msgs::PoseStamped local_position;
 // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::SPHERE;
  int x_cor = 0;
void position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_position = *msg;

     visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "basic_shapes";
    marker.id = x_cor;

    marker.type = shape;

    marker.action = visualization_msgs::Marker::ADD;

    marker.pose = local_position.pose;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
// %Tag(SCALE)%
    marker.scale.x =0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
// %EndTag(SCALE)%

    // Set the color -- be sure to set alpha to something non-zero!
// %Tag(COLOR)%
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
// %EndTag(COLOR)%

// %Tag(LIFETIME)%
    marker.lifetime = ros::Duration(5.0);
// %EndTag(LIFETIME)%
    x_cor++;
    marker_pub.publish(marker);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_landing");
    ros::NodeHandle nh;
    ROS_INFO("start landing node\n");

    bool is_reached_height = false;

    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
 	
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

    
    
    double max_height = 1.0;
    
 
    ros::Rate rate(100.0);
 
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
 
    geometry_msgs::PoseStamped pose;//姿态控制
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = max_height;
    
    geometry_msgs::TwistStamped vel;//速度控制
 
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
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
		
        if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
           	last_request = ros::Time::now();
       	}
        else if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if( arming_client.call(arm_cmd) && arm_cmd.response.success)
            {
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
		ROS_INFO("local_pos: x %f, z %f",local_position.pose.position.x,local_position.pose.position.z);
        local_pos_pub.publish(pose);
 
        ros::spinOnce();
        rate.sleep();
    }
    
   //轨迹
   last_request = ros::Time::now();
   double t = 0;
   double scale = 0.05;
    while(ros::ok())
    {
        ros::Duration dtRosTime = ros::Time::now()-last_request;
        t = dtRosTime.toSec();
        t = t/10;  // slowdown the desired traj 2*pi*10
        //t = t/scale;    // if scale < 1 accelrate,else slowdown
		if(land_mode == 1)
		{
			ROS_INFO("keyboard interrupt");
			break;
		}
        // 心型轨迹
        double sint = sin(t);
        pose.pose.position.x = scale*16*sint*sint*sint+0.8;
        pose.pose.position.y = scale*(13*cos(t)-5*cos(2*t)-2*cos(3*t)-cos(4*t))+0.2;
        pose.pose.position.z = max_height;

        //8字
        //double cos2t = cos(2*t);
       // pose.pose.position.x = 0.6*sin(2*t)+0.6;
        //pose.pose.position.y = 0.6*sin(t);
        //pose.pose.position.z = max_height;

        ROS_INFO("local_pos: x %f, z %f",local_position.pose.position.x,local_position.pose.position.z);
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