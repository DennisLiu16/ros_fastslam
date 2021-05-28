#include "ros/ros.h"
#include "math.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <time.h>

const double PI = 3.14159265359;
float changedegree(float degree){
	while(degree>PI || degree<-PI){
		if(degree>PI){
			degree-=2*PI;	
		}
		else if(degree<-1*PI){
			 degree+=2*PI;
		}
	}
	return degree;
}
double gaussrand()
{
    double mean = 0;
    double std = 0.08;
    double u, v;//uniform distribution
    double x, y;//normal distribution
    u = rand() / (double)RAND_MAX;//RAND_MAX=32767
    v = rand() / (double)RAND_MAX;
    x = sqrt(-2 * log(u)) * cos(2 * M_PI * v) * std + mean;//M_PI=3.14159
    return x;
}
float X = 0.0;
float Y = 0.0;
float THETA = 0.0;
int rate_ = 20;
float del_t = 0.05;

geometry_msgs::Twist carpose;
geometry_msgs::Twist odom;

//get robot position
void poseCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    carpose.linear.x = msg->linear.x;
    carpose.linear.y = msg->linear.y;
    carpose.angular.z = changedegree(msg->angular.z);
    
    //ROS_INFO_STREAM("x="<<carpose.linear.x<<" y="<<carpose.linear.y<<" theta="<<carpose.angular.z);
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odom.linear.x = msg->twist.twist.linear.x + gaussrand(); // add noise
    odom.angular.z = msg->twist.twist.angular.z + gaussrand(); // add noise
     
    //ROS_INFO_STREAM("x="<<carpose.linear.x<<" y="<<carpose.linear.y<<" theta="<<carpose.angular.z);
}

int main(int argc, char **argv)
{
   

    ros::init(argc,argv,"fake_odom");

    ros::NodeHandle n;

   
    ros::Subscriber pose_sub = n.subscribe("/robot_pose", 1, poseCallback);
    ros::Subscriber odom_sub = n.subscribe("odom", 1000, odomCallback);
    ros::Publisher pub = n.advertise<geometry_msgs::Twist>("FeedBack_Vel", 1000);

    ros::Rate rate(rate_);
    odom.linear.x = 0.0f;
    odom.linear.y = 0.0f;
    odom.linear.z = 0.0f;
    odom.angular.x = 0.0f;
    odom.angular.y = 0.0f;
    odom.angular.z = 0.0f;
	
    while(ros::ok()){

	X = X + odom.linear.x * del_t * cos(THETA + odom.angular.z * del_t);
	Y = Y + odom.linear.x * del_t * sin(THETA + odom.angular.z * del_t);
	THETA = changedegree(THETA + odom.angular.z * del_t);
	//ROS_INFO_STREAM("cmd_v = "<<cmd_v<<" cmd_w = "<<cmd_w);
	//ROS_INFO_STREAM("real_v = "<<velocity.linear.x<<" real_w = "<<velocity.angular.z);
	//ROS_INFO_STREAM("cmd_R = "<<cmd_R<<" cmd_L = "<<cmd_L);
	//ROS_INFO_STREAM("real_R = "<<real_R<<" real_L = "<<real_L);
	ROS_INFO_STREAM("real_X = "<<carpose.linear.x<<" real_Y = "<<carpose.linear.y<<" real_THETA = "<<carpose.angular.z);
    	ROS_INFO_STREAM("X = "<<X<<" Y = "<<Y<<" THETA = "<<THETA);
   	//ROS_INFO_STREAM("T = "<<del_t);
	pub.publish(odom);
	ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
