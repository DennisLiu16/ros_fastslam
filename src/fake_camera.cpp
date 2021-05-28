#include "ros/ros.h"
#include "math.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Float64MultiArray.h"
#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <time.h>
const double PI = 3.14159265359;

const int YellowNumber = 26;
const int BlueNumber = 14;
//X Y exchange
double coneYellowY[YellowNumber] = { 1,1,1,1,2,2,2,3,3,5,5,7,7,9,9,10,10,10,11,11,11,11,0,12,12,0 };
double coneYellowX[YellowNumber] = { 4,6,8,10,2,7,12,1,13,0,14,0,14,1,13,2,7,12,4,6,8,10,0,0,15,15 };

double coneBlueY[BlueNumber] = { 3,3,4,4,4,4,6,6,8,8,8,8,9,9 };
double coneBlueX[BlueNumber] = { 5,9,3,6,8,11,2,12,3,6,8,11,5,9 };

const double LB = 0.395;
const double bias = 12.0;

const float originY = 15.56;
const float originX = -14.77;

geometry_msgs::Twist carpose;

float changedegree(float degree){
	while(degree>PI || degree<-PI){
		if(degree>PI){
			degree-=2*PI;	
		}
		else if(degree<=-1*PI){
			 degree+=2*PI;
		}
	}
	return degree;
}
double gaussrand()
{
    double mean = 0;
    double std = 0.12;
    double u, v;//uniform distribution
    double x, y;//normal distribution
    u = rand() / (double)RAND_MAX;//RAND_MAX=32767
    v = rand() / (double)RAND_MAX;
    x = sqrt(-2 * log(u)) * cos(2 * M_PI * v) * std + mean;//M_PI=3.14159
    return x;
}
//get robot position
void poseCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    carpose.linear.x = msg->linear.x;
    carpose.linear.y = msg->linear.y;
    carpose.angular.z = changedegree(msg->angular.z); 
    //ROS_INFO_STREAM("x="<<carpose.linear.x<<" y="<<carpose.linear.y<<" theta="<<carpose.angular.z);
}

bool check_if_in_sight(float rx,float ry,float rth,float cx,float cy)
{
	bool check = false;
	if(sqrt(pow((rx-cx),2)+pow((ry-cy),2)) <= 3.0){
		float relate_angle = changedegree(atan2(cy-ry,cx-rx)) - changedegree(rth);
		if(relate_angle<=PI/9 && relate_angle>=-PI/9){     //20degree
			check = true;
		}
	}
	return check;
}

int main(int argc, char **argv)
{
   

    ros::init(argc,argv,"fake_camera");

    ros::NodeHandle n;
    //ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);
    ros::Publisher cone_pub = n.advertise<std_msgs::Float64MultiArray>("coneXP", 1000);
   
    ros::Subscriber pose_sub = n.subscribe("/robot_pose", 1, poseCallback);

    std_msgs::Float64MultiArray sensorXP;
    ros::Rate rate(20);

    //deal with cone coordinate
    for(int i = 0; i < BlueNumber ; i++)
    {
	coneBlueX[i] = (coneBlueX[i] * LB + bias) + originX;
	coneBlueY[i] = -(coneBlueY[i] * LB + bias) + originY;
	//ROS_INFO_STREAM("i="<<i<<" coneBlueX[i]="<<coneBlueX[i]<<" coneBlueY[i]="<<coneBlueY[i]);
    }
    for(int i = 0; i < YellowNumber ; i++)
    {
	coneYellowX[i] = (coneYellowX[i] * LB + bias) + originX;
	coneYellowY[i] = -(coneYellowY[i] * LB + bias) + originY;
	//ROS_INFO_STREAM("i="<<i<<" coneYellowX[i]="<<coneYellowX[i]<<" coneYellowY[i]="<<coneYellowY[i]);
    }
    while(ros::ok()){
    std_msgs::Float64MultiArray sensorXP;
    ROS_INFO_STREAM("x="<<carpose.linear.x<<" y="<<carpose.linear.y<<" theta="<<carpose.angular.z);
	for(int i = 0; i < BlueNumber ; i++){
		float cx = coneBlueX[i];
		float cy = coneBlueY[i];
		float distance = sqrt(pow((carpose.linear.x-cx),2)+pow((carpose.linear.y-cy),2));
		float direction = changedegree(changedegree(atan2(cy-carpose.linear.y,cx-carpose.linear.x))- changedegree(carpose.angular.z));
		float color = 0.0f;
		//ROS_INFO_STREAM("cx="<<cx<<" cy="<<cy);
		if(check_if_in_sight(carpose.linear.x,carpose.linear.y,carpose.angular.z,cx,cy)){
			sensorXP.data.push_back(distance + gaussrand()*distance); // add noise
			sensorXP.data.push_back(direction + gaussrand()); // add noise
			sensorXP.data.push_back(color);
			ROS_INFO_STREAM("cx="<<cx<<" cy="<<cy<<" color="<<color);
			ROS_INFO_STREAM("distance="<<distance<<" direction="<<direction<<" color="<<color);
		}
	}
	for(int i = 0; i < YellowNumber ; i++){
		float cx = coneYellowX[i];
		float cy = coneYellowY[i];
		float distance = sqrt(pow((carpose.linear.x-cx),2)+pow((carpose.linear.y-cy),2));
		float direction = changedegree(changedegree(atan2(cy-carpose.linear.y,cx-carpose.linear.x))- changedegree(carpose.angular.z));
		float color = 1.0f;
		//ROS_INFO_STREAM("cx="<<cx<<" cy="<<cy);
		if(check_if_in_sight(carpose.linear.x,carpose.linear.y,carpose.angular.z,cx,cy)){
			sensorXP.data.push_back(distance + gaussrand()*distance); // add noise
			sensorXP.data.push_back(direction + gaussrand()); // add noise
			sensorXP.data.push_back(color);
			ROS_INFO_STREAM("cx="<<cx<<" cy="<<cy<<" color="<<color);
			ROS_INFO_STREAM("distance="<<distance<<" direction="<<direction<<" color="<<color);
		}
	}

	cone_pub.publish(sensorXP);
	ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
