#include <ros/ros.h>
#include "std_msgs/String.h"
#include <geometry_msgs/Point.h>
#include <sstream>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int32.h>
#include <vector>
#include "math.h"
#define pi 3.141592653589793

class variables{
    public:
    int global_index;
	void indexCB(const std_msgs::Int32::ConstPtr& msg);
};
void variables::indexCB(const std_msgs::Int32::ConstPtr& msg){
	global_index= msg->data;
}

int main(int argc, char **argv)
{
	variables public_class;
    ros::init(argc, argv, "scada");
    ros::NodeHandle node;
    ros::Publisher joint_pub = node.advertise<sensor_msgs::JointState>("joint_states",10); //topicos
 	ros::Publisher point_pub = node.advertise<geometry_msgs::Point>("position",10);//position es el topico 
	ros::Subscriber index_sub = node.subscribe<std_msgs::Int32>("index",10,&variables::indexCB,&public_class);
    ros::Rate loop_rate(10);
   
    sensor_msgs::JointState joint_state;
	geometry_msgs::Point point_msg;
	std_msgs:: Header header;
	header.frame_id = "";
	std::vector<double> angles(4);
    angles[0]=3.14;
	angles[1]=1.57;
	angles[2]=0;
    angles[3]=0.1;
    uint32_t seq= 0;
	joint_state.name.resize (4);
	joint_state.name[0] ="joint1";
	joint_state.name[1] ="joint2";
	joint_state.name[2] ="joint3";
    joint_state.name[3] ="joint4";
	
    ros::spinOnce();
	double px,py,pz;
	double t1,t2,t3,d2,l1=0.4,l2=0.3,l3=0.2,p=0.1;
	double vectort1[8]={0,pi/2,0,-pi/2,pi/4,pi,0,-3*pi/4};
	double vectort2[8]={0,0,pi/2,0,0,0,0,0};
	double vectort3[8]={0,pi/2,pi/2,-pi/2,pi/2,-pi/2,pi,-pi/2};
    double vectort4[8]={0,0.1,0.2,0.3,0,0.1,0.2,0.3};

    while(1){
 	    ros::spinOnce();
	    header.seq = seq;
	    header.stamp= ros::Time::now(); 
	    joint_state.header = header;
	    joint_state.position=angles;
        public_class.global_index;
	    t1=vectort1[public_class.global_index];
	    t2=vectort2[public_class.global_index];
	    t3=vectort3[public_class.global_index];
        d2=vectort4[public_class.global_index];
	    //ROS_INFO("message:%d",public_class.global_index);

	    angles[0]=t1;
	    angles[1]=t2;
	    angles[2]=t3;
        angles[3]=d2;

	    px= - d2*sin(t2)*cos(t1)*cos(t3) - l2*sin(t1)*sin(t2) + l2*cos(t1)*cos(t2) + l3*(-sin(t1)*sin(t2) + cos(t1)*cos(t2))*cos(t3) + l3*(-sin(t1)*cos(t2) - sin(t2)*cos(t1))*sin(t3) + p*cos(t1);
	    py= d2*sin(t2)*cos(t1)*sin(t3) + l2*sin(t1)*cos(t2) + l2*sin(t2)*cos(t1) + l3*(-sin(t1)*sin(t2) + cos(t1)*cos(t2))*sin(t3) + l3*(sin(t1)*cos(t2) + sin(t2)*cos(t1))*cos(t3) + p*sin(t1);
	    pz= - d2 + l1;
	
	    point_msg.x=px;
	    point_msg.y=py;
	    point_msg.z=pz;
	
	   // ROS_INFO("message:%f",angles[0]);
	    joint_pub.publish(joint_state); 
	    point_pub.publish(point_msg);
	    loop_rate.sleep();
	    seq+=1;
}
    return 0;
}