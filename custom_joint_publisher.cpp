#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Header.h>
#include <vector>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "custom_joint_publisher");//Initializen the ros
    ros::NodeHandle node;//Declare the node handle
    ros::Publisher joint_pub = node.advertise<sensor_msgs::JointState>("joint_states",10);//Decleare a joint state publisher
    sensor_msgs::JointState joint_state;//Define the joint state from sensor messages
    std_msgs::Header header;
    header.frame_id="";
    std::vector<double> angles(4); //the robot has three joints that match with three angles
    
    //std::string name[] = {"joint1", "joint2", "joint3"};
    std::vector<std::string> name(4);
    name[0]="joint1";
    name[1]="joint2";
    name[2]="joint3";
    name[3]="joint4";
    uint32_t seq=0;
    float alpha=3.1416;
    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        header.seq=seq;
        header.stamp=ros::Time::now();
        
        if(alpha<-3.1416){
            alpha=3.1416;
        }
        else{
            alpha=alpha;
        }
        
        angles[0]=alpha;
        angles[1]=alpha;
        angles[2]=alpha;
        angles[3]=alpha;
        
        joint_state.header=header;
        joint_state.position =angles;
        joint_state.name=name;
        joint_pub.publish(joint_state);
        ros::spinOnce();
        loop_rate.sleep();
        ++seq;
        alpha=alpha-0.7854;
    }
    return 0;
}