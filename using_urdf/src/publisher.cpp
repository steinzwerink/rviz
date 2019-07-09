#include "using_urdf/publisher.h"
#include <std_msgs/String.h>

publisher::Publisher::Publisher()
{
}

publisher::Publisher::~Publisher()
{
}

std::string publisher::Publisher::convertMessage(pose::pose p)
{
    std::string l;
    l = p.pose_name;
    return l;
}

int main(int argc, char **argv)
{

    publisher::Publisher pub;
    ros::init(argc, argv, "Publisher");
    ros::NodeHandle n;
    ros::Publisher pose_pub =
        n.advertise<std_msgs::String>("/robot_control", 1000);

    ros::Rate loop_rate(1);
    int count = 0;

    pose::pose p1("#2 P1150 #3 P2500 #4 P1900 T2000 ", 2);
    pose::pose p2("#6 P1800 #7 P1800 T1000 ", 1);
    pose::pose p3("#2 P1350 #3 P1500 #4 P1100 T12000 ", 12);
    pose::pose p4("#6 P1750 #7 P1750 T1000 ", 1);
    pose::pose p5("#1 P1000 T1000 ", 1);
    pose::pose p6("#2 P1150 #3 P2500 #4 P1900 T2000 ", 2);
    pose::pose p7("#1 P1000 #2 P1150 #3 P2500 #4 P1900 T2000 ", 2);
    pose::pose p8("#1 P1600 #2 P1150 #3 P2500 #4 P1900 T15000 ", 15);
    pose::pose p9("#1 P1500 #2 P1150 #3 P2500 #4 P1900 T2000 ", 2);
    pose::pose p10("#1 P1500 #2 P1350 #3 P2500 #4 P1900 T2000 ", 2);
    pose::pose p11("#1 P2000 #2 P1350 #3 P2500 #4 P1900 T2000 ", 2);
    pose::pose p12("#1 P2000 #2 P1150 #3 P2500 #4 P1900 T2000 ", 2);
    pose::pose p13("#1 P1530 #2 P1150 #3 P2500 #4 P1900 T15000 ", 15);
    pose::pose p14("#1 P2000 T1000 ", 1);
    pose::pose p15("#2 P1500 #3 P2100 #4 P1500 T2000 ", 2);
    pose::pose p16("#1 P1500 T1000 ", 1);
    pose::pose p17("#6 P500 #7 P500 T1000 ", 1);

    std::vector<pose::pose> movement = {p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15, p16, p17};
    ros::Duration(6).sleep();
    while (ros::ok())
    {
        for (const auto &p : movement)
        {

            std_msgs::String msg;
            msg.data = p.pose_name;
            pose_pub.publish(msg);
            ros::Duration(p.duration).sleep();
            ros::spinOnce();

            loop_rate.sleep();
            ++count;
        }
        //break;
    }

    return 0;
}
