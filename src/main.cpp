#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>

void laserCallback(const sensor_msgs::LaserScan &msg){
    if (msg.ranges.empty()){
      ROS_WARN_STREAM("URG data is empty.");
      return;
    }
    int posi = msg.ranges.size() / 2;
    double length = msg.ranges.at(posi);
    ROS_INFO_STREAM("ranges[" << posi << "]:" << length);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "laser_position");
    ros::NodeHandle nh;
    ros::Subscriber subscriber = nh.subscribe("scan", 5, laserCallback);
    ros::Rate loop_rate(50);

    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
}