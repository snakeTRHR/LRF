#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<vector>

const double obj_length_max_threshold = 1.5;
const double obj_length_min_threshold = 0.05;

void laserCenterCallback(const sensor_msgs::LaserScan &msg){
    if(msg.ranges.empty()){
      ROS_WARN_STREAM("URG data is empty.");
      return;
    }
    int range_num = msg.ranges.size() / 2;
    double length = msg.ranges.at(range_num);
    ROS_INFO_STREAM("ranges[" << range_num << "]:" << length);
}

std::vector<double> obj_x;
std::vector<double> obj_y;

void ObjectDetectiveCallback(const sensor_msgs::LaserScan &msg){
    obj_x.clear();
    obj_y.clear();
    if(msg.ranges.empty()){
        ROS_WARN_STREAM("URG data is empty.");
        return;
    }
    int j = 0;
    for(int i = 0; i<=1080; ++i){
        if(i%4 == 0){
            double length = msg.ranges.at(i);
            if(length < obj_length_max_threshold && length > obj_length_min_threshold){
                double temp_x = length * cos((i/4 - 135)*M_PI/180);
                double temp_y = length * sin((i/4 - 135)*M_PI/180);
                obj_x.push_back(temp_x);
                obj_y.push_back(temp_y);
            }
            ++j;
        }
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "laser_position");
    ros::NodeHandle nh;
    ros::Subscriber subscriber = nh.subscribe("scan", 5, ObjectDetectiveCallback);
    ros::Rate loop_rate(50);

    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
        for(int i = 0; i < obj_x.size(); ++i){
            ROS_INFO_STREAM("[" << obj_x[i] << ", " << obj_y[i] << "]");
        }
    }
}