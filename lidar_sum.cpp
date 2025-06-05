#include "ros/ros.h"
#include <cmath>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_broadcaster.h>
#include <vector>


class LidarSum {
    private: 
    ros::NodeHandle n;
    ros::Publisher sum_pub;


    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> LidarSyncPolicy;
    message_filters::Subscriber<sensor_msgs::LaserScan> front;
    message_filters::Subscriber<sensor_msgs::LaserScan> rear;
    message_filters::Synchronizer<LidarSyncPolicy> sync;

    public:

    LidarSum() :
    front(n, "/scan_front", 10),
    rear(n, "/scan_back", 10),
    sync(LidarSyncPolicy(10), front, rear) 
    {
        // bind the messages from the two topics
        sync.registerCallback(boost::bind(&LidarSum::LidarCallback, this, _1, _2));
        // publish the new message
        sum_pub= n.advertise<sensor_msgs::LaserScan>("/sum_scan", 1000);
    }

    void LidarCallback(const sensor_msgs::LaserScan::ConstPtr& front_scan_msg,
                       const sensor_msgs::LaserScan::ConstPtr& rear_scan_msg) {
        float remove_ranges= 0.54-0.3; // robot length - x_tf_static position of lidar
        int idx_lim= 135; // overlapping angle of view/resolution (ca 45/0.33 deg)
        std::vector<float> intensities;
        std::vector<float> ranges;
        int msg_dim= front_scan_msg->ranges.size();
        int dim =  msg_dim*2 - 2 * idx_lim;
        intensities.resize(dim);
        ranges.resize(dim);

        // concatenate the two messages, then remove robot points

        // first part where overlapping is verified
        for (int i=0; i< idx_lim; i++) {

            if (front_scan_msg->intensities[i] > rear_scan_msg->intensities[msg_dim - i]) {
                intensities[i]= front_scan_msg->intensities[i];
                ranges[i]= front_scan_msg->ranges[i];
            } 
            else {
                intensities[i]= rear_scan_msg->intensities[msg_dim - i];
                ranges[i]= rear_scan_msg->ranges[msg_dim - i];
            }
        }

        // forward front
        for (int i= idx_lim; i< msg_dim - idx_lim; i++) {
            intensities[i]= front_scan_msg->intensities[i];
            ranges[i]= front_scan_msg->ranges[i];
        }

        // second part where overlapping is verified
        for (int i= msg_dim - idx_lim; i< msg_dim; i++) {

            if (front_scan_msg->intensities[i] > rear_scan_msg->intensities[msg_dim - i]) {
                intensities[i]= front_scan_msg->intensities[i];
                ranges[i]= front_scan_msg->ranges[i];
            } 
            else {
                intensities[i]= rear_scan_msg->intensities[msg_dim - i];
                ranges[i]= rear_scan_msg->ranges[msg_dim - i];
            } 
        }

        // forward rear
        for (int i= msg_dim; i< msg_dim - 2*idx_lim; i++) { 
            intensities[i]= rear_scan_msg->intensities[i + idx_lim];
            ranges[i]= rear_scan_msg->ranges[i + idx_lim];
        }

        // remove robot points
        for (int i=0; i<dim; i++) {
            if (ranges[i] < remove_ranges) {
                intensities[i]= 0.0;
                ranges[i]= front_scan_msg->range_max;
            }
        }

        // fill the new message
        sensor_msgs::LaserScan sum_scan;
        sum_scan.header.stamp = front_scan_msg->header.stamp;
        sum_scan.header.frame_id = "sum_scan";
        sum_scan.angle_min = -M_PI;
        sum_scan.angle_max = M_PI;

        //risoluzione costante (sia nel front che nel rear lidar)
        sum_scan.angle_increment = front_scan_msg->angle_increment;
        sum_scan.time_increment = front_scan_msg->time_increment;
        //scan time costante (sia nel front che nel rear lidar)
        sum_scan.scan_time = front_scan_msg->scan_time;
        //ranges identico dei due lidars
        sum_scan.range_min = front_scan_msg->range_min;
        sum_scan.range_max = front_scan_msg->range_max;

        sum_scan.ranges= ranges;
        sum_scan.intensities= intensities;

        // publish the message
        sum_pub.publish(sum_scan); 


    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_sum");
    LidarSum node;
    ros::spin();
    return 0;
}