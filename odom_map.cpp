#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <geometry_msgs/PointStamped.h>


class Odom_map {
    private:
        // ROS node handle
        ros::NodeHandle nh_;

        ros::Subscriber odometer_sub;
        tf::TransformBroadcaster odometer_br;
        tf::Transform transform;
        tf::Quaternion q;
        nav_msgs::Odometry odom_msg;
        ros::Publisher odometer_pub;
        
        // DATA: 
        double alpha, omega, x, y, theta_k, theta_k1, delta_theta, deg, speed, new_theta;// position in space; alpha -> wheel angle
        double steering_factor = 32;
        double length = 1.765; //m
        double s_k, c_k, s_k1, c_k1; 
        double v_frac_ome;  // speed / omega 
        double dt; 
        ros::Time last_time = ros::Time(0);


    public:
        // Constructor: sets up subscriber, publisher
        Odom_map() {
            x= 0.0;
            y= 0.0;
            theta_k= 90.0;    // the car starts moving in straight line

            // Subscribe to topics with a queue size of 1000
            odometer_sub = nh_.subscribe("/odometry", 1000, &Odom_map::odometerCallback, this);
            
            // Publish the Odometrymsg to the /odom topic
            odometer_pub= nh_.advertise<nav_msgs::Odometry>("/odom", 1000);
        }
    
        // Callback function for the "odom" topic
        void odometerCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {

                // Reading values from the bag
                deg= msg->point.x;   //deg
                speed= msg->point.y; //km/h

                // calcolo delta t
                ros::Time now = msg->header.stamp;
                dt = (now - last_time).toSec();
                last_time = now;
              

                // Conversions
                speed= speed/3.6; // speed (m/s)
                alpha= (deg/steering_factor)/180*M_PI; //wheel angle (rad)
                omega= (speed*tan(alpha))/length;

                s_k = sin(theta_k); 
                c_k = cos(theta_k); 

                v_frac_ome = speed / omega; 

                // Odometry (exact integration + Runge-Kutta for omega too small)                 
                theta_k1 = theta_k + (omega * dt); // theta at k + 1
                s_k1 = sin(theta_k1); 
                c_k1 = cos(theta_k1); 

                if (omega > 0.01){
                     // exact integration
                    x += v_frac_ome * (s_k1 - s_k);
                    y += -v_frac_ome * (c_k1 - c_k);
                } else {    
                    // Runge-Kutta
                    delta_theta = (omega * dt) / 2; 
                    x += speed * dt * cos(theta_k + delta_theta); 
                    y += speed * dt * sin(theta_k + delta_theta);
                }
                theta_k = theta_k1; 

                //TF
                transform.setOrigin(tf::Vector3(x, y, 0)); // scaled to be in km 
                q.setRPY(0,0, theta_k1);
                transform.setRotation(q);

                odometer_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "odom"));

                // ODOMETRY MSG

                odom_msg.header.stamp = ros::Time::now();
                odom_msg.header.frame_id = "odom";
                odom_msg.child_frame_id= "robot"; 
                odom_msg.pose.pose.position.x = x;
                odom_msg.pose.pose.position.y = y; 
                odom_msg.pose.pose.position.z = 0;

                odom_msg.pose.pose.orientation.x = q[0]; 
                odom_msg.pose.pose.orientation.y = q[1];
                odom_msg.pose.pose.orientation.z = q[2];
                odom_msg.pose.pose.orientation.w = q[3];

                odom_msg.twist.twist.linear.x = speed;
                odom_msg.twist.twist.angular.z = omega;

                odometer_pub.publish(odom_msg);
            } 
        };
    
    
    int main(int argc, char **argv) {
        ros::init(argc, argv, "odom_map");
        Odom_map node;
        ros::spin();
        return 0;
    }
