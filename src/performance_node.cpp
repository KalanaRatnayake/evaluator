#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>

#include <octomap/octomap.h>

#include <chrono>
#include <memory>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

std::string pathname;

//time variables
std::chrono::system_clock::time_point start_time, previ_time, current_time;
bool finised = false;

//path length metric variables
double distance = 0;
octomap::point3d previousPosition = octomap::point3d(0,0,0);

//curvature change variables
double curvature;
double curvature_old = 0;
double total_curvature = 0;

//lateral stress variables
double latStress;
double latStress_old = 0; 
double total_latStress = 0;

//tangential stress variables
double acc;
double acc_old = 0;
double acc_total = 0;

//writing to file
std::ofstream accumfile;

void callback(const nav_msgs::Odometry::ConstPtr &msgOdom, const sensor_msgs::Imu::ConstPtr& msgImu){
	
    //time evaluation
    current_time = std::chrono::system_clock::now();
    double time_sec = ((double) std::chrono::duration_cast<std::chrono::milliseconds>(current_time - previ_time).count())/1000;
    
    previ_time = current_time;

    accumfile << "Time stamp" << " : " << ((double) std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count())/1000 << "\n";

    // path length metric
    octomap::point3d currentPosition = octomap::point3d(msgOdom->pose.pose.position.x, msgOdom->pose.pose.position.y, msgOdom->pose.pose.position.z);
    distance += currentPosition.distance(previousPosition);
    previousPosition = currentPosition;
    
    accumfile << "Path length" << " : " << distance << "\n";

    double lin_vel = msgOdom->twist.twist.linear.x;  //robot can move only in the x direction
    double ang_vel = msgOdom->twist.twist.angular.z; //robot can only change the yaw value.
    double acc_raw = msgImu ->linear_acceleration.x; //robot can only accelerate in x direction

    // lateral stress metric
    latStress = std::abs(ang_vel*lin_vel);
    total_latStress += (latStress + latStress_old)*time_sec*0.5;
    latStress_old = latStress;
    
    accumfile << "Lateral stress" << " : " << total_latStress << "\n";

    // tangential stress metric
    acc = std::abs(acc_raw);
    acc_total += (acc + acc_old)*time_sec*0.5;
    acc_old = acc;
    
    accumfile << "tangential stress" << " : " << acc_total << "\n";
    accumfile << "\n";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "performanceEvaluator");
    ros::NodeHandle node;
    
    node.param<std::string>("evaluator/performance",pathname,       "/home/Desktop/accumilated_values.txt");

    accumfile = std::ofstream(pathname);

    previ_time = std::chrono::system_clock::now();
    start_time = std::chrono::system_clock::now();

    message_filters::Subscriber<sensor_msgs::Imu> imu_sub(node, "imu", 20);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(node, "odometry", 20);

    message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::Imu> sync(odom_sub, imu_sub, 10);
    sync.registerCallback(callback);

    ros::AsyncSpinner spinner (2);
    ros::Rate r(200);

    spinner.start();

    ros::waitForShutdown();
    return 0;
}
