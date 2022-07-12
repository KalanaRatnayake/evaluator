#include <ros/ros.h>
#include <ros/console.h>
#include <ros/callback_queue.h>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <chrono>
#include <memory>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

double x_positive;
double y_positive;
double z_positive;
double x_negative;
double y_negative;
double resolution;
std::string pathname;

//time variables
std::chrono::system_clock::time_point start_time, current_time;
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
std::ofstream graphfile;

//octomap holder
octomap::OcTree* tree = nullptr;

void mapCallback(const octomap_msgs::Octomap::ConstPtr &msg){

    current_time = std::chrono::system_clock::now();

    double spent_sec = ((double) std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count())/1000;

    octomap::AbstractOcTree* tempTree = octomap_msgs::fullMsgToMap(*msg);

    if (tree) delete tree;
    tree = (octomap::OcTree*) tempTree;

    double discovered_count = 0;
    double not_discov_count = 0;

    // explored percentage metric

    for (double z=(0.5*resolution); z<z_positive; z+=resolution)
        for (double x=-1*(x_negative-(0.5*resolution)); x<x_positive; x+=resolution)
            for (double y=-1*(y_negative-(0.5*resolution)); y<y_positive; y+=resolution)
                if (tree->search(x, y, z)) discovered_count++; else not_discov_count++;

    double discov_percentage = (discovered_count*100)/(discovered_count+not_discov_count);
    
    graphfile << spent_sec << "," << discov_percentage << "\n";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "explorationEvaluator");
    ros::NodeHandle node;

    node.param("area/front",                        x_positive,     9.5);
    node.param("area/back",                         x_negative,     0.5);
    node.param("area/left",                         y_positive,     5.5);
    node.param("area/right",                        y_negative,     4.5);
    node.param("area/height",                       z_positive,     0.5);
    node.param("map/resolution",                    resolution,     0.05);
    node.param<std::string>("evaluator/exploration",pathname,       "/home/Desktop/graph_percentage.txt");

    graphfile = std::ofstream(pathname);

    start_time = std::chrono::system_clock::now();

    ros::Subscriber sub_map = node.subscribe("octomap", 1, mapCallback);

    ros::AsyncSpinner spinner (2);
    ros::Rate r(200);

    spinner.start();

    ros::waitForShutdown();
    return 0;
}
