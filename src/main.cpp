/*
 * main.pp
 *
 *  Created on: Oct 22, 2014
 *      Author: Filippo Martinoni
 *      Note: The core of the node path_planning
 */

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "map.h"


using namespace std;



// Callback function for map messages
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
  ROS_INFO("I read a map!");
}

// Main
int main(int argc, char **argv)
{
  // Initialize the node
  ros::init(argc, argv, "particle_filter");

  // Define variables
  double robotRadius = 0.08;
  Map gridMap(robotRadius);
  bool PathComputed = false;

  ros::NodeHandle n;

  // Set publisher and subscriber
  ros::Subscriber mapSubscriber = n.subscribe<nav_msgs::OccupancyGrid>("/map", 1000, mapCallback);
  ros::Subscriber mapSubscriberCallback = n.subscribe<nav_msgs::OccupancyGrid>("/map", 1000, &Map::mapCallback, &gridMap);

  ros::Publisher inflatedMapPublisher = n.advertise<nav_msgs::OccupancyGrid>("/inflatedMap",10);
  //ros::Publisher pathPublisher = n.advertise<nav_msgs::Path>("/path", 10);


  ros::Rate loop_rate(1);


  while (ros::ok())
  {
    ros::spinOnce();

    // Check for condition in order to compute the path
    if(gridMap.isUpToDate() and not gridMap.isAlreadyInflated())
    {
    	// Inflate the map
    	gridMap.inflate();
    	// Send the inflated map through /inflatedMap
        inflatedMapPublisher.publish(gridMap.getMap());
    }

    // Publish the path
    loop_rate.sleep();
  }


  return 0;
}
