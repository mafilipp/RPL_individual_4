/*
 * main.pp
 *
 *  Created on: Oct 22, 2014
 *      Author: Filippo Martinoni
 *      Note: The core of the node path_planning
 */

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include "map.h"
#include "path_planner.h"
#include "graph_search.h"
#include "A_star_search.h"
#include "Node.h"
#include <queue>
#include <vector>

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
  ros::init(argc, argv, "path_planner");

  // Define variables
  double robotRadius;
  Map gridMap(robotRadius);
  bool PathComputed = false;

  ros::NodeHandle n;

  // Set publisher and subscriber
  ros::Subscriber mapSubscriber = n.subscribe<nav_msgs::OccupancyGrid>("/map", 1000, mapCallback);
  ros::Subscriber mapSubscriberCallback = n.subscribe<nav_msgs::OccupancyGrid>("/map", 1000, &Map::mapCallback, &gridMap);

  ros::Publisher inflatedMapPublisher = n.advertise<nav_msgs::OccupancyGrid>("/inflatedMap",10);
  ros::Publisher pathPublisher = n.advertise<nav_msgs::Path>("/path", 10);


  ros::Rate loop_rate(1);

  // Initialize start, goal, and goal increment
  geometry_msgs::Point Start, Goal;
  geometry_msgs::Point s1, s2, s3, s4;
  int PointToConsider;
  int MethodPoint;

  /////////////////////////////
  // USER DEFINED VARIABLE
  robotRadius = 0.03; //[m]
  MethodPoint = 8; // 4 or 8
  PointToConsider = 2;//

  // through 2 Point (PointToConsider = 2)
  Start.x = 1.8;
  Start.y = 0.2;
  Goal.x  = 0.2;
  Goal.y  = 1.8;

  // through 4 Point (PointToConsider = 4)
  s1.x = 1.8;
  s1.y = 0.2;
  s2.x = 1.8;
  s2.y =  1.8;
  s3.x = 0.2,
  s3.y = 1.8;
  s4.x = 0.2;
  s4.y = 2;

  /////////////////////////////


  while (ros::ok() and not PathComputed)
  {
    ros::spinOnce();

    // Check for condition in order to compute the path
    if(gridMap.isUpToDate() and not gridMap.isAlreadyInflated() and not PathComputed)
    {
    	// Inflate the map
    	gridMap.inflate();
    	// Send the inflated map through /inflatedMap
        inflatedMapPublisher.publish(gridMap.getMap());

        // Generate Astar
        AStarSearch Astar(gridMap, pathPublisher);
        Astar.setMethodPoint(MethodPoint);
        PathComputed = true;

        //////////// USER DEFINED
        if (PointToConsider == 2)
		{
			Astar.findPath(Start, Goal);
			pathPublisher.publish(Astar.computePath());

		}

        if (PointToConsider == 4)
		{
			Astar.findPath(s1, s2);
			pathPublisher.publish(Astar.computePath());
			Astar.findPath(s2, s3);
			pathPublisher.publish(Astar.computePath());
			Astar.findPath(s3, s4);
			pathPublisher.publish(Astar.computePath());
		}
        /////////////////


    }

    // Publish the path
    loop_rate.sleep();
  }


  return 0;
}
