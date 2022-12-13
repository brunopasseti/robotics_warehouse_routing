#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <robot_router/TSPAction.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include <iostream>
#include <vector>
#include <iostream>
#include "Graph.h"

double Npoints = 0;
std::vector<std::pair<double, double>> points;

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_fibonacci");

  /*ros::NodeHandle n;
  ros::Subscriber map_sub = n.subscribe("map",2000,mapCallback);
  ros::spin();*/

  
      //pc = * msg;

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<robot_router::TSPAction> ac("tsp", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");

  //map origin at (6,6)
  nav_msgs::OccupancyGrid::ConstPtr msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("map", ros::Duration(10));
  if (msg == NULL)
      ROS_INFO("No map messages received");
  else{
      std_msgs::Header header = msg->header;
      nav_msgs::MapMetaData info = msg->info;
      ROS_INFO("Got map %d %d", info.width, info.height);
      for (unsigned int x = 0; x < info.width; x++)
        for (unsigned int y = 0; y < info.height; y++)
        {
            int index=x+info.width*y;
            if(int(msg->data[index]) == 0){
              points.push_back({x, y});
            }     
        }
  }


  Graph graph(points.size());
  for (size_t i = 0; i < points.size(); i++){
    for (size_t j = 0; j < points.size(); j++)
    {
      if((abs(points[i].first - points[j].first) + abs(points[i].second - points[j].second)) != 1 || i == j)
        continue;
      graph.addEdge(i, j, 1);
    }
  }
  
  vector<vector<double>> matrix;
  vector<vector<vector<int>>> directions;
  for (size_t i = 0; i < points.size(); i++)
  {
    auto response = graph.shortestPath(i);
    matrix.push_back(response.distances);
    directions.push_back(response.directions);
  }
  
  // send a goal to the action
  robot_router::TSPGoal goal;

  //std::cout << "points size:" << points.size() << std::endl;

  for(size_t i = 0; i < matrix.size(); i++){
    for (size_t j = 0; j < matrix[i].size(); j++){
      goal.tagDistances.push_back(matrix[i][j]);
    }
  }

  // goal.tagCoordinates.push_back(coordinate);
  //   std::cout << coordinate << std::endl;
  // coordinate.x = 5.0f;
  // goal.tagCoordinates.push_back(coordinate);
  //   std::cout << coordinate << std::endl;
  // coordinate.y = -3.0f;
  // goal.tagCoordinates.push_back(coordinate);
  //   std::cout << coordinate << std::endl;
  // coordinate.x = -4.0f;
  // goal.tagCoordinates.push_back(coordinate);
  // std::cout << coordinate << std::endl;
  
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult();


  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
    auto result = ac.getResult();

    std::string res = "Result: ";
    for (size_t i = 0; i < result->sequence.size(); i++)
    {
      res = res + std::to_string(result->sequence[i]) + " ";
    }
    
    ROS_INFO(res.c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");


  //exit
  return 0;
}