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
#include <tb3_control/CoordinateGoal.h>

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


  ros::init(argc, argv, "publisher_control");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<tb3_control::CoordinateGoal>("tb3_control_node/output/goals", 1);
  ros::Rate loop_rate(0.5);
  
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
  
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult();

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
    auto result = ac.getResult();

    tb3_control::CoordinateGoal goals;
    int previous = -1;
    for (int index : result->sequence){
      if(previous == -1){
        previous = index;
        continue;
      }

      for (size_t i = 0; i < directions[previous][index].size(); i++){
        geometry_msgs::Point32 coordinate;
        coordinate.x = points[directions[previous][index][i]].first - 5;
        coordinate.y = points[directions[previous][index][i]].second - 5;
        goals.coordinates.push_back(coordinate);
      }
      
      previous = index;
    }
    ROS_INFO("%.2lf %.2lf\n", goals.coordinates[0].x, goals.coordinates[0].y);
    ROS_INFO("%.2lf %.2lf\n", goals.coordinates[1].x, goals.coordinates[1].y);

    while(ros::ok()){
      if(goals.coordinates.size() > 1){
        pub.publish(goals);
        goals.coordinates.clear();

      }
      ros::spinOnce();
      loop_rate.sleep();
    }

  }
  else
    ROS_INFO("Action did not finish before the time out.");


  //exit
  return 0;
}