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



  // send a goal to the action
  robot_router::TSPGoal goal;
  geometry_msgs::Point32 coordinate;

  coordinate.z = 0;
  //std::cout << "points size:" << points.size() << std::endl;

  for(int i = 0; i < points.size(); i++){
    coordinate.x = points[i].first - 6;
    coordinate.y = points[i].second - 6;
    std::cout << coordinate << std::endl;
    goal.tagCoordinates.push_back(coordinate);
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

  std::cout << "antes do fim" << std::endl;

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