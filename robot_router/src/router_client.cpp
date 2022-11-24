#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <robot_router/TSPAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "test_fibonacci");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<robot_router::TSPAction> ac("tsp", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  robot_router::TSPGoal goal;
  geometry_msgs::Point32 coordinate;
  goal.tagCoordinates.push_back(coordinate);
  coordinate.x = 5.0f;
  goal.tagCoordinates.push_back(coordinate);
  coordinate.y = -3.0f;
  goal.tagCoordinates.push_back(coordinate);
  coordinate.x = -4.0f;
  goal.tagCoordinates.push_back(coordinate);
  
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