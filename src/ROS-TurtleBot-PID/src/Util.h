#ifndef UTIL_H
#define UTIL_H

#include <cmath>
#include "vector2D.h"

#define YAW_ERROR_TOL 0.025
#define POS_ERROR_TOL 0.1
#define K_YAW 1
#define K_X 1

double clip (double v, double lo, double hi)
{
	if (v < lo)
		return lo;
	if (v > hi)
		return hi;
	return v;
}


double ComputeGoalYaw(
		double curr_x,
		double curr_y,
		double goal_x,
		double goal_y
		)
{
	double goal_yaw =  atan2(goal_y - curr_y, goal_x - curr_x) + M_PI / 2;

	if (goal_yaw > M_PI)
		return goal_yaw - 2 * M_PI;

	if (goal_yaw <- M_PI)
		return goal_yaw + 2 * M_PI;

	return goal_yaw;
}

int ControlVelocities(
		double& cmd_vel_yaw, 
		double& cmd_vel_x, 
		const double goal_yaw,
		const double curr_yaw,
		const double curr_x,
		const double curr_y,
		const double goal_x,
		const double goal_y
		)
{
	VECTOR2D curr_pos(curr_x, curr_y);
	VECTOR2D goal_pos(goal_x, goal_y);

	double pos_error = (goal_pos - curr_pos).getMagnitude();
	double yaw_error = (curr_yaw - goal_yaw);

	ROS_INFO("%f %f", pos_error, yaw_error);

	if (abs(yaw_error) > 0.05)
	{
		// cmd_vel_yaw = K_YAW * yaw_error;
		cmd_vel_yaw = clip(K_YAW * yaw_error, -1, 1);
		cmd_vel_x = 0;

		return 0;
	}
	else 
	{
		if (pos_error > 0.2)
		{
			// cmd_vel_yaw = 0;
			// cmd_vel_yaw = K_YAW * yaw_error;
			cmd_vel_yaw = clip(K_YAW * yaw_error, -1, 1);
			cmd_vel_x = 0.5;
			return 1;
		}
		else {
			// cmd_vel_yaw = K_YAW * yaw_error;
			cmd_vel_yaw = clip(K_YAW * yaw_error, -1, 1);
			cmd_vel_x = 0;
			return 2;
		}
	}
}

#endif

