#include <cmath>
#include <map>

std::pair<double, double> PotentialFunc(double x, double y)
{
	double des_x = 6;
	double des_y = 6;
	double vel_factor = 10;
	double dist = sqrt(pow(x - des_x, 2) + pow(y - des_y, 2));

	double grad_x = - (x - des_x) / (
			pow(pow(x - des_x, 2) + pow(y - des_y, 2), 1.5)
			);

	double grad_y = - (y - des_y) / (
			pow(pow(x - des_x, 2) + pow(y - des_y, 2), 1.5)
			);
	grad_y = 0;

	std::pair<double, double> grad = make_pair(vel_factor * grad_x, vel_factor * grad_y);

	return grad;
}
