/*
 * DubinsCarTest.cpp
 *
 *  Created on: Jan 4, 2015
 *      Author: yuhanlyu
 */

#include <iostream>
#include <random>
#include <cassert>
#include "../Utility.h"
#include "../Configuration.h"
#include "../Transformation.h"
#include "../Synthesis.h"
#include "DubinsCar.h"

using namespace RigidBody;
const int numOfRuns = 1000000;

static void test(const Configuration& initial, const Configuration& goal)
{
	//double distance = ReedsSheppCar::distance(initial, goal);
	Trajectory traj = DubinsCar::shortestPath(initial, goal);
	//std::cout << "Distance is " << distance;
	//std::cout << traj;
	Transformation final = Transformation(initial).move(traj);
	//std::cout << final << ' ' << Transformation(goal) << '\n';
	double error = fabs(final.getX() - goal.getX()) + fabs(final.getY() - goal.getY())
			     + angleDiff(final.getTheta(), goal.getTheta());
	assert(error < 1e-14);
}

static void randomTest()
{
	std::default_random_engine generator;
	std::uniform_real_distribution<double> distribution(-M_PI, M_PI);
	for (int i = 0; i < numOfRuns; ++i) {
		double xi = distribution(generator), yi = distribution(generator), thetai = normalize(distribution(generator));
		Configuration initial(xi, yi, thetai);
		//Configuration initial(0, 0, 0);
	    double x = distribution(generator), y = distribution(generator), theta = normalize(distribution(generator));
	    Configuration goal(x, y, theta);
	    //std::cout << i << ' ' << x << ' ' << y << ' ' << theta << '\n';
	    test(initial, goal);
	}
}

static void gridTest()
{
	static const double STEP = 0.05;
	static const double THETA_STEP = M_PI * STEP;
	static const double X_LB = -3, X_UB = 3;
	static const double Y_LB = -3, Y_UB = 3;

	Configuration initial(0, 0, 0);
	for (double x = X_LB; x <= X_UB; x += STEP) {
		std::cout << x << '\n';
		for (double y = Y_LB; y <= Y_UB; y += STEP) {
			for (double theta = -M_PI; theta <= M_PI; theta += THETA_STEP) {
				Configuration goal(x, y, theta);
				test(initial, goal);
			}
		}
	}
}

static void DubinsSynthesis()
{
	Dubins(-3, 3, -3, 3, (5.0 / 6.0) * M_PI, (5.0 / 6.0) * M_PI, 0.02, 0.05);
}

/*
int main(void)
{
	//simpleTest();
	randomTest();
	gridTest();
	//DubinsSynthesis();
	return 0;
}
*/
