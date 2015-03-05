/*
 * DiffDriveTest.cpp
 *
 *  Created on: Jan 5, 2015
 *      Author: yuhanlyu
 */

#include <iostream>
#include <random>
#include <cassert>
#include "../Utility.h"
#include "../Configuration.h"
#include "../Transformation.h"
#include "../Synthesis.h"
#include "DiffDrive.h"

using namespace RigidBody;
const int numOfRuns = 1000000;

static void test(const Configuration& initial, const Configuration& goal)
{
	//double distance = ReedsSheppCar::distance(initial, goal);
	Trajectory traj = DiffDrive::shortestPath(initial, goal);
	//std::cout << "Distance is " << distance;
	//std::cout << traj;
	Transformation final = Transformation(initial).move(traj);
	//std::cout << final << ' ' << Transformation(goal) << '\n';
	double error = fabs(final.getX() - goal.getX()) + fabs(final.getY() - goal.getY())
			     + angleDiff(final.getTheta(), goal.getTheta());
	//std::cout << error << '\n';
	assert(error < 1e-13);
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

static void DiffDriveSynthesis()
{
	DiffDriveCar(-2, 2, -2, 2, 0.5 * M_PI_2, 0.5 * M_PI_2, 0.02, 0.05);
}

static void simpleTest()
{
	Configuration initial(0.14, 0.14, 0.5 * M_PI_2), goal(0, 0, 0);
	Trajectory traj = DiffDrive::shortestPath(initial, goal);
	std::cout << traj.getControl(0) << ' ' << traj.getDuration(0) << '\n';
	std::cout << traj.getControl(1) << ' ' << traj.getDuration(1) << '\n';
	std::cout << traj.getControl(2) << ' ' << traj.getDuration(2) << '\n';
	std::cout << DiffDrive::shortestPathType(initial, goal) << '\n';
	std::cout << DiffDrive::distance(initial, goal) << '\n';
}

int main(void)
{
	//simpleTest();
	//randomTest();
	gridTest();
	//DiffDriveSynthesis();
	return 0;
}
