/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Dartmouth College
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Rice University nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * ReedsSheppTest.cpp
 *
 * Test driver for ReedsSheppCar
 * I used random configurations and grid points to verify my program.
 * It is important to test goal's orientation equals multiples of pi
 *
 *  Created on: Dec 28, 2014
 *      Author: yuhanlyu
 */

#include <iostream>
#include <random>
#include <cassert>
#include "../Utility.h"
#include "../Configuration.h"
#include "../Transformation.h"
#include "../Synthesis.h"
#include "ReedsSheppCar.h"

using namespace RigidBody;
const int numOfRuns = 1000000;

static void test(const Configuration& initial, const Configuration& goal)
{
	double distance = ReedsSheppCar::distance(initial, goal);
	Trajectory traj = ReedsSheppCar::shortestPath(initial, goal);
	//std::cout << "Distance is " << distance;
	///std::cout << traj;
	Transformation final = Transformation(initial).move(traj);
	//std::cout << final << ' ' << Transformation(goal) << '\n';
	double error = fabs(final.getX() - goal.getX()) + fabs(final.getY() - goal.getY())
			     + fabs(normalize(final.getTheta() - goal.getTheta()));

	//std::cout << "error is " << error << "\n\n";
	assert(error < 1e-8);
}


static void simpleTest()
{
	Configuration initial(0, 0, 0), goal1(-4, 3, 1), goal2(0.97, -1.4, 1.2), goal3(-0.4, -0.4, -3), goal4(0.0/0.0, 0, 0);
	//test(initial, goal1);
	//test(initial, goal2);
	//test(initial, goal3);
	//test(initial, goal4);
	test(initial, Configuration(0, 0, 0.1));
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
    int count = 0;
	for (double x = X_LB; x <= X_UB; x += STEP) {
		//std::cout << x << '\n';
		for (double y = Y_LB; y <= Y_UB; y += STEP) {
			for (double theta = -M_PI; theta <= M_PI; theta += THETA_STEP) {
				Configuration goal(x, y, theta);
                ++count;
				test(initial, goal);
			}
		}
	}
    std::cout << count << '\n';
}

static void RSSynthesis()
{
	ReedsShepp(-3, 3, -3, 3, 0.5 * M_PI_2, 0.5 * M_PI_2, 0.02, 0.05);
}


int main(void)
{
	//simpleTest();
	//randomTest();
	gridTest();
	//RSSynthesis();
	return 0;
}
