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
 * Synthesis.cpp
 *
 * Construct numerical synthesis
 *
 *  Created on: Jan 4, 2015
 *      Author: yuhanlyu
 */

#include <iostream>
#include <random>
#include <cassert>
#include "Utility.h"
#include "Configuration.h"
#include "Transformation.h"
#include "ReedsSheppCar.h"
#include "DubinsCar.h"
#include "DiffDrive.h"

namespace RigidBody {

void ReedsShepp(double x_lb, double x_ub, double y_lb, double y_ub, double theta_lb, double theta_ub, double step, double theta_step)
{
	if (x_lb == x_ub) {
		for (double y = y_lb; y <= y_ub; y += step) {
			for (double theta = theta_lb; theta <= theta_ub; theta += theta_step) {
				Configuration initial(0, 0, 0);
				Configuration goal(x_lb, y, theta_lb);
				int type = ReedsSheppCar::shortestPathType(initial, goal);
				double distance = ReedsSheppCar::distance(initial, goal);
				std::cout << y << ' ' << theta << ' ' << type << ' ' << distance << '\n';
			}
		}
	} else if (y_lb == y_ub) {
		for (double x = x_lb; x <= x_ub; x += step) {
			for (double theta = theta_lb; theta <= theta_ub; theta += theta_step) {
				Configuration initial(0, 0, 0);
				Configuration goal(x, y_lb, theta_lb);
				int type = ReedsSheppCar::shortestPathType(initial, goal);
				double distance = ReedsSheppCar::distance(initial, goal);
				std::cout << x << ' ' << theta << ' ' << type << ' ' << distance << '\n';
			}
		}
	} else if (theta_lb == theta_ub) {
		for (double x = x_lb; x <= x_ub; x += step) {
			for (double y = y_lb; y <= y_ub; y += step) {
				Configuration initial(0, 0, 0);
				Configuration goal(x, y, theta_lb);
				int type = ReedsSheppCar::shortestPathType(initial, goal);
				double distance = ReedsSheppCar::distance(initial, goal);
				std::cout << x << ' ' << y << ' ' << type << ' ' << distance << '\n';
			}
		}
	} else
		assert(false);
}

void Dubins(double x_lb, double x_ub, double y_lb, double y_ub, double theta_lb, double theta_ub, double step, double theta_step)
{
	if (x_lb == x_ub) {
		for (double y = y_lb; y <= y_ub; y += step) {
			for (double theta = theta_lb; theta <= theta_ub; theta += theta_step) {
				Configuration initial(0, 0, 0);
				Configuration goal(x_lb, y, theta_lb);
				int type = DubinsCar::shortestPathType(initial, goal);
				double distance = DubinsCar::distance(initial, goal);
				std::cout << y << ' ' << theta << ' ' << type << ' ' << distance << '\n';
			}
		}
	} else if (y_lb == y_ub) {
		for (double x = x_lb; x <= x_ub; x += step) {
			for (double theta = theta_lb; theta <= theta_ub; theta += theta_step) {
				Configuration initial(0, 0, 0);
				Configuration goal(x, y_lb, theta_lb);
				int type = DubinsCar::shortestPathType(initial, goal);
				double distance = DubinsCar::distance(initial, goal);
				std::cout << x << ' ' << theta << ' ' << type << ' ' << distance << '\n';
			}
		}
	} else if (theta_lb == theta_ub) {
		for (double x = x_lb; x <= x_ub; x += step) {
			for (double y = y_lb; y <= y_ub; y += step) {
				Configuration initial(0, 0, 0);
				Configuration goal(x, y, theta_lb);
				int type = DubinsCar::shortestPathType(initial, goal);
				double distance = DubinsCar::distance(initial, goal);
				std::cout << x << ' ' << y << ' ' << type << ' ' << distance << '\n';
			}
		}
	} else
		assert(false);
}

void DiffDriveCar(double x_lb, double x_ub, double y_lb, double y_ub, double theta_lb, double theta_ub, double step, double theta_step)
{
	if (x_lb == x_ub) {
		for (double y = y_lb; y <= y_ub; y += step) {
			for (double theta = theta_lb; theta <= theta_ub; theta += theta_step) {
				Configuration initial(0, 0, 0);
				Configuration goal(x_lb, y, theta_lb);
				int type = DiffDrive::shortestPathType(initial, goal);
				double distance = DiffDrive::distance(initial, goal);
				std::cout << y << ' ' << theta << ' ' << type << ' ' << distance << '\n';
			}
		}
	} else if (y_lb == y_ub) {
		for (double x = x_lb; x <= x_ub; x += step) {
			for (double theta = theta_lb; theta <= theta_ub; theta += theta_step) {
				Configuration initial(0, 0, 0);
				Configuration goal(x, y_lb, theta_lb);
				int type = DiffDrive::shortestPathType(initial, goal);
				double distance = DiffDrive::distance(initial, goal);
				std::cout << x << ' ' << theta << ' ' << type << ' ' << distance << '\n';
			}
		}
	} else if (theta_lb == theta_ub) {
		for (double x = x_lb; x <= x_ub; x += step) {
			for (double y = y_lb; y <= y_ub; y += step) {
				Configuration initial(0, 0, 0);
				Configuration goal(x, y, theta_lb);
				int type = DiffDrive::shortestPathType(initial, goal);
				double distance = DiffDrive::distance(initial, goal);
				std::cout << x << ' ' << y << ' ' << type << ' ' << distance << '\n';
			}
		}
	} else
		assert(false);
}
}


