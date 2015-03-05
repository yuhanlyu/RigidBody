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
 * DiffDrive.cpp
 *
 * The following algorithm comes from
 * Devin J. Balkcom and Matthew T. Mason, Time Optimal Trajectories for Bounded Velocity Differential Drive Vehicles,
 * International Journal of Robotics Research, 21(3):199-217, 2002.
 *
 *  Created on: Jan 5, 2015
 *      Author: yuhanlyu
 */

#include <cmath>
#include <cassert>
#include <iostream>
#include <limits>
#include <algorithm>
#include "../Configuration.h"
#include "../Transformation.h"
#include "../Trajectory.h"
#include "../Utility.h"
#include "DiffDrive.h"

namespace RigidBody {

static const double DD_EPS = 1e-13;

enum DDType { NOP = -1, FORWARD = 0, LEFT = 1 };
static const Control controlMapping[][2] = { {Control(-1, 0, 0), Control(1, 0, 0)},
		                                     {Control(0, 0, -2), Control(0, 0, 2)} };

static const int MAX_NUM_ACTIONS = 4;
static const DDType DDPathType[][MAX_NUM_ACTIONS] = {
		{ LEFT, FORWARD, LEFT, NOP },
		{ FORWARD, LEFT, FORWARD, NOP },
		{ LEFT, FORWARD, LEFT, FORWARD},
		{ FORWARD, LEFT, FORWARD, LEFT},
};
static const int numOfTypes = sizeof(DDPathType) / sizeof(DDPathType[0]);
static const int lengthOfPath[numOfTypes] = {3, 3, 4, 4};
static const int reverseMapping[numOfTypes] = {0, 1, 3, 2};
static const int TYPE_RBR = 0;
static const int TYPE_LBR = 0;
static const int TYPE_BRF = 1;
static const int TYPE_LBRF = 2;

class DDPath {
public:
	DDPath(int type = 0,
		   double t = std::numeric_limits<double>::infinity(), double u = 0.0, double v = 0.0,
		   double w = 0.0) : length_{t, u, v, w}, type_(type)  {}

	int getType() const { return type_; }

	double getLength(int index) const { return length_[index]; }

	double length() const {
		return fabs(length_[0]) + fabs(length_[1]) + fabs(length_[2]) + fabs(length_[3]);
	}

	/**
	 * Reverse the controls
	 */
	void reverse() {
		std::reverse(length_, length_+ lengthOfPath[getType()]);
		type_ = reverseMapping[getType()];
	}

	/**
	 * Switch between right turn and left turn
	 * Change from L to R.
	 * The definition of L and R is L+ = (1, 0, 1), L- = (-1, 0, 1), R+ = (1, 0, -1), R- = (-1, 0, -1)
	 * which is different from original Reeds-Sheep paper and OMPL implementation
	 */
	void flipOmega() {
		for (int i = 0; i < lengthOfPath[getType()]; ++i)
			if (DDPathType[getType()][i] == LEFT)
				length_[i] = -length_[i];
	}

	/**
	 * Negate Vx
	 * The definition of L and R is L+ = (1, 0, 1), L- = (-1, 0, 1), R+ = (1, 0, -1), R- = (-1, 0, -1)
	 * which is different from original Reeds-Sheep paper and OMPL implementation
	 */
	void negateVx() {
		for (int i = 0; i < lengthOfPath[getType()]; ++i)
			if (DDPathType[getType()][i] == FORWARD)
				length_[i] = -length_[i];
	}
private:
	double length_[MAX_NUM_ACTIONS];
	int type_;
};

/**
 * Convert a DiffDrive path to a Trajectory
 */
static inline Trajectory convertToTrajectory(const DDPath& path)
{
	const DDType *pathType = DDPathType[path.getType()];
	Trajectory traj;
	for (int i = 0; i < lengthOfPath[path.getType()]; ++i)
		traj.add(controlMapping[pathType[i]][path.getLength(i) >= 0], fabs(path.getLength(i)));
	return traj;
}

/**
 * Verify the validity for the path, for debugging only
 */
static inline bool verify(double x, double y, double phi, const DDPath& path)
{
	Trajectory traj = convertToTrajectory(path);
	Transformation final = Transformation(Configuration(x, y, phi)).move(traj);
	double error = fabs(final.getX()) + fabs(final.getY()) + normalize(final.getTheta());
	return fabs(final.getX()) + fabs(final.getY()) + normalize(final.getTheta()) < DD_EPS;
}

/**
 * Finding the shortest path from origin to (x, y, phi), where phi >= 0
 * and x and y are in the first quadrant in the rotated coordinate
 */
static inline DDPath shortestPath(double x, double y, double phi)
{
	double d, theta;
	polar(x, y, d, theta);
	if (theta <= phi) {
		assert(verify(x, y, phi, DDPath(TYPE_RBR, 0.5 * (theta - phi), -d, -0.5 * theta)));
		return DDPath(TYPE_RBR, 0.5 * (theta - phi), -d, -0.5 * theta);
	} else if (y <= 1 - cos(phi)) {
		assert(verify(x, y, phi, DDPath(TYPE_BRF, -y / sin(phi), -0.5 * (phi), y * tan(M_PI_2 - phi) - x)));
		return DDPath(TYPE_BRF, -y / sin(phi), -0.5 * (phi), y * tan(M_PI_2 - phi) - x);
	} else if (d >= tan(0.5 * theta)) {
		assert(verify(x, y, phi, DDPath(TYPE_LBR, 0.5 * (theta - phi), -d, -0.5 * theta)));
		return DDPath(TYPE_LBR, 0.5 * (theta - phi), -d, -0.5 * theta);
	}
	theta = acos(1 - y);
	assert(verify(x, y, phi, DDPath(TYPE_LBRF, 0.5 * (theta - phi), -y / sin(theta), -0.5 * theta, y * tan(M_PI_2 - theta) - x)));
	return DDPath(TYPE_LBRF, 0.5 * (theta - phi), -y / sin(theta), -0.5 * theta, y * tan(M_PI_2 - theta) - x);
}

/**
 * Exploit the symmetry of the configuration space, normalize to only one quadrant
 * After finding the optimal path, adjust accordingly.
 */
static inline DDPath helper(double x, double y, double phi)
{
	bool needFlipOmega = false;
	if (phi < 0) {
		phi = -phi;
		y = -y;
		needFlipOmega = true;
	}
	assert(0 <= phi && phi <= M_PI);
	double xdelta, ydelta;
	rotate(x, y, -0.5 * phi, xdelta, ydelta);
	bool needNegate = xdelta < 0, needReverse = (xdelta < 0) != (ydelta < 0);
	rotate(fabs(xdelta), fabs(ydelta), 0.5 * phi, xdelta, ydelta);
	DDPath path = shortestPath(xdelta, ydelta, phi);
	if (needFlipOmega)
		path.flipOmega();
	if (needNegate)
		path.negateVx();
	if (needReverse)
		path.reverse();
	return path;
}

/**
 * Finding shortest path from initial to goal
 */
static DDPath normalizeToOrigin(const Configuration &initial, const Configuration& goal)
{
	// Rotate the initial to origin
	double x, y;
	rotate(initial.getX() - goal.getX(), initial.getY() - goal.getY(), -goal.getTheta(), x, y);
	return helper(x, y, normalize(initial.getTheta() - goal.getTheta()));
}

double DiffDrive::distance(const Configuration &initial, const Configuration& goal)
{
	return normalizeToOrigin(initial, goal).length();
}

Trajectory DiffDrive::shortestPath(const Configuration &initial, const Configuration& goal) {
	return convertToTrajectory(normalizeToOrigin(initial, goal));
}

int DiffDrive::shortestPathType(const Configuration &initial, const Configuration& goal) {
	DDPath path = normalizeToOrigin(initial, goal);
	int type = path.getType();
	int multiplier1 = path.getLength(0) >= 0.0;
	int multiplier2 = path.getLength(1) >= 0.0;
	return type + numOfTypes * (multiplier1 + 2 * multiplier2);
}

}

