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
 * DubinsCar.cpp
 *
 *  Created on: Jan 4, 2015
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
#include "DubinsCar.h"

namespace RigidBody {

static const double D_EPS = 1e-13;

enum DType { LEFT = 0, STRAIGHT = 1, RIGHT = 2 };
static const int MAX_NUM_ACTIONS = 3;
static const Control controlMapping[MAX_NUM_ACTIONS] = { Control(1, 0, 1),
														 Control(1, 0, 0),
														 Control(1, 0, -1)};

static const DType DPathType[][MAX_NUM_ACTIONS] = {
	{ LEFT, STRAIGHT, RIGHT},   // LSR
	{ RIGHT, STRAIGHT, LEFT},   // RSL
	{ LEFT, STRAIGHT, LEFT},    // LSL
	{ RIGHT, STRAIGHT, RIGHT},  // RSR
	{ LEFT, RIGHT, LEFT},       // LRL
	{ RIGHT, LEFT, RIGHT},      // RLR
};

static const int TYPE_LSR = 0;
static const int TYPE_RSL = 1;
static const int TYPE_LSL = 2;
static const int TYPE_RSR = 3;
static const int TYPE_LRL = 4;
static const int TYPE_RLR = 5;

class DPath {
public:
	DPath(int type = 0,
		  double t = std::numeric_limits<double>::infinity(), double u = 0.0, double v = 0.0)
			: length_{t, u, v}, type_(type)  {}

	int getType() const { return type_; }

	double getLength(int index) const { return length_[index]; }

	double length() const {
		return length_[0] + length_[1] + length_[2];
	}

private:
	double length_[MAX_NUM_ACTIONS];
	int type_;
};

static const int ABS = 0;
static const int ARG = 1;
static const int LCLC = 0;
static const int LCRC = 1;
static const int RCLC = 2;
static const int RCRC = 3;

/**
 * Normalize an angle x to [0, 2pi)
 */
inline double mod2pi(double x)
{
	double temp = fmod(x, TWO_PI);
	return temp >= 0 ? temp : temp + TWO_PI;
}

/**
 * Normalize an angle x to [0, 2pi), assuming x is in [0, 4pi]
 */
inline double mod2piDown(double x)
{
	return x < TWO_PI ? x : x - TWO_PI;
}

/**
 * Normalize an angle x to [0, 2pi), assuming x is in [-2pi, 2pi]
 */
inline double mod2piUp(double x)
{
	return x >= 0 ? x : x + TWO_PI;
}

/**
 * Convert a DubinsPath to a Trajectory
 */
static inline Trajectory convertToTrajectory(const DPath& path)
{
	const DType *pathType = DPathType[path.getType()];
	Trajectory traj;
	for (int i = 0; i < MAX_NUM_ACTIONS; ++i)
		traj.add(controlMapping[pathType[i]], fabs(path.getLength(i)));
	return traj;
}

/**
 * Verify the validity for the path, for debugging only
 */
static inline bool verify(double x, double y, double phi, const DPath& path)
{
	Trajectory traj = convertToTrajectory(path);
	Transformation final = Transformation(Configuration(0, 0, 0)).move(traj);
	double error = fabs(final.getX() - x) + fabs(final.getY() - y) + angleDiff(final.getTheta(), phi);
	return fabs(final.getX() - x) + fabs(final.getY() - y) + angleDiff(final.getTheta(), phi) < D_EPS;
}

/**
 * LSR trajectory
 */
static inline bool LSR(double x, double y, double phi, double d, double theta, double& t, double& u, double& v)
{
	if (d < 2.0)
		return false;
	u = sqrt(d * d - 4.0);
	t = mod2piUp(theta + atan2(2.0, u));
	v = mod2piUp(-(phi - t));
	assert(verify(x, y, phi, DPath(TYPE_LSR, t, u, v)));
	return true;
}

/**
 * RSL trajectory
 */
static inline bool RSL(double x, double y, double phi, double d, double theta, double& t, double& u, double& v)
{
	if (d < 2.0)
		return false;
	u = sqrt(d * d - 4.0);
	t = mod2piUp(-(theta - atan2(2.0, u)));
	v = mod2piDown(phi + t);
	assert(verify(x, y, phi, DPath(TYPE_RSL, t, u, v)));
	return true;
}

/**
 * LSL trajectory
 */
static inline bool LSL(double x, double y, double phi, double d, double theta, double& t, double& u, double& v)
{
	u = d;
	t = mod2piUp(theta);
	v = mod2piUp(phi - t);
	assert(verify(x, y, phi, DPath(TYPE_LSL, t, u, v)));
	return t + v <= TWO_PI;

}

/**
 * RSR trajectory
 */
static inline bool RSR(double x, double y, double phi, double d, double theta, double& t, double& u, double& v)
{
	u = d;
	t = mod2piUp(-theta);
	v = TWO_PI - mod2piDown(phi + t);
	assert(verify(x, y, phi, DPath(TYPE_RSR, t, u, v)));
	return t + v <= TWO_PI;
}

/**
 * LRL trajectory
 */
static inline bool LRL(double x, double y, double phi, double d, double theta, double& t, double& u, double& v)
{
	if (d > 4.0)
		return false;
	u = asin(0.25 * d);
	t = mod2piUp(M_PI - u + theta);
	u = TWO_PI - 2.0 * u;
	if (t >= u - ZERO)
		return false;
	v = phi - t + u;
	assert(verify(x, y, phi, DPath(TYPE_LRL, t, u, v)));
	return v <= u + ZERO && (t < u - M_PI || v < u - M_PI);
}

/**
 * RLR trajectory
 */
static inline bool RLR(double x, double y, double phi, double d, double theta, double& t, double& u, double& v)
{
	if (d > 4.0)
		return false;
	u = asin(0.25 * d);
	t = mod2pi(-(M_PI + u + theta));
	u = TWO_PI - 2.0 * u;
	v = mod2pi(-(phi + t - u));
	assert(verify(x, y, phi, DPath(TYPE_RLR, t, u, v)));
	return t <= u + ZERO && v <= u + ZERO && (t < u - M_PI || v < u - M_PI);
}

/**
 * Finding the shortest path from origin to (x, y, phi)
 */
static inline DPath shortestPath(double x, double y, double phi)
{
	DPath path;
	double sine, cosine, polarArray[4][2];
	mySinCos(phi, sine, cosine);
	polar(x - sine, y + cosine - 1.0, polarArray[LCLC][ABS], polarArray[LCLC][ARG]);
	polar(x + sine, y - cosine - 1.0, polarArray[LCRC][ABS], polarArray[LCRC][ARG]);
	polar(x - sine, y + cosine + 1.0, polarArray[RCLC][ABS], polarArray[RCLC][ARG]);
	polar(x + sine, y - cosine + 1.0, polarArray[RCRC][ABS], polarArray[RCRC][ARG]);

	double t, u, v, Lmin = std::numeric_limits<double>::infinity(), L;
	if (LSR(x, y, phi, polarArray[LCRC][ABS], polarArray[LCRC][ARG], t, u, v) && Lmin > (L = t + u + v)) {
		path = DPath(TYPE_LSR, t, u, v);
		Lmin = L;
	}
	if (RSL(x, y, phi, polarArray[RCLC][ABS], polarArray[RCLC][ARG], t, u, v) && Lmin > (L = t + u + v)) {
		path = DPath(TYPE_RSL, t, u, v);
		Lmin = L;
	}
	if (LSL(x, y, phi, polarArray[LCLC][ABS], polarArray[LCLC][ARG], t, u, v) && Lmin > (L = t + u + v)) {
		path = DPath(TYPE_LSL, t, u, v);
		Lmin = L;
	}
	if (RSR(x, y, phi, polarArray[RCRC][ABS], polarArray[RCRC][ARG], t, u, v) && Lmin > (L = t + u + v)) {
		path = DPath(TYPE_RSR, t, u, v);
		Lmin = L;
	}
	if (LRL(x, y, phi, polarArray[LCLC][ABS], polarArray[LCLC][ARG], t, u, v) && Lmin > (L = t + u + v)) {
		path = DPath(TYPE_LRL, t, u, v);
		Lmin = L;
	}
	if (RLR(x, y, phi, polarArray[RCRC][ABS], polarArray[RCRC][ARG], t, u, v) && Lmin > (L = t + u + v)) {
		path = DPath(TYPE_RLR, t, u, v);
		Lmin = L;
	}
	return path;
}

/**
 * Finding shortest path from initial to goal
 */
static inline DPath normalizeToOrigin(const Configuration &initial, const Configuration& goal)
{
	// Rotate the initial to origin
	double x, y;
	rotate(goal.getX() - initial.getX(), goal.getY() - initial.getY(), -initial.getTheta(), x, y);
	return shortestPath(x, y, mod2pi(goal.getTheta() - initial.getTheta()));
}

double DubinsCar::distance(const Configuration &initial, const Configuration& goal)
{
	return normalizeToOrigin(initial, goal).length();
}

Trajectory DubinsCar::shortestPath(const Configuration &initial, const Configuration& goal) {
	return convertToTrajectory(normalizeToOrigin(initial, goal));
}

int DubinsCar::shortestPathType(const Configuration &initial, const Configuration& goal) {
	return normalizeToOrigin(initial, goal).getType();
}

}
