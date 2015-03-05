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

#include <cmath>
#include <cassert>
#include <iostream>
#include <limits>
#include <algorithm>
#include "../Utility.h"
#include "../Configuration.h"
#include "../Transformation.h"
#include "../Trajectory.h"
#include "ReedsSheppCar.h"

namespace RigidBody {

static const double RS_EPS = 1e-7;

enum RSType { NOP = -1, LEFT = 0, STRAIGHT = 1, RIGHT = 2 }; // This definition is different from OMPL
static const Control controlMapping[][2] = { { Control(-1, 0, 1), Control(1, 0, 1)}, //L-, L+
									  { Control(-1, 0, 0), Control(1, 0, 0) }, //S+, S-
									  { Control(-1, 0, -1), Control(1, 0, -1) } }; // R-, R+
static const int MAX_NUM_ACTIONS = 5;
static const RSType RSPathType[][MAX_NUM_ACTIONS] = {
		// For all trajectories, switching L and R can be done by change to its neighbor
		// For first four trajectories, their reverse is the last four 4 trajectories.
		{ RIGHT, RIGHT, STRAIGHT, LEFT, NOP },       // 0 C|CSC RRSL
		{ LEFT, LEFT, STRAIGHT, RIGHT, NOP },        // 1 C|CSC LLSR
		{ RIGHT, RIGHT, STRAIGHT, RIGHT, NOP },      // 2 C|CSC RRSR
		{ LEFT, LEFT, STRAIGHT, LEFT, NOP },         // 3 C|CSC LLSL
		{ RIGHT, RIGHT, LEFT, NOP, NOP },            // 4 CCC RRL
		{ LEFT, LEFT, RIGHT, NOP, NOP },             // 5 CCC LLR
		{ LEFT, STRAIGHT, RIGHT, RIGHT, NOP },       // 6 CSC|C LSRR
		{ RIGHT, STRAIGHT, LEFT, LEFT, NOP },        // 7 CSC|C RSLL
		{ RIGHT, STRAIGHT, RIGHT, RIGHT, NOP },      // 8 CSC|C RSRR
		{ LEFT, STRAIGHT, LEFT, LEFT, NOP },         // 9 CSC|C LSLL
		{ LEFT, RIGHT, RIGHT, NOP, NOP },            // 10 CCC LRR
		{ RIGHT, LEFT, LEFT, NOP, NOP },             // 11 CCC RLL
		// These trajectories' reverse are their neighbors
		{ RIGHT, RIGHT, STRAIGHT, LEFT, LEFT },      // 12 C|CSC|C RRSLL
		{ LEFT, LEFT, STRAIGHT, RIGHT, RIGHT },      // 13 C|CSC|C LLSRR
		{ LEFT, STRAIGHT, RIGHT, NOP, NOP },         // 14 CSC LSR
		{ RIGHT, STRAIGHT, LEFT, NOP, NOP },         // 15 CSC RSL
		{ LEFT, LEFT, RIGHT, RIGHT, NOP },           // 16 CCCC LLRR
		{ RIGHT, RIGHT, LEFT, LEFT, NOP },           // 17 CCCC RRLL
		// Below trajectories's reverse is itself
		{ LEFT, RIGHT, RIGHT, LEFT, NOP },           // 18 CCCC LRRL
		{ RIGHT, LEFT, LEFT, RIGHT, NOP },           // 19 CCCC RLLR
		{ LEFT, STRAIGHT, LEFT, NOP, NOP },          // 20 CSC LSL
		{ RIGHT, STRAIGHT, RIGHT, NOP, NOP },        // 21 CSC RSR
		{ RIGHT, RIGHT, RIGHT, NOP, NOP },           // 22 CCC RRR
		{ LEFT, LEFT, LEFT, NOP, NOP },              // 23 CCC LLL
};

// When types are changed, the following definitions should change accordingly
static const int numOfTypes = sizeof(RSPathType) / sizeof(RSPathType[0]);
static const int reverseMapping[numOfTypes] = {6, 7, 8, 9, 10, 11, 0, 1, 2, 3, 4, 5, 13, 12, 15, 14, 17, 16, 18, 19, 20, 21, 22, 23};
static const int flipMapping[numOfTypes] = {1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14, 17, 16, 19, 18, 21, 20, 23, 22};
static const int lengthOfPath[numOfTypes] = {4, 4, 4, 4, 3, 3, 4, 4, 4, 4, 3, 3, 5, 5, 3, 3, 4, 4, 4, 4, 3, 3, 3, 3};
static const int LSRR = 6;
static const int RSRR = 8;
static const int LRR = 10;
static const int LLSRR = 13;
static const int LSR = 14;
static const int LLRR = 16;
static const int LRRL = 18;
static const int RSR = 21;
static const int RRR = 22;

class RSPath {
public:
	RSPath(int type = 0,
		   double t = std::numeric_limits<double>::infinity(), double u = 0.0, double v = 0.0,
		   double w = 0.0, double x = 0.0) : length_{t, u, v, w, x}, type_(type)  {}

	int getType() const { return type_; }

	double getLength(int index) const { return length_[index]; }

	double length() const {
		return fabs(length_[0]) + fabs(length_[1]) + fabs(length_[2]) + fabs(length_[3]) + fabs(length_[4]);
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
		type_ = flipMapping[getType()];
	}

	/**
	 * Negate Vx
	 * The definition of L and R is L+ = (1, 0, 1), L- = (-1, 0, 1), R+ = (1, 0, -1), R- = (-1, 0, -1)
	 * which is different from original Reeds-Sheep paper and OMPL implementation
	 */
	void negateVx() {
		for (int i = 0; i < lengthOfPath[getType()]; ++i)
			length_[i] = -length_[i];
	}
private:
	double length_[MAX_NUM_ACTIONS];
	int type_;
};

// These constants are used for computing polar representation
// from the first rotation center to the last rotation center
// LC means that the rotation center is in the left and RC means that the rotation center is in the right.
// Thus, LC -> L+ or R-, RC -> R+ or L- (this is used in Reeds-Shepp's paper)
static const int ABS = 0;
static const int ARG = 1;
static const int LCLC = 0;
static const int LCRC = 1;
static const int RCLC = 2;
static const int RCRC = 3;

/**
 * Convert a ReedsSheppPath to a Trajectory
 */
static inline Trajectory convertToTrajectory(const RSPath& path)
{
	const RSType *pathType = RSPathType[path.getType()];
	Trajectory traj;
	for (int i = 0; i < lengthOfPath[path.getType()]; ++i)
		traj.add(controlMapping[pathType[i]][path.getLength(i) >= 0], fabs(path.getLength(i)));
	return traj;
}

/**
 * Verify the validity for the path, for debugging only
 */
static inline bool verify(double x, double y, double phi, const RSPath& path)
{
	Trajectory traj = convertToTrajectory(path);
	Transformation final = Transformation(Configuration(0, 0, 0)).move(traj);
	double error = fabs(final.getX() - x) + fabs(final.getY() - y) + angleDiff(final.getTheta(), phi);
	return fabs(final.getX() - x) + fabs(final.getY() - y) + angleDiff(final.getTheta(), phi) < RS_EPS;
}

/**
 * Assume (x, y, phi), where phi <= 0 and x and y are in the first quadrant in the rotated coordinate
 * Finding the shortest path in the type R+R-R+ from [3]
 * return true if found a path else return false
 */
static inline bool RpRmRp(double x, double y, double phi, double sine, double cosine, double& d, double& theta, double &t, double &u, double &v)
{
	// C|C|C R+R-R+ differs from [1] and [2]
	if (std::isnan(d))
		polar(x + sine, y - cosine + 1.0, d, theta);
	if (d > 4.0)
		return false;
	u = -2.0 * asin(0.25 * d);
	if (u <= phi - ZERO)
		return false;
	t = M_PI + 0.5 * u - theta;
	if (!between(t, 0, -phi))
		return false;
	v = u - t - phi;
	if (v <= -ZERO)
		return false;
	assert(verify(x, y, phi, RSPath(RRR, t, u, v)));
	return true;
}

/**
 * Assume (x, y, phi), where phi <= 0 and x and y are in the first quadrant in the rotated coordinate
 * Finding the shortest whirl path
 * return true  if found a path else return false
 */
static inline bool whirl(double x, double y, double phi, double sine, double cosine, double polarArray[][2], RSPath& path)
{
	double t, u, v;
	if (!RpRmRp(x, y, phi, sine, cosine, polarArray[RCRC][ABS], polarArray[RCRC][ARG], t, u, v))
		return false;
	path = RSPath(RRR, t, u, v);
	return true;
}

/**
 * Assume (x, y, phi), where phi <= 0 and x and y are in the first quadrant in the rotated coordinate
 * Finding the shortest path in the type L+R+R- from [3]
 * return true if found a path else return false
 */
static inline bool LpRpRm(double x, double y, double phi, double sine, double cosine, double& d, double& theta, double &t, double &u, double &v)
{
	// C|CC L+R+R- different from [1] and [2]
	if (std::isnan(d))
		polar(x - sine, y + cosine - 1.0, d, theta);
	if (d > 4.0)
		return false;
	u = 2.0 * asin(0.25 * d);
	if (!between(u, -0.5 * phi, M_PI_2))
		return false;
	t = 0.5 * u + theta;
	if (t >= u + ZERO)
		return false;
	v = -(t - u - phi);
	if (!between(-v, std::max(0.0, -phi - u), std::min(-phi, u)))
		return false;
	assert(verify(x, y, phi, RSPath(LRR, t, u, v)));
	return true;
}

/**
 * Assume (x, y, phi), where phi <= 0 and x and y are in the first quadrant in the rotated coordinate
 * Finding the shortest path in the type L+R+R-L- from [3]
 * return true if found a path else return false
 * I use a different formula from OMPL and Reeds-Shepp's paper
 */
static inline bool LpRupRumLm(double x, double y, double phi, double sine, double cosine, double& d, double& theta, double &t, double &u, double &v)
{
	// CC|CC L+R+R-L- differs from [1] and [2]
	if (phi < -TWO_PI_3)
		return false;
	if (std::isnan(d))
		polar(x + sine, y - cosine - 1.0, d, theta);
	if (d > 2.0)
		return false;
	u = acos(0.5 + 0.25 * d);
	if (!between(u, -0.5 * phi, M_PI_3))
		return false;
	t = u + theta + M_PI_2;
	if (t >= u + ZERO || !between(t, u + 0.5 * phi, 2.0 * u + phi))
		return false;
	v = -(2.0 * u + phi - t);
	assert(verify(x, y, phi, RSPath(LRRL, t, u, -u, v)));
	return true;
}

/**
 * Assume (x, y, phi), where phi <= 0 and x and y are in the first quadrant in the rotated coordinate
 * Finding the shortest path in the type L-L+R+R- from [3]
 * return true if found a path else return false
 * I use a different formula from OMPL and Reeds-Shepp's paper
 */
static inline bool LmLupRupRm(double x, double y, double phi, double sine, double cosine, double& d, double& theta, double &t, double &u, double &v)
{
	// C|CC|C L-L+R+R- differs from [1] and [2]
	if (phi < -M_PI_3)
		return false;
	if (std::isnan(d))
		polar(x - sine, y + cosine + 1.0, d, theta);
	if (d * d > 20.0 || d * d < 4.0)
		return false;
	u = acos(1.25 - (d * d) / 16.0);
	if (!between(u, -phi, M_PI_2))
		return false;
	t = M_PI_2 - theta - acos(1.5 / d + d / 8.0);
	v = t + phi;
	if (!between(-v, -phi, u))
		return false;
	assert(verify(x, y, phi, RSPath(LLRR, t, u, u, v)));
	return true;
}

/**
 * Assume (x, y, phi), where phi <= 0 and x and y are in the first quadrant in the rotated coordinate
 * Finding the shortest regular path
 * return true  if found a path else return false
 */
static inline bool regular(double x, double y, double phi, double sine, double cosine, double polarArray[][2], RSPath& path)
{
	double t, u, v;

	if (LpRpRm(x, y, phi, sine, cosine, polarArray[LCLC][ABS], polarArray[LCLC][ARG], t, u, v)) {
		path = RSPath(LRR, t, u, v);
		return true;
	}
	if (LpRupRumLm(x, y, phi, sine, cosine, polarArray[LCRC][ABS], polarArray[LCRC][ARG], t, u, v)) {
		path = RSPath(LRRL, t, u, -u, v);
		if (phi <= -M_PI_3)
			return true;
	}
	if (LmLupRupRm(x, y, phi, sine, cosine, polarArray[RCLC][ABS], polarArray[RCLC][ARG], t, u, v) && -t + 2.0 * u - v < path.length()) {
		path = RSPath(LLRR, t, u, u, v);
		return true;
	}
	return false;
}

/**
 * Assume (x, y, phi), where phi <= 0 and x and y are in the first quadrant in the rotated coordinate
 * Finding the shortest path in the type R+S+R+ from [3]
 * return true if found a path else return false
 */
static inline bool RpSpRp(double x, double y, double phi, double sine, double cosine, double& d, double& theta, double &t, double &u, double &v)
{
	// CSC R+S+R+ differs from [1] and [2]
	if (std::isnan(d))
		polar(x + sine, y - cosine + 1.0, d, theta);
	u = d;
	t = -theta;
	if (!between(t, 0, M_PI_2))
		return false;
	v = -phi - t;
	if (!between(v, 0, M_PI_2))
		return false;
	assert(verify(x, y, phi, RSPath(RSR, t, u, v)));
	return true;
}

/**
 * Assume (x, y, phi), where phi <= 0 and x and y are in the first quadrant in the rotated coordinate
 * Finding the shortest path in the type L+S+R+ from [3]
 * return true if found a path else return false
 */
static inline bool LpSpRp(double x, double y, double phi, double sine, double cosine, double& d, double& theta, double &t, double &u, double &v)
{
	// CSC L+S+R+ differs from [1] and [2]
	if (phi < -M_PI_2)
		return false;
	if (std::isnan(d))
		polar(x + sine, y - cosine - 1.0, d, theta);
	if (d < 2.0)
		return false;
	u = sqrt(d * d - 4.0);
	t = theta + atan2(2.0, u);
	if (!between(t, 0, M_PI_2))
		return false;
	v = t - phi;
	if (!between(v, 0, M_PI_2))
		return false;
	assert(verify(x, y, phi, RSPath(LSR, t, u, v)));
	return true;
}

/**
 * Assume (x, y, phi), where phi <= 0 and x and y are in the first quadrant in the rotated coordinate
 * Finding the shortest path in the type L+S+R+R- from [3]
 * return true if found a path else return false
 * I use a different formula from OMPL and Reeds-Shepp's paper
 */
static inline bool LpSpRpRm(double x, double y, double phi, double sine, double cosine, double& d, double& theta, double &t, double &u, double &v)
{
	// CSC|C L+S+R+R- differs from [1] and [2]
	if (std::isnan(d))
		polar(x - sine, y + cosine - 1.0, d, theta);
	if (d < 2.0)
		return false;
	u = sqrt(d * d - 4.0) - 2.0;
	if (u <= -ZERO)
		return false;
	t = M_PI_2 + theta - acos(2.0 / d);
	if (!between(t, 0, M_PI_2))
		return false;
	v = M_PI_2 + phi - t;
	if (!between(-v, 0, M_PI))
		return false;
	assert(verify(x, y, phi, RSPath(LSRR, t, u, M_PI_2, v)));
	return true;
}

/**
 * Assume (x, y, phi), where phi <= 0 and x and y are in the first quadrant in the rotated coordinate
 * Finding the shortest path in the type R+S+R+R- from [3]
 * return true if found a path else return false
 */
static inline bool RpSpRpRm(double x, double y, double phi, double sine, double cosine, double& d, double& theta, double &t, double &u, double &v)
{
	// CSC|C R+S+R+R- differs from [1] and [2]
	if (phi > -M_PI_2)
		return false;
	if (std::isnan(d))
		polar(x - sine, y + cosine + 1.0, d, theta);
	if (d < 2.0 || !between(-theta, 0, M_PI_2))
		return false;
	v = M_PI_2 + phi - theta;
	if (!between(-v, 0, M_PI))
		return false;
	t = -theta;
	u = d - 2.0;
	assert(verify(x, y, phi, RSPath(RSRR, t, u, M_PI_2, v)));
	return true;
}

/**
 * Assume (x, y, phi), where phi <= 0 and x and y are in the first quadrant in the rotated coordinate
 * Finding the shortest path in the type L-L+S+R+R- from [3]
 * return true if found a path else return false
 * I use a different formula from OMPL and Reeds-Shepp's paper
 */
static inline bool LmLpSpRpRm(double x, double y, double phi, double sine, double cosine, double& d, double& theta, double &t, double &u, double &v)
{
	// C|CSC|C L-L+S+R+R- differs from [1] and [2]
	if (phi < TWO_ACOT2)
		return false;
	if (std::isnan(d))
		polar(x - sine, y + cosine + 1.0, d, theta);
	if (d < 2.0)
		return false;
	u = sqrt(d * d - 4.0) - 4.0;
	if (u <= -ZERO)
		return false;
	t = M_PI_2 - theta - asin(2.0 / d);
	if (!between(-t, 0, M_PI_2))
		return false;
	v = phi + t;
	if (!between(-v, 0, M_PI_2))
		return false;
	assert(verify(x, y, phi, RSPath(LLSRR, t, M_PI_2, u, M_PI_2, v)));
	return true;
}

/**
 * Assume (x, y, phi), where phi <= 0 and x and y are in the first quadrant in the rotated coordinate
 * Finding the shortest singular path]
 * return true  if found a path else return false
 */
static inline bool singular(double x, double y, double phi, double sine, double cosine, double polarArray[][2], RSPath& path)
{
	double t, u, v, Lmin = std::numeric_limits<double>::infinity(), L;

	if (RpSpRp(x, y, phi, sine, cosine, polarArray[RCRC][ABS], polarArray[RCRC][ARG], t, u, v) && Lmin > (L = t + u + v)) {
		path = RSPath(RSR, t, u, v);
		Lmin = L;
	}
	if (LpSpRp(x, y, phi, sine, cosine, polarArray[LCRC][ABS], polarArray[LCRC][ARG], t, u, v) && Lmin > (L = t + u + v)) {
		path = RSPath(LSR, t, u, v);
		Lmin = L;
	}
	if (LpSpRpRm(x, y, phi, sine, cosine, polarArray[LCLC][ABS], polarArray[LCLC][ARG], t, u, v) && Lmin > (L = M_PI_2 + t + u - v)) {
		path = RSPath(LSRR, t, u, M_PI_2, v);
		Lmin = L;
	}
	if (RpSpRpRm(x, y, phi, sine, cosine, polarArray[RCLC][ABS], polarArray[RCLC][ARG], t, u, v) && Lmin > (L = M_PI_2 + t + u - v)) {
		path = RSPath(RSRR, t, u, M_PI_2, v);
		Lmin = L;
	}
	if (LmLpSpRpRm(x, y, phi, sine, cosine, polarArray[RCLC][ABS], polarArray[RCLC][ARG], t, u, v) && Lmin > (L = M_PI - t + u - v)) {
		path = RSPath(LLSRR, t, M_PI_2, u, M_PI_2, v);
		Lmin = L;
	}
	return std::isfinite(Lmin);
}

/**
 * Finding the shortest path from origin to (x, y, phi), where phi <= 0
 * and x and y are in the first quadrant in the rotated coordinate
 * The order of three types and the threshold is suggested in [3]
 */
static inline void shortestPath(double x, double y, double phi, RSPath& path)
{
	static const double threshold = 2.0;
	assert(phi >= -M_PI && phi <= 0);
	//assert((phi == 0 && x >= 0 && y >= 0)
    //     || between(y, x * tan(0.5 * phi), -x * tan(M_PI_2 - 0.5 * phi)));
	double sine, cosine, polarArray[4][2] = { {std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()},
			{std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()},
			{std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()},
			{std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()}};
	mySinCos(phi, sine, cosine);

	if (fabs(x) > threshold || fabs(y) > threshold)
		singular(x, y, phi, sine, cosine, polarArray, path)
     || regular(x, y, phi, sine, cosine, polarArray, path)
	 || whirl(x, y, phi, sine, cosine, polarArray, path);
	else
		whirl(x, y, phi, sine, cosine, polarArray, path)
	 || regular(x, y, phi, sine, cosine, polarArray, path)
	 || singular(x, y, phi, sine, cosine, polarArray, path);
}

/**
 * Exploit the symmetry of the configuration space, normalize to only one quadrant
 * After finding the optimal path, adjust accordingly.
 */
static inline RSPath helper(double x, double y, double phi)
{
	RSPath path;
	bool needFlipOmega = false;
	if (phi > 0) {
		phi = -phi;
		y = -y;
		needFlipOmega = true;
	}
	assert(phi >= -M_PI && phi <= 0);
	double xdelta, ydelta;
	rotate(x, y, -0.5 * phi, xdelta, ydelta);
	bool needNegate = xdelta < 0, needReverse = (xdelta < 0) != (ydelta < 0);
	rotate(fabs(xdelta), fabs(ydelta), 0.5 * phi, xdelta, ydelta);
	shortestPath(xdelta, ydelta, phi, path);
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
static RSPath normalizeToOrigin(const Configuration &initial, const Configuration& goal)
{
	assert(std::isfinite(initial.getX()) && std::isfinite(initial.getY() && std::isfinite(initial.getTheta())));
	assert(std::isfinite(goal.getX()) && std::isfinite(goal.getY() && std::isfinite(goal.getTheta())));
	// Rotate the initial to origin
	double x, y;
	rotate(goal.getX() - initial.getX(), goal.getY() - initial.getY(), -initial.getTheta(), x, y);
	return helper(x, y, normalize(goal.getTheta() - initial.getTheta()));
}

/**
 * Compute the shortest distance
 */
double ReedsSheppCar::distance(const Configuration &initial, const Configuration& goal)
{
	return normalizeToOrigin(initial, goal).length();
}

/**
 * Finding the minimum trajectory
 */
Trajectory ReedsSheppCar::shortestPath(const Configuration &initial, const Configuration& goal) {
	return convertToTrajectory(normalizeToOrigin(initial, goal));
}

/**
 * Compute the type of the minimum trajectory
 */
int ReedsSheppCar::shortestPathType(const Configuration &initial, const Configuration& goal) {
	RSPath path = normalizeToOrigin(initial, goal);
	int type = path.getType();
	return path.getLength(0) >= 0.0 ? type : 2 * type;
}

/**
 * Use OMPL to check the optimality
 *
 * When compared with OMPL
 * need to comment enum Type and #include "OMPLRSCar.h"
 */
/*
static inline bool verifyShortestPath(double x, double y, double phi, const RSPath& path)
{
	double cost1 = path.length();
	ReedsSheppPath path2 = shortestPathHelper(Configuration(0, 0, 0), Configuration(x, y, phi));
	double cost2 = path2.length();
	//std::cout << "First path " << path;
	//std::cout << "Second path " << path2;
	//std::cout << "Costs " << cost1 << ' ' << cost2 << '\n';
	//Trajectory traj = convertToTrajectory(path2);
	//std::cout << Configuration(Transformation(Configuration(0, 0, 0)).move(traj));
	return fabs(cost1 - cost2) < RS_EPS;
}

inline std::ostream& operator<<(std::ostream& os, const RSPath& path)
{
	os << "Type is: " << path.type_[0] << path.type_[1] << path.type_[2] << path.type_[3] << path.type_[4] << '\n';
	return os << "Length is: " << path.length_[0] << ' ' << path.length_[1] << ' ' << path.length_[2] << ' ' << path.length_[3] << ' ' << path.length_[4] << '\n';
}

static inline void verifySymmetry(double x, double y, double phi)
{
	double dist = ReedsSheppCar::distance(Configuration(0, 0, 0), Configuration(x, y, phi));

	// There are three isometric trajectories with the same phi
	// Let (xdelta, ydelta, phi) be the configuration by rotating the axis by -phi/2
	// Then, (-xdelta, ydelta, phi), (xdelta, -ydelta, phi), (-xdelta, -ydelta, phi)
	// The last one is (x, -y, -phi) and this one also has corresponding three isometric trajectories.

	double xdelta, ydelta;
	rotate(x, y, -0.5 * phi, xdelta, ydelta);
	rotate(-xdelta, ydelta, 0.5 * phi, xdelta, ydelta);
	double dist1 = ReedsSheppCar::distance(Configuration(0, 0, 0), Configuration(xdelta, ydelta, phi));
	std::cout << dist << ' ' << dist1 << '\n';
	assert(fabs(dist - dist1) < RS_EPS);

	rotate(x, y, -0.5 * phi, xdelta, ydelta);
	rotate(xdelta, -ydelta, 0.5 * phi, xdelta, ydelta);
	double dist2 = ReedsSheppCar::distance(Configuration(0, 0, 0), Configuration(xdelta, ydelta, phi));
	assert(fabs(dist - dist2) < RS_EPS);

	rotate(x, y, -0.5 * phi, xdelta, ydelta);
	rotate(-xdelta, -ydelta, 0.5 * phi, xdelta, ydelta);
	double dist3 = ReedsSheppCar::distance(Configuration(0, 0, 0), Configuration(xdelta, ydelta, phi));
	assert(fabs(dist - dist3) < RS_EPS);

	double dist4 = ReedsSheppCar::distance(Configuration(0, 0, 0), Configuration(x, -y, -phi));
	assert(fabs(dist - dist4) < RS_EPS);
}*/
} /* namespace RigidBody */
