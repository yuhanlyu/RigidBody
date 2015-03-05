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
 * ReedsSheppCar.cpp
 *
 *  Created on: Dec 23, 2014
 *      Author: yuhanlyu
 *
 *  I modified OMPL's code and combined several paper's result.
 *  I also simplified the formula of computing the trajectory based on my own work..
 *
 *  [1] Philippe Souères, J. -D. Boissonnat, "Optimal Trajectories for Nonholonomic Mobile Robotics,"
 *	    Robot Motion Planning and Control Lecture Notes in Control and Information Sciences
 *	    Volume 229, 1998, pp 93-170
 *  [2] Philippe Souères, "Minimum-Length Trajectories for a Car:
 *  	An Example of the Use of Boltianskii’s Sufficient Conditions for Optimality,"
 *  	IEEE Transactions on Automatic Control, 2007
 *  [3] Huifang Wang, Yangzhou Chen, and Philippe Souères,
 *  	"A Geometric Algorithm to Compute Time-Optimal Trajectories for a Bidirectional Steered Robot,"
 *  	IEEE Transactions on Robotics, 2009
 */

#ifndef REEDSSHEPPCAR_H_
#define REEDSSHEPPCAR_H_
#include "../Configuration.h"
#include "../Trajectory.h"

namespace RigidBody {

namespace ReedsSheppCar {
	Trajectory shortestPath(const Configuration &initial, const Configuration& goal);
	int shortestPathType(const Configuration &initial, const Configuration& goal);
	double distance(const Configuration &initial, const Configuration& goal);
};

} /* namespace RigidBody */

#endif /* REEDSSHEPPCAR_H_ */
