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
 * DubinsCar.h
 *
 * There is one paper mentioned how to optimize the computation
 * Andrei M. Shkela, Vladimir Lumelskyb, Classification of the Dubins set,
 * Robotics and Autonomous Systems Volume 34, Issue 4, 31 March 2001, Pages 179–202
 *
 * But I just implement it naively..
 *
 *  Created on: Jan 4, 2015
 *      Author: yuhanlyu
 */

#ifndef DUBINSCAR_H_
#define DUBINSCAR_H_
#include "../Configuration.h"
#include "../Trajectory.h"

namespace RigidBody {

class DubinsCar {
public:
	explicit DubinsCar() = delete;
	static Trajectory shortestPath(const Configuration &initial, const Configuration& goal);
	static int shortestPathType(const Configuration &initial, const Configuration& goal);
	static double distance(const Configuration &initial, const Configuration& goal);
};

} /* namespace RigidBody */



#endif /* DUBINSCAR_H_ */
