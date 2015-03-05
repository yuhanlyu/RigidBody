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
 * Trajectory.h
 *
 * Basic definition of a generic trajectory for all planar rigid-body robots
 *
 *  Created on: Dec 23, 2014
 *      Author: yuhanlyu
 */

#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_
#include <vector>
#include <algorithm>
#include "Control.h"

namespace RigidBody {

struct ControlTime {
public:
	explicit ControlTime(const Control &arg_u, double arg_time) : u(arg_u), time(arg_time) {}
	const Control u;
	const double time;
};

class Trajectory {
private:
	std::vector<ControlTime> controls;
public:
	explicit Trajectory() : controls() {}
	Control getControl(int index) const { return controls[index].u; }
	double getDuration(int index) const { return controls[index].time; }
	void add(Control u, double time) { controls.emplace_back(u, time); }
	double length() const {
		double sum = 0.0;
		for (int i = 0; i < controls.size(); ++i)
			sum += controls[i].time;
		return sum;
	}
	int size() const { return controls.size(); }
};

inline std::ostream& operator<<(std::ostream& os, const Trajectory& traj)
{
	for (int i = 0; i < traj.size(); ++i) {
	        os << traj.getControl(i) << ' ';
	        os << traj.getDuration(i) << '\n';
	    }
	return os;
}

} /* namespace RigidBody */

#endif /* TRAJECTORY_H_ */
