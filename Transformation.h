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
 * Transformation.h
 *
 * Basic definition of a transformation matrix in 2D.
 *
 *  Created on: Dec 23, 2014
 *      Author: yuhanlyu
 */

#ifndef TRANSFORMATION_H_
#define TRANSFORMATION_H_
#include <cmath>
#include "Utility.h"
#include "Configuration.h"
#include "Control.h"
#include "Trajectory.h"

namespace RigidBody {

class Transformation {
public:
	explicit Transformation(const Configuration& q) : x(q.getX()), y(q.getY()), sine(sin(q.getTheta())), cosine(cos(q.getTheta())) {}
	explicit Transformation(double arg_x, double arg_y, double arg_sine, double arg_cosine) : x(arg_x), y(arg_y), sine(arg_sine), cosine(arg_cosine) {}
	/**
	 * Conversion of a configuration
	 */
	explicit operator Configuration() const {
		return Configuration(x, y, atan2(sine, cosine));
	}
	/**
	 * Construct a transformation from a control with a duration
	 */
	explicit inline Transformation(const Control& u, double time) {
		double theta = u.getOmega() * time, sinc, verc;
	    sincosc(theta, sinc, verc);
	    x = time * (u.getVx() * sinc - u.getVy() * verc);
	    y = time * (u.getVx() * verc + u.getVy() * sinc);
	    mySinCos(theta, sine, cosine);
	}
	double getX() const { return x; }
	double getY() const { return y; }
	double getSin() const { return sine; }
	double getCos() const { return cosine; }
	double getTheta() const { return atan2(sine, cosine); }

	Transformation transform(const Transformation& T) const {
		return Transformation(cosine * T.getX() - sine * T.getY() + x,
				              sine * T.getX() + cosine * T.getY() + y,
				              sine * T.getCos() + cosine * T.getSin(),
							  cosine * T.getCos() - sine * T.getSin());
	}

	Transformation move(const Control& c, const double& time) const {
		return transform(Transformation(c, time));
    }

	Transformation move(const Trajectory& trajectory) const {
		Transformation T(*this);
		for (int i = 0; i < trajectory.size(); ++i) {
			T = T.move(trajectory.getControl(i), trajectory.getDuration(i));
		}
		return T;
	}
private:
	// These four variables should be constants during the life of a transformation
	// I didn't declare them as constants is due to I need to use sincos to initialize them.
	double x;
	double y;
	double sine;
	double cosine;
};

inline std::ostream& operator<<(std::ostream& os, const Transformation& T)
{
	return os << "[ [" << T.getCos() << ' ' << -T.getSin() << ' ' << T.getX() << "] [" <<
	        T.getSin() << ' ' << T.getCos() << ' ' << T.getY() << "] [" <<
	        0.0 << ' ' << 0.0 << ' ' << 1.0 << "] ]";
}
} /* namespace RigidBody */

#endif /* TRANSFORMATION_H_ */
