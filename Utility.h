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
 * Utility.h
 *
 * Contains a set of utility functions
 *
 *  Created on: Dec 27, 2014
 *      Author: yuhanlyu
 */

#ifndef UTILITY_H_
#define UTILITY_H_
#include <cmath>
#ifndef M_PI
const double M_PI = 3.14159265358979323846264338327950288419716939937510582097494459230781640628620899862803482534211706798214808651328230664;
#endif
#ifndef M_PI_2
const double M_PI_2 = 1.57079632679489661923132169163975144209858469968755291048747229615390820314310449931401741267105853399107404325664115332;
#endif

const double M_PI_3 = 1.04719755119659774615421446109316762806572313312503527365831486410260546876206966620934494178070568932738269550442743554;
const double TWO_PI = 6.28318530717958647692528676655900576839433879875021164194988918461563281257241799725606965068423413596429617302656461329;
const double TWO_PI_3 = 2.09439510239319549230842892218633525613144626625007054731662972820521093752413933241868988356141137865476539100885487109;
const double TWO_ACOT2 = -0.927295218001612232428512462922428804057074108572240527621866177440395728331483410601200567969775785113059704502381675;

const double ZERO = 10.0 * std::numeric_limits<double>::epsilon();

/**
 * Transform (x, y) to (r, theta)
 */
inline void polar(double x, double y, double &r, double &theta) {
	r = hypot(x, y);
	theta = atan2(y, x);
}

/**
 * Checking whether lb < x < ub, assuming lb <= ub
 */
inline bool between(double x, double lb, double ub)
{
	return lb - ZERO < x && x < ub + ZERO;
}

/**
 * Normalize an angle x to [-pi, pi)
 */
inline double normalize(double x) {
	return x - TWO_PI * floor((x + M_PI) / TWO_PI);
}

/**
 * Compute the difference between two angles
 */
inline double angleDiff(double a, double b)
{
    double error1 = fabs(normalize(a) - normalize(b));
    double error2 = TWO_PI - error1;
    return error1 < error2 ? error1 : error2;
}

/**
 * If possible, compute sin and cos at the same time by using sincos function
 */
inline void mySinCos(double theta, double& sine, double &cosine)
{
#ifdef _GNU_SOURCE
	sincos(theta, &sine, &cosine);
#else
	sine = sin(theta);
	cosine = cos(theta);
#endif
}

/**
 * This function is used to compute the trajectory of a control
 */
inline void sincosc(double x, double& sinc, double& verc)
{
    if (x == 0.0) {
    	sinc = 1.0;
    	verc = 0.0;
    } else {
        mySinCos(x, sinc, verc);
        sinc /= x;
        verc = (1.0 - verc) / x;
    }
}

/**
 * Rotate x, y by an angle theta, and store the results in newx and newy
 */
inline void rotate(double x, double y, double theta, double& newx, double& newy)
{
	double sine, cosine;
	mySinCos(theta, sine, cosine);
	newx = cosine * x - sine * y;
	newy = sine * x + cosine * y;
}

#endif /* UTILITY_H_ */
