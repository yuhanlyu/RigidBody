/*
 * Synthesis.h
 *
 *  Created on: Jan 4, 2015
 *      Author: yuhanlyu
 */

#ifndef SYNTHESIS_H_
#define SYNTHESIS_H_


namespace RigidBody {

void ReedsShepp(double x_lb, double x_ub, double y_lb, double y_ub, double theta_lb, double theta_ub, double step, double theta_step);
void Dubins(double x_lb, double x_ub, double y_lb, double y_ub, double theta_lb, double theta_ub, double step, double theta_step);
void DiffDriveCar(double x_lb, double x_ub, double y_lb, double y_ub, double theta_lb, double theta_ub, double step, double theta_step);
}


#endif /* SYNTHESIS_H_ */
