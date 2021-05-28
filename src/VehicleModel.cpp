#include "VehicleModel.h"
#include <cmath>
#include <stdio.h>
#include "MatrixCal.h"
#include "QuadrantAngle.h"



void VehicleModel(double v, double w, double *previous_pose, double del_t, double *X) {
	double c1 = 0.000001;
	double c2 = 0.000001;
	double c3 = 0.000001;
	double c4 = 0.000001;

	double r[4] = {0,0,0,0};
	r[0] = (double)pow((c1 * fabs(v) + c2 * fabs(w)), 2);
	r[3] = (double)pow((c3 * fabs(v) + c4 * fabs(w)), 2);

	double ran[2];
	mvnrnd2(v, w, r, ran);
	double v_actual = ran[0];
	double w_actual = ran[1];

	double change_pose[3];
	change_pose[0] = v_actual * del_t * cos(previous_pose[2] + w_actual * del_t);
	change_pose[1] = v_actual * del_t * sin(previous_pose[2] + w_actual * del_t);
	change_pose[2] = w_actual * del_t;

	for (int i = 0; i < 3; i++) {
		X[i] = previous_pose[i] + change_pose[i];
	}
	Qangle(&X[2]);
}

void VehicleModel3(double u, double v, double r, double* previous_pose, double del_t, double* X) {
	//double c1 = 1.0;
	//double c2 = 1.0;
	//double c3 = 0.1;


	double g[9] = { 0 };
	g[0] = 0.001 * fabs(u) + 0.001;
	g[4] = 0.001 * fabs(v) + 0.001;
	g[8] = 0.001 * fabs(r) + 0.001;

	double mu[3];
	mu[0] = u;
	mu[1] = v;
	mu[2] = r;
	double ran[3];
	mvnrnd3(mu, g, ran);
	double u_actual = ran[0];
	double v_actual = ran[1];
	double r_actual = ran[2];

	double change_pose[3];
	change_pose[0] = (u_actual * cos(previous_pose[2]) - v_actual * sin(previous_pose[2])) * del_t;
	change_pose[1] = (u_actual * sin(previous_pose[2]) + v_actual * cos(previous_pose[2])) * del_t;
	change_pose[2] = r_actual * del_t;

	for (int i = 0; i < 3; i++) {
		X[i] = previous_pose[i] + change_pose[i];
	}
	Qangle(&X[2]);
}

void VehicleModel3LOC(double u, double v, double r, double* previous_pose, double del_t, double* X) {

	double g[9] = { 0 };
	g[0] = 0.0 * fabs(u) + 0.01;
	g[4] = 0.0 * fabs(v) + 0.001;
	g[8] = 0.0001;

	double mu[3];
	mu[0] = u;
	mu[1] = v;
	mu[2] = r;
	double ran[3];
	mvnrnd3(mu, g, ran);
	double u_actual = ran[0];
	double v_actual = ran[1];
	double r_actual = ran[2];

	double change_pose[3];
	change_pose[0] = (u_actual * cos(previous_pose[2]) - v_actual * sin(previous_pose[2])) * del_t;
	change_pose[1] = (u_actual * sin(previous_pose[2]) + v_actual * cos(previous_pose[2])) * del_t;
	change_pose[2] = r_actual * del_t;

	for (int i = 0; i < 3; i++) {
		X[i] = previous_pose[i] + change_pose[i];
	}
	Qangle(&X[2]);
}

