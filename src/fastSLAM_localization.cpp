#include "fastSLAM_localization.h"
#include "VehicleModel.h"
#include <stdio.h>
#include "Particle.h"
#include "MatrixCal.h"
#include <cmath>
#include "QuadrantAngle.h"


void fast_slam_loc(double* zt, double* ut, double* St, int Nt_1, struct ConeSet* Cone_set, double* St_par, double* wt, int *localFlag, int color) {
	//////// parameters ////////
	// sensor model parameter
	//--const double Rt_disgain = 0.000001;				// Rt[0] = q * q * Rt_disgain;
	const double Rt_disgain = 0.005;
	//--const double Rt_degree = 0.000001;				// sqrt()=> deg
	const double Rt_degree = 0.005;				
	// the parameter of the cone is the same or different (association)
	//--const double observer_distance = 0.3;			// unit(meter)
	const double observer_distance = 0.3;
	//--const double observer_angle = 0.4;			// +-11 deg
	const double observer_angle = 0.4;
	// new cone pdf gain
	//--const double wn_percent = 0.01;
	const double wn_percent = 0.01;
	// if the cone color is wrong , the particle weight may divided something
	//--const double wn_divided = 0.01;
	const double wn_divided = 0.01;
	//////// parameters ////////

	double max_wn = 0.0;
	double max_num = 0.0;
	int max_cone_color = -1;
	int n_hat = -1;

	struct ConeSet* predict_Cone_set = Cone_set;
	
	////// Particle filter predict
	for (int n = 0; n < Nt_1; n++) {
		if (n != 0) {
			if (predict_Cone_set) predict_Cone_set = predict_Cone_set->next;	// pointer point to next cone data
			else printf("Cone_set error\n");
		}
		double cone_mu[2] = { 0 };												// cone X Y coordinate
		double cone_cor[4] = { 0 };												// cone covariance
		int cone_color = -1;
		if (predict_Cone_set) {
			cone_mu[0] = predict_Cone_set->mu[0];
			cone_mu[1] = predict_Cone_set->mu[1];
			cone_color = predict_Cone_set->color;
			for (int i = 0; i < 4; i++) {
				cone_cor[i] = predict_Cone_set->cor[i];
			}
		}
		double q = distance2(cone_mu[0], cone_mu[1], St[0], St[1]);				// Sensor model predict distance
		double zn_hat[2];														// Sensor model predict Phi
		zn_hat[0] = q;
		zn_hat[1] = atan2(cone_mu[1] - St[1], cone_mu[0] - St[0]) - St[2];
		Qangle(&zn_hat[1]);
		double G[4];															// Sensor model Jacobian 
		G[0] = (cone_mu[0] - St[0]) / q;
		G[1] = (cone_mu[1] - St[1]) / q;
		G[2] = -(cone_mu[1] - St[1]) / (q * q);
		G[3] = (cone_mu[0] - St[0]) / (q * q);
		double Rt[4] = { 0 };													// Sensor model noise covariance
		Rt[0] = q * q * Rt_disgain;
		Rt[3] = Rt_degree;

		double Gcor[4];
		mattimes(G, 2, 2, cone_cor, 2, 2, Gcor);
		double GT[4];
		transpose(G, 2, 2, GT);
		double GcorGT[4];
		mattimes(Gcor, 2, 2, GT, 2, 2, GcorGT);
		double Q[4];															// Error propagation covariance : mean Sensor model predict cone covariance
		matplus(GcorGT, 2, 2, Rt, Q);
		// calculate resample weight  
		double err[2];
		err[0] = zt[0] - zn_hat[0];
		err[1] = zt[1] - zn_hat[1];
		Qangle(&err[1]);
		
		double invQ[4];
		inv2(Q, invQ);
		double errTinvQ[2];
		mattimes(err, 1, 2, invQ, 2, 2, errTinvQ);
		double errTinvQerr[1];
		mattimes(errTinvQ, 1, 2, err, 2, 1, errTinvQerr);

		double detQ = Q[0] * Q[3] - Q[1] * Q[2];
		double num_wn = sqrt(2.0 * PI * detQ);
		double den_wn = exp(-0.5 * errTinvQerr[0]);
		double wn = den_wn / num_wn;
		//printf("%f\r\n", wn);
		if (wn >= max_wn) {
			max_wn = wn;
			n_hat = n;
			max_cone_color = cone_color;
			//printf("n=%f\n", den_wn);
		}
		if ((1.0/num_wn) >= max_num) max_num = 1.0/num_wn;
		//printf("%f,%f\r\n", num_wn, max_num);
	}
	double p0 = max_num * wn_percent;

	if (max_cone_color != color) {
		max_wn = max_wn * wn_divided;
	}
	else printf("color OK\n");
	*localFlag = max_wn > p0 ? 0 : 1;
	//printf("%f,%f\r\n", max_num,max_wn);
	//printf("%d\r\n", *localFlag);

	
	for (int i = 0; i < 3; i++) {
		St_par[i] = St[i];
	}
	*wt = max_wn;
}
