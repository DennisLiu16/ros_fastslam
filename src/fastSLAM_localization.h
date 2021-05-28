#pragma once
void fast_slam_loc(double* zt, double* ut, double* St, int Nt_1, struct ConeSet* Cone_set, double* St_par, double* wt, int* localFlag, int cone_color);
