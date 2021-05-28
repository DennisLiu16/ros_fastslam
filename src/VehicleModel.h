#pragma once

void VehicleModel(double v, double w, double* previous_pose, double del_t, double* X);
void VehicleModel3(double u, double v, double r, double* previous_pose, double del_t, double* X);
void VehicleModel3LOC(double u, double v, double r, double* previous_pose, double del_t, double* X);
