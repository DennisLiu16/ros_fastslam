#pragma once

void mattimes(double* matrix_a, int row_a, int col_a, double* matrix_b, int row_b, int col_b, double* matrix_ans);
void transpose(double* mat, int row, int col, double* mat_ans);
void matplus(double* mat_a, int row, int col, double* mat_b, double* mat_ans);
void inv3(double* S, double* mat_ans);
double distance2(double X1, double Y1, double X2, double Y2);
double rad2deg(double rad);
void mvnrnd2(double v, double w, double* r, double* ans);
void inv2(double* S, double* mat_ans);
void mvnrnd3(double* mu, double* cor, double* ans);
