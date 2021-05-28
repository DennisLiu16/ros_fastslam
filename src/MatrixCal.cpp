#include <stdio.h>
#include "QuadrantAngle.h"
#include <random>
#include <cmath>

std::default_random_engine generator;
std::normal_distribution<double> distribution(0.0, 1.0);

void mattimes(double* matrix_a, int row_a, int col_a, double* matrix_b, int row_b, int col_b, double* matrix_ans) {
	if (col_a != row_b) {
		printf("matrix dimension error\r\n");
	}
	for (int uu = 0; uu < row_a * col_b; uu++) {
		matrix_ans[uu] = 0;
	}

	int num = 0;
	for (int i = 0; i < row_a; i++) {
		for (int j = 0; j < col_b; j++) {
			for (int k = 0; k < col_a; k++) {
				matrix_ans[num] = matrix_ans[num] + matrix_a[k + i * col_a] * matrix_b[k * col_b + j];
			}
			num++;
		}
	}
}

void transpose(double* mat, int row, int col, double* mat_ans) {
	//	for(int uu = 0; uu < row*col; uu++){
	//		printf("%d\r\n",(int)(mat[uu]*10));
	//	}
	for (int i = 0; i < row; i++) {
		for (int j = 0; j < col; j++) {
			mat_ans[j * row + i] = mat[i * col + j];
		}
	}
}

void matplus(double* mat_a, int row, int col, double* mat_b, double* mat_ans) {
	for (int i = 0; i < row; i++) {
		for (int j = 0; j < col; j++) {
			mat_ans[i * col + j] = mat_a[i * col + j] + mat_b[i * col + j];
		}
	}
}

void inv3(double* S, double* mat_ans) {	//just for 3*3 matrix
	double det1 = S[0] * S[4] * S[8] + S[1] * S[5] * S[6] + S[2] * S[3] * S[7];
	double det2 = S[2] * S[4] * S[6] + S[0] * S[5] * S[7] + S[1] * S[3] * S[8];
	double det = det1 - det2;

	if (det < 1e-12) {
		printf("matrix is singular inv3\r\n");
	}
	mat_ans[0] = (S[4] * S[8] - S[5] * S[7]) / det;
	mat_ans[1] = -(S[1] * S[8] - S[2] * S[7]) / det;
	mat_ans[2] = (S[1] * S[5] - S[2] * S[4]) / det;
	mat_ans[3] = -(S[3] * S[8] - S[5] * S[6]) / det;
	mat_ans[4] = (S[0] * S[8] - S[2] * S[6]) / det;
	mat_ans[5] = -(S[0] * S[5] - S[2] * S[3]) / det;
	mat_ans[6] = (S[3] * S[7] - S[4] * S[6]) / det;
	mat_ans[7] = -(S[0] * S[7] - S[1] * S[6]) / det;
	mat_ans[8] = (S[0] * S[4] - S[1] * S[3]) / det;
}

void inv2(double* S, double* mat_ans) {
	double det = (double)S[0] * (double)S[3] - (double)S[1] * (double)S[2];
	if (det < 1e-12) {
		printf("matrix is singular inv2\r\n");
	}
	mat_ans[0] = (double)(S[3] / det);
	mat_ans[1] = (double)(-S[1] / det);
	mat_ans[2] = (double)(-S[2] / det);
	mat_ans[3] = (double)(S[0] / det);
}

double distance2(double X1, double Y1, double X2, double Y2) {
	double ans = 0;
	ans = (double)sqrt(pow((X2 - X1), 2) + pow((Y2 - Y1), 2));
	return ans;
}

double rad2deg(double rad) {
	double ans = 0;
	ans = rad * 180.0f / PI;
	return ans;
}

void mvnrnd2(double v, double w, double* r, double* ans) {
	// calcilate G  sigma = G*G'
	double g11 = (double)sqrt(r[0]);
	double g21 = g11 == 0 ? 0 : r[1] / g11;
	double g22 = (double)sqrt(r[3] - pow(g21, 2));
	double G[4];
	G[0] = g11;
	G[1] = 0;
	G[2] = g21;
	G[3] = g22;
	double number[2];
	//double number = distribution(generator);
	number[0] = (double)distribution(generator);
	number[1] = (double)distribution(generator);
	double Gnum[2];

	mattimes(G, 2, 2, number, 2, 1, Gnum);
	ans[0] = v + Gnum[0];
	ans[1] = w + Gnum[1];
}

void mvnrnd3(double* mu, double* cor, double* ans) {
	double g11 = sqrt(cor[0]);
	double g21 = g11 == 0 ? 0 : cor[1] / g11;
	double g22 = sqrt(cor[4] - pow(g21, 2));
	double g31 = g11 == 0 ? 0 : cor[2] / g11;
	double g32 = g22 == 0 ? 0 : (cor[5] - g21 * g31) / g22;
	double g33 = sqrt(cor[8] - pow(g31, 2) - pow(g32, 2));
	double G[9] = { 0 };
	G[0] = g11;
	G[3] = g21;
	G[4] = g22;
	G[6] = g31;
	G[7] = g32;
	G[8] = g33;

	double number[3];
	number[0] = (double)distribution(generator);
	number[1] = (double)distribution(generator);
	number[2] = (double)distribution(generator);
	double Gnum[3];
	mattimes(G, 3, 3, number, 3, 1, Gnum);

	for (int i = 0; i < 3; i++) {
		ans[i] = mu[i] + Gnum[i];
	}
}
