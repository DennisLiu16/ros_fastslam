#include "resample_weight.h"
#include <random>

std::default_random_engine uniform_seed;
std::uniform_real_distribution<double> uniform_distribution(0.0, 1.0);


void resample_weight(double* wt, int* re_sample_index,int M) {
	double total_weight = 0;
	for (int i = 0; i < M; i++) {
		total_weight = total_weight + wt[i];
	}
	
	for (int mm = 0; mm < M; mm++) {
		double re_weight = 0;
		double weight = 0;
		weight = uniform_distribution(uniform_seed) * total_weight;
		for (int i = 0; i < M; i++) {
			re_weight = re_weight + wt[i];
			if (re_weight > weight) {
				re_sample_index[mm] = i;
				break;
			}
		}
	}

}
