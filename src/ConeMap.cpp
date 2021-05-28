#include "ConeMap.h"
#include "Particle.h"
#include <cmath>
#include <stdio.h>

//Choose one
#define useBtrack 0
#define useEllipse 0
#define useEight 1

#if useBtrack
const int YellowNumber = 29;
const int BlueNumber = 19;
#endif

#if useEllipse
const int YellowNumber = 20;
const int BlueNumber = 10;
#endif

#if useEight
const int YellowNumber = 26;
const int BlueNumber = 14;
#endif

void ConeInit(ConeSet** ptr) {

#if useBtrack
	double coneYellowX[YellowNumber] = { 0,12, 0,12,3,5,7,9,11,12,12,11,10,11,12,12,11,9,7,5,3,1,0,0,0,0,0,0,1 };
	double coneYellowY[YellowNumber] = { 0, 0,27,27,1,0,0,1,3,6,9,12,14,16,19,21,24,26,27,27,26,24,21,18,15,12,9,6,3 };

	double coneBlueX[BlueNumber] = { 4,6,8,9,9,8,7,8,9,9,8,6,4,3,3,3,3,3,3 };
	double coneBlueY[BlueNumber] = { 4,3,4,6,9,11,14,17,19,21,23,24,23,21,18,15,12,9,6 };
#endif

#if useEllipse
	double coneYellowX[YellowNumber] = { 3,5,7,9,11,12,12,11,9,7,5,3,1,0,0,1,0,12,12,0 };
	double coneYellowY[YellowNumber] = { 1,0,0,1,3,6,9,12,14,15,15,14,12,9,6,3,0,0,15,15 };

	double coneBlueX[BlueNumber] = { 4,6,8,9,9,8,6,4,3,3 };
	double coneBlueY[BlueNumber] = { 4,3,4,6,9,11,12,11,9,6 };
#endif

#if useEight
	double coneYellowX[YellowNumber] = {1,1,1,1,2,2,2,3,3,5,5,7,7,9,9,10,10,10,11,11,11,11,0,12,12,0};
	double coneYellowY[YellowNumber] = {4,6,8,10,2,7,12,1,13,0,14,0,14,1,13,2,7,12,4,6,8,10,0,0,15,15 };

	double coneBlueX[BlueNumber] = { 3,3,4,4,4,4,6,6,8,8,8,8,9,9 };
	double coneBlueY[BlueNumber] = { 5,9,3,6,8,11,2,12,3,6,8,11,5,9 };
#endif

	double LB = 0.395;
	double bias = 12.0;
	double corLimit = 0.001;

	struct ConeSet* first = NULL;
	struct ConeSet* current = NULL;
	current = first;

	for (int i = 0; i < (YellowNumber + BlueNumber); i++) {
		struct ConeSet* cone_data;
		cone_data = (struct ConeSet*)malloc(sizeof(struct ConeSet));
		if (cone_data) {
			if (i < YellowNumber) {
				cone_data->mu[0] = coneYellowX[i] * LB + bias;
				cone_data->mu[1] = coneYellowY[i] * LB + bias;
				cone_data->color = 1;
			}
			else
			{
				cone_data->mu[0] = coneBlueX[i - YellowNumber] * LB + bias;
				cone_data->mu[1] = coneBlueY[i - YellowNumber] * LB + bias;
				cone_data->color = 0;
			}
			for (int j = 0; j < 4; j++) {
				if (j == 0 || j == 3) cone_data->cor[j] = corLimit;
				else cone_data->cor[j] = 0.0;
			}
			
		}


		if (i == 0) {
			if (first == NULL) {
				first = cone_data;
				current = cone_data;
			}
			else printf("first error ConeMap.cpp\r\n");
		}
		else {
			if (cone_data) {
				current->next = cone_data;
				current = current->next;
			}
			else  printf("current error ConeMap.cpp\r\n");
		}
	}

	*ptr = first;
}

