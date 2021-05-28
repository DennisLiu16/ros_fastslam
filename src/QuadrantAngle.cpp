#include "QuadrantAngle.h"

void Qangle(double* angle) {
	while (*angle > PI || *angle <= -PI) {
		if (*angle > 0) {
			*angle = *angle - 2 * PI;
		}
		else {
			*angle = *angle + 2 * PI;
		}
	}
}
