#include "Math/Math.h"

#include <math.h>

#include "Math/Vector3.h"

namespace Math {

	float To_Deg(float rad) {
		return rad * to_deg;
	}
	float To_Rad(float deg) {
		return deg * to_rad;
	}

	float Sin(float deg) {
		return sinf(To_Rad(deg));
	}
	float Cos(float deg) {
		return cosf(To_Rad(deg));
	}
	float Tan(float deg) {
		return tanf(To_Rad(deg));
	}
	float Asin(float a) {
		return To_Deg(asinf(a));
	}
	float Acos(float a) {
		return To_Deg(acosf(a));
	}
	float Atan2(float y, float x) {
		return To_Deg(atan2f(y, x));
	}
	float Sqrt(float a) {
		return sqrtf(a);
	}
	float Fabs(float a) {
		return fabsf(a);
	}
	float Pow(float n, float p) {
		return powf(n, p);
	}
	int Pow(int n, int p) {
		int rtv = 1;
		for (int i = 0; i < p; ++i) { rtv *= n; }
		return rtv;
	}
	float Exp(float x) {
		return expf(x);
	}

}//namespace Math