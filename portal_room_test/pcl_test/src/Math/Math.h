#ifndef INCLUDED_MATH_MATH_H
#define INCLUDED_MATH_MATH_H

namespace Math {
	class Vector3;
}

namespace Math{

	//円周率
	static const float pi = 3.14159265358979323846f;
	//角度の単位変換に使う
	static const float to_deg = 180.f / pi;
	static const float to_rad = pi / 180.f;

	//ラジアンから度に直す
	float To_Deg(float rad);
	//度からラジアンに直す
	float To_Rad(float deg);

	//度をもらうsin関数
	float Sin(float deg);
	//度をもらうcos関数
	float Cos(float deg);
	//度をもらうtan関数
	float Tan(float deg);
	//floatをもらうasin関数
	float Asin(float a);
	//floatをもらうacos関数
	float Acos(float a);
	//floatをもらい度を返すatan2関数
	float Atan2(float y, float x);
	//floatをもらうsqrt関数
	float Sqrt(float a);
	//floatをもらうfabs関数
	float Fabs(float a);
	//floatをもらうpow関数 nのp乗
	float Pow(float n, float p);
	//intをもらうpow関数 nのp乗
	int Pow(int n, int p);
	//floatをもらうexp関数
	float Exp(float x);

}//namespace Math

#endif