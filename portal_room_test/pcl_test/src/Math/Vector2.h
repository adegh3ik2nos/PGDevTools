#ifndef INCLUDED_MATH_VECTOR2_H
#define INCLUDED_MATH_VECTOR2_H

namespace Math {

	class Vector2 {
	public:
		//生成
		Vector2();
		Vector2(int ax, int ay);
		Vector2(float ax, float ay);
		Vector2(const Vector2& a);
		~Vector2();
		//代入演算
		Vector2& operator=(const Vector2& a);
		Vector2& operator+=(const Vector2& a);
		Vector2& operator-=(const Vector2& a);
		Vector2& operator*=(const Vector2& a);
		Vector2& operator/=(const Vector2& a);
		Vector2& operator*=(float a);
		Vector2& operator/=(float a);
		//値を返す演算
		const Vector2 operator+(const Vector2& a) const;
		const Vector2 operator-(const Vector2& a) const;
		const Vector2 operator*(const Vector2& a) const;
		const Vector2 operator/(const Vector2& a) const;
		//マイナス単項
		const Vector2 operator-() const;
		//内積
		float dot(const Vector2& a) const;
		//外積
		void set_cross(
			const Vector2& a,
			const Vector2& b);
		//正規化して代入
		void set_normalize();
		//正規化
		const Vector2 normalize() const;
		//大きさの二乗
		float square_length() const;
		//大きさ
		float length() const;

		float x, y;
	};
	//値を返す演算
	const Vector2 operator*(const Vector2& a, float b);
	const Vector2 operator*(float a, const Vector2& b);
	const Vector2 operator/(const Vector2& a, float b);
	const Vector2 operator/(float a, const Vector2& b);

}//namespace Math

#endif