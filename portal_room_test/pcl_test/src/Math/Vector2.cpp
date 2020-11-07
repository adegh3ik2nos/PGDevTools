#include "Math/Vector2.h"

#include "Math/Math.h"

namespace Math {

	Vector2::Vector2() {}
	Vector2::Vector2(int ax, int ay) :
		x(static_cast<float>(ax)),
		y(static_cast<float>(ay)) {}
	Vector2::Vector2(float ax, float ay) :
		x(ax),
		y(ay) {}
	Vector2::Vector2(const Vector2& a) :
		x(a.x),
		y(a.y) {}
	Vector2::~Vector2(){}
	Vector2& Vector2::operator=(const Vector2& a) {
		x = a.x;
		y = a.y;

		return *this;
	}
	Vector2& Vector2::operator+=(const Vector2& a) {
		x += a.x;
		y += a.y;

		return *this;
	}
	Vector2& Vector2::operator-=(const Vector2& a) {
		x -= a.x;
		y -= a.y;

		return *this;
	}
	Vector2& Vector2::operator*=(const Vector2& a) {
		x *= a.x;
		y *= a.y;

		return *this;
	}
	Vector2& Vector2::operator/=(const Vector2& a) {
		x /= a.x;
		y /= a.y;

		return *this;
	}
	Vector2& Vector2::operator*=(float a) {
		x *= a;
		y *= a;

		return *this;
	}
	Vector2& Vector2::operator/=(float a) {
		float div = 1.f / a;
		x *= div;
		y *= div;

		return *this;
	}
	const Vector2 Vector2::operator+(const Vector2& a) const {
		return Vector2(x + a.x, y + a.y);
	}
	const Vector2 Vector2::operator-(const Vector2& a) const {
		return Vector2(x - a.x, y - a.y);
	}
	const Vector2 Vector2::operator*(const Vector2& a) const {
		return Vector2(x * a.x, y * a.y);
	}
	const Vector2 Vector2::operator/(const Vector2& a) const {
		return Vector2(x / a.x, y / a.y);
	}
	const Vector2 Vector2::operator-() const {
		return Vector2(-x, -y);
	}
	float Vector2::dot(const Vector2& a) const {
		return x*a.x + y*a.y;
	}
	void Vector2::set_cross(const Vector2& a, const Vector2& b) {
		x = y;
		y = -x;
	}
	void Vector2::set_normalize() {
		float len = 1.f / length();
		x *= len;
		y *= len;
	}
	const Vector2 Vector2::normalize() const {
		float len = 1.f / length();
		return Vector2(x * len, y * len);
	}
	float Vector2::square_length() const {
		return x*x + y*y;
	}
	float Vector2::length() const {
		return Sqrt(square_length());
	}

	//‚±‚±‚©‚çƒNƒ‰ƒXŠO
	const Vector2 operator*(const Vector2& a, float b) {
		return Vector2(a.x * b, a.y * b);
	}
	const Vector2 operator*(float a, const Vector2& b) {
		return Vector2(a * b.x, a * b.y);
	}
	const Vector2 operator/(const Vector2& a, float b) {
		return Vector2(a.x / b, a.y / b);
	}
	const Vector2 operator/(float a, const Vector2& b) {
		return Vector2(a / b.x, a / b.y);
	}

}//namespace Math