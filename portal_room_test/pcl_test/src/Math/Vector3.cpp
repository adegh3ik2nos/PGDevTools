#include "src/Math/Vector3.h"

#include "src/Math/Math.h"

namespace Math {

	Vector3::Vector3() {}
	Vector3::Vector3(float ax, float ay, float az) :
		x(ax),
		y(ay),
		z(az) {}
	Vector3::Vector3(const Vector3& a) :
		x(a.x),
		y(a.y),
		z(a.z) {}
	Vector3::~Vector3() {}
	Vector3& Vector3::operator=(const Vector3& a) {
		x = a.x;
		y = a.y;
		z = a.z;

		return *this;
	}
	Vector3& Vector3::operator+=(const Vector3& a) {
		x += a.x;
		y += a.y;
		z += a.z;

		return *this;
	}
	Vector3& Vector3::operator-=(const Vector3& a) {
		x -= a.x;
		y -= a.y;
		z -= a.z;

		return *this;
	}
	Vector3& Vector3::operator*=(const Vector3& a) {
		x *= a.x;
		y *= a.y;
		z *= a.z;

		return *this;
	}
	Vector3& Vector3::operator/=(const Vector3& a) {
		x /= a.x;
		y /= a.y;
		z /= a.z;

		return *this;
	}
	Vector3& Vector3::operator*=(float a) {
		x *= a;
		y *= a;
		z *= a;

		return *this;
	}
	Vector3& Vector3::operator/=(float a) {
		float div = 1.f / a;
		x *= div;
		y *= div;
		z *= div;

		return *this;
	}
	const Vector3 Vector3::operator+(const Vector3& a) const {
		return Vector3(x + a.x, y + a.y, z + a.z);
	}
	const Vector3 Vector3::operator-(const Vector3& a) const {
		return Vector3(x - a.x, y - a.y, z - a.z);
	}
	const Vector3 Vector3::operator*(const Vector3& a) const {
		return Vector3(x * a.x, y * a.y, z * a.z);
	}
	const Vector3 Vector3::operator/(const Vector3& a) const {
		return Vector3(x / a.x, y / a.y, z / a.z);
	}
	const Vector3 Vector3::operator-() const {
		return Vector3(-x, -y, -z);
	}
	float Vector3::dot(const Vector3& a) const {
		return x*a.x + y*a.y + z*a.z;
	}
	const Vector3 Vector3::cross(const Vector3& a) const {
		Vector3 rtv;
		rtv.x = y*a.z - z*a.y;
		rtv.y = z*a.x - x*a.z;
		rtv.z = x*a.y - y*a.x;
		return rtv;
	}
	void Vector3::set_cross(const Vector3& a, const Vector3& b) {
		x = a.y*b.z - a.z*b.y;
		y = a.z*b.x - a.x*b.z;
		z = a.x*b.y - a.y*b.x;
	}
	void Vector3::set_normalize() {
		float len = 1.f / length();
		x *= len;
		y *= len;
		z *= len;
	}
	const Vector3 Vector3::normalize() const {
		float len = 1.f / length();
		return Vector3(x * len, y * len, z * len);
	}
	float Vector3::square_length() const {
		return x*x + y*y + z*z;
	}
	float Vector3::length() const {
		return Sqrt(square_length());
	}

	//‚±‚±‚©‚çƒNƒ‰ƒXŠO
	const Vector3 operator*(const Vector3& a, float b) {
		return Vector3(a.x * b, a.y * b, a.z * b);
	}
	const Vector3 operator*(float a, const Vector3& b) {
		return Vector3(a * b.x, a * b.y, a * b.z);
	}
	const Vector3 operator/(const Vector3& a, float b) {
		return Vector3(a.x / b, a.y / b, a.z / b);
	}
	const Vector3 operator/(float a, const Vector3& b) {
		return Vector3(a / b.x, a / b.y, a / b.z);
	}

}//namespace Math