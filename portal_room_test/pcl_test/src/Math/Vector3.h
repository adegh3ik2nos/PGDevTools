#ifndef INCLUDED_MATH_VECTOR3_H
#define INCLUDED_MATH_VECTOR3_H

#include "Matrix44.h"
#include <pcl/point_types.h>  

namespace Math {

	class Vector3 {
	public:
		//����
		Vector3();
		Vector3(float ax, float ay, float az);
		Vector3(const Vector3& a);
		//�j��
		~Vector3();
		//������Z
		Vector3& operator=(const Vector3& a);
		Vector3& operator+=(const Vector3& a);
		Vector3& operator-=(const Vector3& a);
		Vector3& operator*=(const Vector3& a);
		Vector3& operator/=(const Vector3& a);
		Vector3& operator*=(float a);
		Vector3& operator/=(float a);

		Vector3& operator*=(const Matrix44& a)
		{
			a.multiply(this, *this);
			return *this;
		}
		
		pcl::PointXYZ ToPointXYZ() const
		{
			return pcl::PointXYZ(x, y, z);
		}
		//�l��Ԃ����Z
		const Vector3 operator+(const Vector3& a) const;
		const Vector3 operator-(const Vector3& a) const;
		const Vector3 operator*(const Vector3& a) const;
		const Vector3 operator/(const Vector3& a) const;
		//�}�C�i�X�P��
		const Vector3 operator-() const;
		//����
		float dot(const Vector3& a) const;
		//�O��
		const Vector3 cross(const Vector3& a) const;
		void set_cross(
			const Vector3& a, const Vector3& b);
		//���K�����đ��
		void set_normalize();
		//���K��
		const Vector3 normalize() const;
		//�傫���̓��
		float square_length() const;
		//�傫��
		float length() const;

		float x, y, z;
	};
	//�l��Ԃ����Z
	const Vector3 operator*(const Vector3& a, float b);
	const Vector3 operator*(float a, const Vector3& b);
	const Vector3 operator/(const Vector3& a, float b);
	const Vector3 operator/(float a, const Vector3& b);

}//namespace Math

#endif