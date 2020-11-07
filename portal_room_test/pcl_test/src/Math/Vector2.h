#ifndef INCLUDED_MATH_VECTOR2_H
#define INCLUDED_MATH_VECTOR2_H

namespace Math {

	class Vector2 {
	public:
		//����
		Vector2();
		Vector2(int ax, int ay);
		Vector2(float ax, float ay);
		Vector2(const Vector2& a);
		~Vector2();
		//������Z
		Vector2& operator=(const Vector2& a);
		Vector2& operator+=(const Vector2& a);
		Vector2& operator-=(const Vector2& a);
		Vector2& operator*=(const Vector2& a);
		Vector2& operator/=(const Vector2& a);
		Vector2& operator*=(float a);
		Vector2& operator/=(float a);
		//�l��Ԃ����Z
		const Vector2 operator+(const Vector2& a) const;
		const Vector2 operator-(const Vector2& a) const;
		const Vector2 operator*(const Vector2& a) const;
		const Vector2 operator/(const Vector2& a) const;
		//�}�C�i�X�P��
		const Vector2 operator-() const;
		//����
		float dot(const Vector2& a) const;
		//�O��
		void set_cross(
			const Vector2& a,
			const Vector2& b);
		//���K�����đ��
		void set_normalize();
		//���K��
		const Vector2 normalize() const;
		//�傫���̓��
		float square_length() const;
		//�傫��
		float length() const;

		float x, y;
	};
	//�l��Ԃ����Z
	const Vector2 operator*(const Vector2& a, float b);
	const Vector2 operator*(float a, const Vector2& b);
	const Vector2 operator/(const Vector2& a, float b);
	const Vector2 operator/(float a, const Vector2& b);

}//namespace Math

#endif