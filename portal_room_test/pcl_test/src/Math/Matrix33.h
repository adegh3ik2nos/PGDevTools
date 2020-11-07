#ifndef INCLUDED_MATH_MATRIX33_H
#define INCLUDED_MATH_MATRIX33_H

namespace Math {

	class Vector2;

	class Matrix33 {//���w�ɍ��킹�ĉE����W�n���g�p
	public:
		Matrix33();
		Matrix33(const Matrix33& a);
		~Matrix33();
		Matrix33& operator=(const Matrix33& a);
		//�P�ʍs��
		void set_identity();
		//�]�u
		void set_transposition();
		//�t�s��
		void set_inverse();
		//�ϊ��s��
		void set_translate(const Vector2& t);
		void set_rotate(float deg);
		void set_scale(const Vector2& s);
		//�s��|�����킹
		Matrix33& operator*=(const Matrix33& m);
		void multiply(
			Vector2* out,
			const Vector2& in);
		//���[���h�s���@���p�ɕϊ�����
		void world_mat_convert_for_normal();

		float m[3][3];
	};

}//namespace Math

#endif