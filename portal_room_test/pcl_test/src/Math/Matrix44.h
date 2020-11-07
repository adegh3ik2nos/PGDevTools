#ifndef INCLUDED_MATH_MATRIX44_H
#define INCLUDED_MATH_MATRIX44_H

namespace Math {

	class Vector3;

	class Matrix44 {//�E��n��񂩂獶��n���ɂ���
	public:
		Matrix44();
		Matrix44(const Matrix44& a);
		~Matrix44();
		Matrix44& operator=(const Matrix44& a);
		//�P�ʍs��
		void set_identity();
		static Matrix44 identity()
		{
			Matrix44 rtv;
			rtv.set_identity();
			return rtv;
		}
		//�]�u
		void set_transposition();
		//�t�s��
		void set_inverse();
		Matrix44& inverse()
		{
			set_inverse();
			return *this;
		}
		//�ϊ��s��
		void set_translate(const Vector3& a);
		void set_scale(const Vector3& a);
		void set_rotate_x(float deg);
		void set_rotate_y(float deg);
		void set_rotate_z(float deg);
		//2�ڈȍ~
		void translate(const Vector3& a);
		void scale(const Vector3& a);
		void rotate_x(float deg);
		void rotate_y(float deg);
		void rotate_z(float deg);
		//�r���[�s��
		void set_view_transform(//�E��n
			const Vector3& position,
			const Vector3& target);
		void set_view_transform(//�����������̉E��n
			const Vector3& position,
			const Vector3& target,
			const Vector3& up_dir);
		//�����s��
		void set_perspective_transform(//�E��n
			float view_angle_y,
			float window_width,
			float window_height,
			float near_clip,
			float far_clip);
		//�s��|�����킹
		Matrix44& operator*=(const Matrix44& a);
		const Matrix44 operator*(const Matrix44& a) const;
		//�����x�N�g���Ŗ��Ȃ�
		void multiply(
			Vector3* out, 
			const Vector3& in) const;
		//float�^�̔z��ɃR�s�[
		void copy_to_array(float (*ary)[4][4]) const;
		//���[���h�s���@���p�ɕϊ�����
		void world_mat_convert_for_normal();

		float m[4][4];
	};

}//namespace Math

#endif