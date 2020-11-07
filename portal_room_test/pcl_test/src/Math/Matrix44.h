#ifndef INCLUDED_MATH_MATRIX44_H
#define INCLUDED_MATH_MATRIX44_H

namespace Math {

	class Vector3;

	class Matrix44 {//右手系情報から左手系情報にする
	public:
		Matrix44();
		Matrix44(const Matrix44& a);
		~Matrix44();
		Matrix44& operator=(const Matrix44& a);
		//単位行列
		void set_identity();
		static Matrix44 identity()
		{
			Matrix44 rtv;
			rtv.set_identity();
			return rtv;
		}
		//転置
		void set_transposition();
		//逆行列
		void set_inverse();
		Matrix44& inverse()
		{
			set_inverse();
			return *this;
		}
		//変換行列
		void set_translate(const Vector3& a);
		void set_scale(const Vector3& a);
		void set_rotate_x(float deg);
		void set_rotate_y(float deg);
		void set_rotate_z(float deg);
		//2個目以降
		void translate(const Vector3& a);
		void scale(const Vector3& a);
		void rotate_x(float deg);
		void rotate_y(float deg);
		void rotate_z(float deg);
		//ビュー行列
		void set_view_transform(//右手系
			const Vector3& position,
			const Vector3& target);
		void set_view_transform(//上方方向ありの右手系
			const Vector3& position,
			const Vector3& target,
			const Vector3& up_dir);
		//透視行列
		void set_perspective_transform(//右手系
			float view_angle_y,
			float window_width,
			float window_height,
			float near_clip,
			float far_clip);
		//行列掛け合わせ
		Matrix44& operator*=(const Matrix44& a);
		const Matrix44 operator*(const Matrix44& a) const;
		//同じベクトルで問題なし
		void multiply(
			Vector3* out, 
			const Vector3& in) const;
		//float型の配列にコピー
		void copy_to_array(float (*ary)[4][4]) const;
		//ワールド行列を法線用に変換する
		void world_mat_convert_for_normal();

		float m[4][4];
	};

}//namespace Math

#endif