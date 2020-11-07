#ifndef INCLUDED_MATH_MATRIX33_H
#define INCLUDED_MATH_MATRIX33_H

namespace Math {

	class Vector2;

	class Matrix33 {//数学に合わせて右手座標系を使用
	public:
		Matrix33();
		Matrix33(const Matrix33& a);
		~Matrix33();
		Matrix33& operator=(const Matrix33& a);
		//単位行列
		void set_identity();
		//転置
		void set_transposition();
		//逆行列
		void set_inverse();
		//変換行列
		void set_translate(const Vector2& t);
		void set_rotate(float deg);
		void set_scale(const Vector2& s);
		//行列掛け合わせ
		Matrix33& operator*=(const Matrix33& m);
		void multiply(
			Vector2* out,
			const Vector2& in);
		//ワールド行列を法線用に変換する
		void world_mat_convert_for_normal();

		float m[3][3];
	};

}//namespace Math

#endif