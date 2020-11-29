#include "src/Math/Matrix44.h"

#include <memory>

#include "src/Math/Vector3.h"
#include "src/Math/Math.h"

namespace Math {

	Matrix44::Matrix44() {}
	Matrix44::Matrix44(const Matrix44& a) {
		memcpy_s(m, sizeof(m), a.m, sizeof(a.m));
	}
	Matrix44::~Matrix44() {}
	Matrix44& Matrix44::operator=(const Matrix44& a) {
		memcpy_s(m, sizeof(m), a.m, sizeof(a.m));
		return *this;
	}
	void Matrix44::set_identity() {
		m[0][0] = m[1][1] = m[2][2] = m[3][3] = 1.f;
		m[0][1] = m[0][2] = m[0][3] = 0.f;
		m[1][0] = m[1][2] = m[1][3] = 0.f;
		m[2][0] = m[2][1] = m[2][3] = 0.f;
		m[3][0] = m[3][1] = m[3][2] = 0.f;
	}
	void Matrix44::set_transposition() {
		Matrix44 prev = *this;
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 4; ++j) {
				m[i][j] = prev.m[j][i];
			}
		}
	}
	void Matrix44::set_inverse() {
		float delta =
			m[0][0]*m[1][1]*m[2][2]*m[3][3] + m[0][0]*m[1][2]*m[2][3]*m[3][1] + 
			m[0][0]*m[1][3]*m[2][1]*m[3][2] + m[0][1]*m[1][0]*m[2][3]*m[3][2] + 
			m[0][1]*m[1][2]*m[2][0]*m[3][3] + m[0][1]*m[1][3]*m[2][2]*m[3][0] + 
			m[0][2]*m[1][0]*m[2][1]*m[3][3] + m[0][2]*m[1][1]*m[2][3]*m[3][0] + 
			m[0][2]*m[1][3]*m[2][0]*m[3][1] + m[0][3]*m[1][0]*m[2][2]*m[3][1] + 
			m[0][3]*m[1][1]*m[2][0]*m[3][2] + m[0][3]*m[1][2]*m[2][1]*m[3][0] - 
			m[0][0]*m[1][1]*m[2][3]*m[3][2] - m[0][0]*m[1][2]*m[2][1]*m[3][3] - 
			m[0][0]*m[1][3]*m[2][2]*m[3][1] - m[0][1]*m[1][0]*m[2][2]*m[3][3] - 
			m[0][1]*m[1][2]*m[2][3]*m[3][0] - m[0][1]*m[1][3]*m[2][0]*m[3][2] - 
			m[0][2]*m[1][0]*m[2][3]*m[3][1] - m[0][2]*m[1][1]*m[2][0]*m[3][3] - 
			m[0][2]*m[1][3]*m[2][1]*m[3][0] - m[0][3]*m[1][0]*m[2][1]*m[3][2] - 
			m[0][3]*m[1][1]*m[2][2]*m[3][0] - m[0][3]*m[1][2]*m[2][0]*m[3][1];
		//計算できない場合は終了
		if (delta == 0.f) return;
		//逆数にして計算負荷軽減
		delta = 1.f / delta;
		//逆行列計算
		Matrix44 prev = *this;
		m[0][0] =
			prev.m[1][1] * prev.m[2][2] * prev.m[3][3] +
			prev.m[1][2] * prev.m[2][3] * prev.m[3][1] +
			prev.m[1][3] * prev.m[2][1] * prev.m[3][2] -
			prev.m[1][1] * prev.m[2][3] * prev.m[3][2] -
			prev.m[1][2] * prev.m[2][1] * prev.m[3][3] -
			prev.m[1][3] * prev.m[2][2] * prev.m[3][1];
		m[0][1] =
			prev.m[0][1] * prev.m[2][3] * prev.m[3][2] +
			prev.m[0][2] * prev.m[2][1] * prev.m[3][3] +
			prev.m[0][3] * prev.m[2][2] * prev.m[3][1] -
			prev.m[0][1] * prev.m[2][2] * prev.m[3][3] -
			prev.m[0][2] * prev.m[2][3] * prev.m[3][1] -
			prev.m[0][3] * prev.m[2][1] * prev.m[3][2];
		m[0][2] =
			prev.m[0][1] * prev.m[1][2] * prev.m[3][3] +
			prev.m[0][2] * prev.m[1][3] * prev.m[3][1] +
			prev.m[0][3] * prev.m[1][1] * prev.m[3][2] -
			prev.m[0][1] * prev.m[1][3] * prev.m[3][2] -
			prev.m[0][2] * prev.m[1][1] * prev.m[3][3] -
			prev.m[0][3] * prev.m[1][2] * prev.m[3][1];
		m[0][3] =
			prev.m[0][1] * prev.m[1][3] * prev.m[2][2] +
			prev.m[0][2] * prev.m[1][1] * prev.m[2][3] +
			prev.m[0][3] * prev.m[1][2] * prev.m[2][1] -
			prev.m[0][1] * prev.m[1][2] * prev.m[2][3] -
			prev.m[0][2] * prev.m[1][3] * prev.m[2][1] -
			prev.m[0][3] * prev.m[1][1] * prev.m[2][2];
		m[1][0] =
			prev.m[1][0] * prev.m[2][3] * prev.m[3][2] +
			prev.m[1][2] * prev.m[2][0] * prev.m[3][3] +
			prev.m[1][3] * prev.m[2][2] * prev.m[3][0] -
			prev.m[1][0] * prev.m[2][2] * prev.m[3][3] -
			prev.m[1][2] * prev.m[2][3] * prev.m[3][0] -
			prev.m[1][3] * prev.m[2][0] * prev.m[3][2];
		m[1][1] =
			prev.m[0][0] * prev.m[2][2] * prev.m[3][3] +
			prev.m[0][2] * prev.m[2][3] * prev.m[3][0] +
			prev.m[0][3] * prev.m[2][0] * prev.m[3][2] -
			prev.m[0][0] * prev.m[2][3] * prev.m[3][2] -
			prev.m[0][2] * prev.m[2][0] * prev.m[3][3] -
			prev.m[0][3] * prev.m[2][2] * prev.m[3][0];
		m[1][2] =
			prev.m[0][0] * prev.m[1][3] * prev.m[3][2] +
			prev.m[0][2] * prev.m[1][0] * prev.m[3][3] +
			prev.m[0][3] * prev.m[1][2] * prev.m[3][0] -
			prev.m[0][0] * prev.m[1][2] * prev.m[3][3] -
			prev.m[0][2] * prev.m[1][3] * prev.m[3][0] -
			prev.m[0][3] * prev.m[1][0] * prev.m[3][2];
		m[1][3] =
			prev.m[0][0] * prev.m[1][2] * prev.m[2][3] +
			prev.m[0][2] * prev.m[1][3] * prev.m[2][0] +
			prev.m[0][3] * prev.m[1][0] * prev.m[2][2] -
			prev.m[0][0] * prev.m[1][3] * prev.m[2][2] -
			prev.m[0][2] * prev.m[1][0] * prev.m[2][3] -
			prev.m[0][3] * prev.m[1][2] * prev.m[2][0];
		m[2][0] =
			prev.m[1][0] * prev.m[2][1] * prev.m[3][3] +
			prev.m[1][1] * prev.m[2][3] * prev.m[3][0] +
			prev.m[1][3] * prev.m[2][0] * prev.m[3][1] -
			prev.m[1][0] * prev.m[2][3] * prev.m[3][1] -
			prev.m[1][1] * prev.m[2][0] * prev.m[3][3] -
			prev.m[1][3] * prev.m[2][1] * prev.m[3][0];
		m[2][1] =
			prev.m[0][0] * prev.m[2][3] * prev.m[3][1] +
			prev.m[0][1] * prev.m[2][0] * prev.m[3][3] +
			prev.m[0][3] * prev.m[2][1] * prev.m[3][0] -
			prev.m[0][0] * prev.m[2][1] * prev.m[3][3] -
			prev.m[0][1] * prev.m[2][3] * prev.m[3][0] -
			prev.m[0][3] * prev.m[2][0] * prev.m[3][1];
		m[2][2] =
			prev.m[0][0] * prev.m[1][1] * prev.m[3][3] +
			prev.m[0][1] * prev.m[1][3] * prev.m[3][0] +
			prev.m[0][3] * prev.m[1][0] * prev.m[3][1] -
			prev.m[0][0] * prev.m[1][3] * prev.m[3][1] -
			prev.m[0][1] * prev.m[1][0] * prev.m[3][3] -
			prev.m[0][3] * prev.m[1][1] * prev.m[3][0];
		m[2][3] =
			prev.m[0][0] * prev.m[1][3] * prev.m[2][1] +
			prev.m[0][1] * prev.m[1][0] * prev.m[2][3] +
			prev.m[0][3] * prev.m[1][1] * prev.m[2][0] -
			prev.m[0][0] * prev.m[1][1] * prev.m[2][3] -
			prev.m[0][1] * prev.m[1][3] * prev.m[2][0] -
			prev.m[0][3] * prev.m[1][0] * prev.m[2][1];
		m[3][0] =
			prev.m[1][0] * prev.m[2][2] * prev.m[3][1] +
			prev.m[1][1] * prev.m[2][0] * prev.m[3][2] +
			prev.m[1][2] * prev.m[2][1] * prev.m[3][0] -
			prev.m[1][0] * prev.m[2][1] * prev.m[3][2] -
			prev.m[1][1] * prev.m[2][2] * prev.m[3][0] -
			prev.m[1][2] * prev.m[2][0] * prev.m[3][1];
		m[3][1] =
			prev.m[0][0] * prev.m[2][1] * prev.m[3][2] +
			prev.m[0][1] * prev.m[2][2] * prev.m[3][0] +
			prev.m[0][2] * prev.m[2][0] * prev.m[3][1] -
			prev.m[0][0] * prev.m[2][2] * prev.m[3][1] -
			prev.m[0][1] * prev.m[2][0] * prev.m[3][2] -
			prev.m[0][2] * prev.m[2][1] * prev.m[3][0];
		m[3][2] =
			prev.m[0][0] * prev.m[1][2] * prev.m[3][1] +
			prev.m[0][1] * prev.m[1][0] * prev.m[3][2] +
			prev.m[0][2] * prev.m[1][1] * prev.m[3][0] -
			prev.m[0][0] * prev.m[1][1] * prev.m[3][2] -
			prev.m[0][1] * prev.m[1][2] * prev.m[3][0] -
			prev.m[0][2] * prev.m[1][0] * prev.m[3][1];
		m[3][3] =
			prev.m[0][0] * prev.m[1][1] * prev.m[2][2] +
			prev.m[0][1] * prev.m[1][2] * prev.m[2][0] +
			prev.m[0][2] * prev.m[1][0] * prev.m[2][1] -
			prev.m[0][0] * prev.m[1][2] * prev.m[2][1] -
			prev.m[0][1] * prev.m[1][0] * prev.m[2][2] -
			prev.m[0][2] * prev.m[1][1] * prev.m[2][0];
	}
	void Matrix44::set_translate(const Vector3& a) {
		m[0][0] = m[1][1] = m[2][2] = m[3][3] = 1.f;
		m[0][3] = a.x;
		m[1][3] = a.y;
		m[2][3] = a.z;
		m[0][1] = m[0][2] = 0.f;
		m[1][0] = m[1][2] = 0.f;
		m[2][0] = m[2][1] = 0.f;
		m[3][0] = m[3][1] = m[3][2] = 0.f;

	}
	void Matrix44::set_scale(const Vector3& a) {
		m[0][0] = a.x;
		m[1][1] = a.y;
		m[2][2] = a.z;
		m[3][3] = 1.f;
		m[0][1] = m[0][2] = m[0][3] = 0.f;
		m[1][0] = m[1][2] = m[1][3] = 0.f;
		m[2][0] = m[2][1] = m[2][3] = 0.f;
		m[3][0] = m[3][1] = m[3][2] = 0.f;
	}
	void Matrix44::set_rotate_x(float deg) {
		float s = Sin(deg);
		float c = Cos(deg);
		m[1][1] = m[2][2] = c;
		m[0][0] = m[3][3] = 1.f;
		m[1][2] = -s;
		m[2][1] = s;
		m[0][1] = m[0][2] = m[0][3] = 0.f;
		m[1][0] = m[1][3] = 0.f;
		m[2][0] = m[2][3] = 0.f;
		m[3][0] = m[3][1] = m[3][2] = 0.f;
	}
	void Matrix44::set_rotate_y(float deg) {
		float s = Sin(deg);
		float c = Cos(deg);
		m[0][0] = m[2][2] = c;
		m[1][1] = m[3][3] = 1.f;
		m[2][0] = -s;
		m[0][2] = s;
		m[0][1] = m[0][3] = 0.f;
		m[1][0] = m[1][2] = m[1][3] = 0.f;
		m[2][1] = m[2][3] = 0.f;
		m[3][0] = m[3][1] = m[3][2] = 0.f;
	}
	void Matrix44::set_rotate_z(float deg) {
		float s = Sin(deg);
		float c = Cos(deg);
		m[0][0] = m[1][1] = c;
		m[2][2] = m[3][3] = 1.f;
		m[0][1] = -s;
		m[1][0] = s;
		m[0][2] = m[0][3] = 0.f;
		m[1][2] = m[1][3] = 0.f;
		m[2][0] = m[2][1] = m[2][3] = 0.f;
		m[3][0] = m[3][1] = m[3][2] = 0.f;
	}
	void Matrix44::translate(const Vector3& a) {
		m[0][3] += m[0][0] * a.x + m[0][1] * a.y + m[0][2] * a.z;
		m[1][3] += m[1][0] * a.x + m[1][1] * a.y + m[1][2] * a.z;
		m[2][3] += m[2][0] * a.x + m[2][1] * a.y + m[2][2] * a.z;
	}
	void Matrix44::scale(const Vector3& a) {
		m[0][0] *= a.x;
		m[0][1] *= a.y;
		m[0][2] *= a.z;
		m[1][0] *= a.x;
		m[1][1] *= a.y;
		m[1][2] *= a.z;
		m[2][0] *= a.x;
		m[2][1] *= a.y;
		m[2][2] *= a.z;
	}
	void Matrix44::rotate_x(float deg) {
		float s = Sin(deg);
		float c = Cos(deg);
		float t;
		t = c * m[0][1] + s * m[0][2];
		m[0][2] = -s * m[0][1] + c * m[0][2];
		m[0][1] = t;
		t = c * m[1][1] + s * m[1][2];
		m[1][2] = -s * m[1][1] + c * m[1][2];
		m[1][1] = t;
		t = c * m[2][1] + s * m[2][2];
		m[2][2] = -s * m[2][1] + c * m[2][2];
		m[2][1] = t;
	}
	void Matrix44::rotate_y(float deg) {
		float s = Sin(deg);
		float c = Cos(deg);
		float t;
		t = c * m[0][0] - s * m[0][2];
		m[0][2] = s * m[0][0] + c * m[0][2];
		m[0][0] = t;
		t = c * m[1][0] - s * m[1][2];
		m[1][2] = s * m[1][0] + c * m[1][2];
		m[1][0] = t;
		t = c * m[2][0] - s * m[2][2];
		m[2][2] = s * m[2][0] + c * m[2][2];
		m[2][0] = t;
	}
	void Matrix44::rotate_z(float deg) {
		float s = Sin(deg);
		float c = Cos(deg);
		float t;
		t = c * m[0][0] + s * m[0][1];
		m[0][1] = -s * m[0][0] + c * m[0][1];
		m[0][0] = t;
		t = c * m[1][0] + s * m[1][1];
		m[1][1] = -s * m[1][0] + c * m[1][1];
		m[1][0] = t;
		t = c * m[2][0] + s * m[2][1];
		m[2][1] = -s * m[2][0] + c * m[2][1];
		m[2][0] = t;
	}
	void Matrix44::set_view_transform(
		const Vector3& p, const Vector3& t) {
		Vector3 d(t - p);
		float l = Sqrt(d.x*d.x + d.z*d.z);
		float ax = Atan2(d.y, l);
		float ay = Atan2(d.x, d.z) + 180.f;
		//角度補正
		if (ay > 180.f) { ay -= 360.f; }
		set_rotate_x(-ax);
		rotate_y(-ay);
		translate(Vector3(-p.x, -p.y, -p.z));
	}
	void Matrix44::set_view_transform(
		const Vector3& p,
		const Vector3& t,
		const Vector3& up) {
		//Z方向の基底ベクトル
		Math::Vector3 e = (p - t).normalize();
		//X方向の基底ベクトル
		Math::Vector3 v = up.cross(e).normalize();
		//Y方向の基底ベクトル
		Math::Vector3 u = e.cross(v).normalize();
		//向きを合わせる行列
		m[0][0] = v.x;
		m[0][1] = v.y;
		m[0][2] = v.z;
		m[1][0] = u.x;
		m[1][1] = u.y;
		m[1][2] = u.z;
		m[2][0] = e.x;
		m[2][1] = e.y;
		m[2][2] = e.z;
		m[3][3] = 1.f;
		m[0][3] = 0.f;
		m[1][3] = 0.f;
		m[2][3] = 0.f;
		m[3][0] = m[3][1] = m[3][2] = 0.f;
		//原点に移動
		translate(Vector3(-p.x, -p.y, -p.z));
	}
	void Matrix44::set_perspective_transform(
		float view_angle_y,
		float window_width,
		float window_height,
		float near_clip,
		float far_clip) {
		float s = 1.f / Tan(view_angle_y * 0.5f);
		float r = window_height / window_width;
		float a = far_clip / (near_clip - far_clip);
		float b = near_clip * a;
		m[0][0] = s * r;
		m[1][1] = s;
		m[2][2] = a;
		m[2][3] = b;
		m[3][2] = -1.f;
		m[0][1] = m[0][2] = m[0][3] = 0.f;
		m[1][0] = m[1][2] = m[1][3] = 0.f;
		m[2][0] = m[2][1] = 0.f;
		m[3][0] = m[3][1] = m[3][3] = 0.f;
	}
	Matrix44& Matrix44::operator*=(const Matrix44& a) {
		//同じベクトルで問題なし
		float t[4][4];
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 4; ++j) {
				t[i][j] = 0.f;
				for (int k = 0; k < 4; ++k) {
					t[i][j] += (m[i][k] * a.m[k][j]);
				}
			}
		}
		memcpy_s(m, sizeof(m), t, sizeof(t));

		return *this;
	}
	const Matrix44 Matrix44::operator*(const Matrix44& a) const {
		Matrix44 t;
		for (int i = 0; i < 4; ++i) {
			for (int j = 0; j < 4; ++j) {
				t.m[i][j] = 0.f;
				for (int k = 0; k < 4; ++k) {
					t.m[i][j] += (m[i][k] * a.m[k][j]);
				}
			}
		}

		return t;
	}
	Math::Vector3 Matrix44::operator*(const Math::Vector3& v)
	{
		Math::Vector3 rtv;
		multiply(&rtv, v);
		return rtv;
	}
	void Matrix44::multiply(Vector3* out, const Vector3& in) const {
		//同じベクトルで問題なし
		float dx = in.x;
		float dy = in.y;
		out->x = m[0][0] * dx + m[0][1] * dy + m[0][2] * in.z + m[0][3];
		out->y = m[1][0] * dx + m[1][1] * dy + m[1][2] * in.z + m[1][3];
		out->z = m[2][0] * dx + m[2][1] * dy + m[2][2] * in.z + m[2][3];
	}
	void Matrix44::copy_to_array(float (*ary)[4][4]) const {
		memcpy_s(*ary, sizeof(*ary), m, sizeof(m));
	}
	void Matrix44::world_mat_convert_for_normal() {
		set_inverse();
		set_transposition();
	}

}//namespace Math