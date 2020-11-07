#include "Math/Matrix33.h"

#include <memory>

#include "Math/Vector2.h"
#include "Math/Math.h"

namespace Math {

	Matrix33::Matrix33() {}
	Matrix33::Matrix33(const Matrix33& a) {
		memcpy_s(m, sizeof(m), a.m, sizeof(a.m));
	}
	Matrix33::~Matrix33() {}
	Matrix33& Matrix33::operator=(const Matrix33& a) {
		memcpy_s(m, sizeof(m), a.m, sizeof(a.m));
		return *this;
	}
	void Matrix33::set_identity() {
		m[0][0] = m[1][1] = m[2][2] = 1.f;
		m[0][1] = m[0][2] = 0.f;
		m[1][0] = m[1][2] = 0.f;
		m[2][0] = m[2][1] = 0.f;
	}
	void Matrix33::set_transposition() {
		Matrix33 prev = *this;
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				m[i][j] = prev.m[j][i];
			}
		}
	}
	void Matrix33::set_inverse() {
		float delta = 
			m[0][0]*m[1][1]*m[2][2] + m[1][0]*m[2][1]*m[0][2] + m[2][0]*m[0][1]*m[1][2] - 
			m[0][0]*m[2][1]*m[1][2] - m[2][0]*m[1][1]*m[0][2] - m[1][0]*m[0][1]*m[2][2];
		//ŒvŽZ‚Å‚«‚È‚¢ê‡‚ÍI—¹
		if (delta == 0.f) return;
		//‹t”‚É‚µ‚ÄŒvŽZ•‰‰×ŒyŒ¸
		delta = 1.f / delta;
		//‹ts—ñŒvŽZ
		Matrix33 prev = *this;
		m[0][0] = (prev.m[1][1]*prev.m[2][2] - prev.m[1][2]*prev.m[2][1])*delta;
		m[0][1] = (prev.m[0][2]*prev.m[2][1] - prev.m[0][1]*prev.m[2][2])*delta;
		m[0][2] = (prev.m[0][1]*prev.m[1][2] - prev.m[0][2]*prev.m[1][1])*delta;
		m[1][0] = (prev.m[1][2]*prev.m[2][0] - prev.m[1][0]*prev.m[2][2])*delta;
		m[1][1] = (prev.m[0][0]*prev.m[2][2] - prev.m[0][2]*prev.m[2][0])*delta;
		m[1][2] = (prev.m[0][2]*prev.m[1][0] - prev.m[0][0]*prev.m[1][2])*delta;
		m[2][0] = (prev.m[1][0]*prev.m[2][1] - prev.m[1][1]*prev.m[2][0])*delta;
		m[2][1] = (prev.m[0][1]*prev.m[2][0] - prev.m[0][0]*prev.m[2][1])*delta;
		m[2][2] = (prev.m[0][0]*prev.m[1][1] - prev.m[0][1]*prev.m[1][0])*delta;
	}
	void Matrix33::set_translate(const Vector2& t) {
		m[0][0] = m[1][1] = m[2][2] = 1.f;
		m[0][2] = t.x;
		m[1][2] = t.y;
		m[0][1] = 0.f;
		m[1][0] = 0.f;
		m[2][0] = m[2][1] = 0.f;
	}
	void Matrix33::set_rotate(float deg) {
		float s = Sin(deg);
		float c = Cos(deg);
		m[0][0] = m[1][1] = c;
		m[2][2] = 1.f;
		m[0][1] = -s;
		m[1][0] = s;
		m[0][2] = 0.f;
		m[1][2] = 0.f;
		m[2][0] = m[2][1] = 0.f;
	}
	void Matrix33::set_scale(const Vector2& s) {
		m[0][0] = s.x;
		m[1][1] = s.y;
		m[2][2] = 1.f;
		m[0][1] = m[0][2] = 0.f;
		m[1][0] = m[1][2] = 0.f;
		m[2][0] = m[2][1] = 0.f;
	}
	Matrix33& Matrix33::operator*=(const Matrix33& a) {
		//“¯‚¶ƒxƒNƒgƒ‹‚Å–â‘è‚È‚µ
		float t[3][3];
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				t[i][j] = 0.f;
				for (int k = 0; k < 3; ++k) {
					t[i][j] += (m[i][k] * a.m[k][j]);
				}
			}
		}
		memcpy_s(m, sizeof(m), t, sizeof(t));

		return *this;
	}
	void Matrix33::multiply(Vector2* out, const Vector2& in) {
		//“¯‚¶ƒxƒNƒgƒ‹‚Å–â‘è‚È‚µ
		float dx = in.x;
		out->x = m[0][0] * dx + m[0][1] * in.y + m[0][2];
		out->y = m[1][0] * dx + m[1][1] * in.y + m[1][2];
	}
	void Matrix33::world_mat_convert_for_normal() {
		set_inverse();
		set_transposition();
	}

}//namespace Math