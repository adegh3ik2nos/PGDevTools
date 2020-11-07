#include "src/Math/Box2.h"

namespace Math {

	Box2::Box2() {}
	Box2::Box2(int ax, int ay, int aw, int ah) :
		x(ax),
		y(ay),
		w(aw),
		h(ah) {}
	Box2::Box2(const Box2& a):
		x(a.x),
		y(a.y),
		w(a.w),
		h(a.h) {}
	Box2::~Box2() {}
	Box2& Box2::operator=(const Box2& a) {
		x = a.x;
		y = a.y;
		w = a.w;
		h = a.h;
		return *this;
	}
	void Box2::set_offset(int ofs_x, int ofs_y) {
		x += ofs_x;
		y += ofs_y;
	}
	void Box2::set_offset(float ofs_x, float ofs_y) {
		x += static_cast<int>(ofs_x);
		y += static_cast<int>(ofs_y);
	}
	const Box2 Box2::offset(int ofs_x, int ofs_y) const{
		return Box2(x + ofs_x, y + ofs_y, w, h);
	}
	const Box2 Box2::offset(float ofs_x, float ofs_y) const {
		return Box2(x + static_cast<int>(ofs_x), y + static_cast<int>(ofs_y), w, h);
	}

}//namespace Math