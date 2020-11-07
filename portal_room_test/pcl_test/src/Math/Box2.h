#ifndef INCLUDED_MATH_BOX2_H
#define INCLUDED_MATH_BOX2_H

namespace Math {

	class Box2 {
	public:
		//ê∂ê¨
		Box2();
		Box2(int ax, int ay, int aw, int ah);
		Box2(const Box2& a);
		~Box2();
		Box2& operator=(const Box2& a);
		//à⁄ìÆ
		void set_offset(int ofs_x, int ofs_y);
		void set_offset(float ofs_x, float ofs_y);
		const Box2 offset(int ofs_x, int ofs_y) const;
		const Box2 offset(float ofs_x, float ofs_y) const;

		int x, y, w, h;
	};

}//namespace Math

#endif