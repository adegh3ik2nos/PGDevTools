#ifndef INCLUDED_PHYSICS_KDTREE_H
#define INCLUDED_PHYSICS_KDTREE_H

#include <memory>

#include "Math/Vector3.h"

namespace Physics {
	class AABB;
}

namespace Physics {

	namespace Octree {
		//オブジェクトとモートン番号をまとめる
		template<typename T> class ObjAndMN {
		public:
			typedef std::shared_ptr<ObjAndMN> SP;

			ObjAndMN() :
				mObject(),
				mMortonNo(0) {}
			~ObjAndMN() {}

			T mObject;
			unsigned mMortonNo;
		private:
			//コピー禁止
			ObjAndMN(const ObjAndMN& a);
			ObjAndMN& operator=(const ObjAndMN& a);
		};

		//空間分割レベル
		static unsigned area_div_lv = 1;
		//最大空間分割レベル
		static const  unsigned max_area_div_lv = 8;
		//空間全体の大きさ
		static Math::Vector3 area_size(100.f, 100.f, 100.f);

		//空間分割レベル設定
		void Set_Area_Div_Lv(unsigned lv);
		//空間全体の大きさを設定
		void Set_Area_Size(Math::Vector3 size);

		//2ビット間隔にする
		unsigned Bit_Separate(unsigned char bit_data);
		//モートン番号取得
		unsigned Morton_No(unsigned char x, unsigned char y, unsigned char z);
		//線形モートン番号取得
		unsigned Linear_Morton_No(const Physics::AABB& aabb);
		//空間レベル取得
		unsigned Area_Lv(unsigned no);
		//空間レベルを下げる
		unsigned Down_Area_Lv(unsigned no);
		//同じ空間かどうか
		bool Same_Area(unsigned no0, unsigned no1);

	}//namespace Octree

}//namespace Physics

#endif