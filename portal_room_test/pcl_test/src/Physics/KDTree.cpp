#include "src/Physics/KDTree.h"

#include "src/Math/Math.h"
#include "src/Physics/CollisionModel.h"

namespace Physics {

	namespace Octree {
		//空間分割レベル設定
		void Set_Area_Div_Lv(unsigned lv) {
			if (lv > max_area_div_lv) return;
			area_div_lv = lv;
		}
		//空間全体の大きさを設定
		void Set_Area_Size(Math::Vector3 size) {
			area_size = size;
		}
		//2ビット間隔にする
		unsigned Bit_Separate(unsigned char bit_data) {
			unsigned sep_bit = bit_data;
			sep_bit = (sep_bit | sep_bit << 8) & 0x0000f00f;
			sep_bit = (sep_bit | sep_bit << 4) & 0x000c30c3;
			sep_bit = (sep_bit | sep_bit << 2) & 0x00249249;
			return sep_bit;
		}
		//モートン番号取得
		unsigned Morton_No(unsigned char x, unsigned char y, unsigned char z){
			return Bit_Separate(x) | Bit_Separate(y) << 1 | Bit_Separate(z) << 2;
		}
		//線形モートン番号取得
		unsigned Linear_Morton_No(const Physics::AABB& aabb) {
			//位置を正の方向に補正
			Math::Vector3 ofs = area_size / 2.f;
			Math::Vector3 temp_min = aabb.mMin + ofs;
			Math::Vector3 temp_max = aabb.mMax + ofs;
			//レベルに合わせ分割
			Math::Vector3 sep = area_size / Math::Pow(2.f, static_cast<float>(area_div_lv));
			temp_min /= sep;
			temp_max /= sep;
			unsigned min_no = Morton_No(
				static_cast<unsigned char>(temp_min.x),
				static_cast<unsigned char>(temp_min.y),
				static_cast<unsigned char>(temp_min.z));
			unsigned max_no = Morton_No(
				static_cast<unsigned char>(temp_max.x),
				static_cast<unsigned char>(temp_max.y),
				static_cast<unsigned char>(temp_max.z));

			unsigned xor_no = min_no ^ max_no;

			unsigned shift_num = 0;
			for (unsigned l = 0; l < area_div_lv; ++l) {
				unsigned mask = 0x00000007 << l * 3;
				if ((xor_no & mask) != 0) {
					shift_num = l + 1;
				}
			}
			unsigned share_area = area_div_lv - shift_num;
			unsigned share_area_no = min_no >> shift_num * 3;

			unsigned morton_no =  
				static_cast<unsigned>(Math::Pow(8, share_area) - 1) / 7 + share_area_no;

			return morton_no;
		}
		//空間レベル取得
		unsigned Area_Lv(unsigned no) {
			int temp_no = no;
			for (unsigned lv = 0; lv < area_div_lv; ++lv) {
				temp_no -= Math::Pow(8, lv);
				if (temp_no < 0) return lv;
			}
			return area_div_lv;
		}
		//空間レベルを下げる
		unsigned Down_Area_Lv(unsigned no) {
			return (no - 1) >> 3;
		}
		//同じ空間かどうか
		bool Same_Area(unsigned no0, unsigned no1) {
			//空間レベル取得
			unsigned area_lv0 = Area_Lv(no0);
			unsigned area_lv1 = Area_Lv(no1);
			//同じであればそのままチェック
			if (area_lv0 == area_lv1)  return no0 == no1;
			//それ以外は低いほうに合わせる
			unsigned temp_no0 = no0;
			unsigned temp_no1 = no1;
			//高い方のレベルを下げる
			if (area_lv0 > area_lv1) {
				int sub = area_lv0 - area_lv1;
				for (int lv = 0; lv < sub; ++lv) {
					temp_no0 = Down_Area_Lv(temp_no0);
				}
			}
			else if (area_lv0 < area_lv1) {
				int sub = area_lv1 - area_lv0;
				for (int lv = 0; lv < sub; ++lv) {
					temp_no1 = Down_Area_Lv(temp_no1);
				}
			}
			return temp_no0 == temp_no1;
		}

	}//namespace Octree


}//namespace Physics