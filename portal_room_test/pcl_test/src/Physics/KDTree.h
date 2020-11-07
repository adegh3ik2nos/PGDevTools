#ifndef INCLUDED_PHYSICS_KDTREE_H
#define INCLUDED_PHYSICS_KDTREE_H

#include <memory>

#include "Math/Vector3.h"

namespace Physics {
	class AABB;
}

namespace Physics {

	namespace Octree {
		//�I�u�W�F�N�g�ƃ��[�g���ԍ����܂Ƃ߂�
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
			//�R�s�[�֎~
			ObjAndMN(const ObjAndMN& a);
			ObjAndMN& operator=(const ObjAndMN& a);
		};

		//��ԕ������x��
		static unsigned area_div_lv = 1;
		//�ő��ԕ������x��
		static const  unsigned max_area_div_lv = 8;
		//��ԑS�̂̑傫��
		static Math::Vector3 area_size(100.f, 100.f, 100.f);

		//��ԕ������x���ݒ�
		void Set_Area_Div_Lv(unsigned lv);
		//��ԑS�̂̑傫����ݒ�
		void Set_Area_Size(Math::Vector3 size);

		//2�r�b�g�Ԋu�ɂ���
		unsigned Bit_Separate(unsigned char bit_data);
		//���[�g���ԍ��擾
		unsigned Morton_No(unsigned char x, unsigned char y, unsigned char z);
		//���`���[�g���ԍ��擾
		unsigned Linear_Morton_No(const Physics::AABB& aabb);
		//��ԃ��x���擾
		unsigned Area_Lv(unsigned no);
		//��ԃ��x����������
		unsigned Down_Area_Lv(unsigned no);
		//������Ԃ��ǂ���
		bool Same_Area(unsigned no0, unsigned no1);

	}//namespace Octree

}//namespace Physics

#endif