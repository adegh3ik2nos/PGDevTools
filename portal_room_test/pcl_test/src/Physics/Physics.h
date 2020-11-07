#ifndef INCLUDED_PHYSICS_PHYSICS_H
#define INCLUDED_PHYSICS_PHYSICS_H

#include "src/Math/Vector2.h"
#include "src/Math/Vector3.h"

namespace Physics {
	class Sphere;
	class Lay;
	class TriPlane;
	class AABB;
	class OBB;
}

namespace Physics {

	//�d�͉����x
	static float gravity = -980.f;
	static Math::Vector2 gravity_2D(0.f, 980.f);
	static Math::Vector3 gravity_3D(0.f, -980.f, 0.f);
	//�������̂̏Փ˔���̐��x
	static float collision_quality = 1.f;

	//�T�|�[�g�֐�
	//�_��������ւ̍ŒZ�x�N�g��
	void ShortVec_Of_Point_And_Lay(
		Math::Vector3* short_vec,
		const Math::Vector3& pos,
		const Lay& lay);
	//�_��AABB�̍ŒZ����
	float ShortLen_Of_Point_And_AABB(
		const Math::Vector3& pos, const AABB& aabb);
	//�_��OBB�̍ŒZ����
	float ShortLen_Of_Point_And_OBB(
		const Math::Vector3& pos, const OBB& obb);
	//���΃x�N�g��
	void Relative_Vector(
		Math::Vector3* rtv_vec, 
		const Math::Vector3& vec, const Math::Vector3& angle);
	//���P�ʂ̊p�x
	float angle_x(const Math::Vector3& vec);
	float angle_y(const Math::Vector3& vec);
	float angle_z(const Math::Vector3& vec);
	//�x�N�g���̊p�x
	void vec_angle(Math::Vector3* angle, const Math::Vector3& vec);
	//���������
	//3����
	bool In_View(
		const Math::Vector3& vec,
		const Math::Vector3& angle,
		float view_angle_x, float view_angle_y,
		float view_lengh);

	//�Փ˔���(��{)
	//���Ƌ�
	bool Sphere_HitCheck_Sphere(
		const Sphere& a, const Sphere& b);
	//�����Ƌ�
	bool Lay_HitCheck_Sphere(
		const Lay& lay, const Sphere& sp);
	//�����Ƌ�: �Փˈʒu���擾
	bool Lay_HitCheck_Sphere(
		Math::Vector3* hit_pos,
		const Lay& lay, const Sphere& sp);
	//�����ƎO�p����
	bool Lay_HitCheck_TriPlane(
		Math::Vector3* hit_pos,
		const Lay& lay, const TriPlane& tp);
	//���ƎO�p����
	bool Sphere_HitCheck_TriPlane(
		const Sphere& sp, const TriPlane& tp);
	//�O�p���ʂƎO�p����
	bool TriPlane_HitCheck_TriPlane(
		const TriPlane& a, const TriPlane& b);
	//����AABB
	bool Sphere_HitCheck_AABB(
		const Sphere& sp, const AABB& aabb);
	//�_��AABB
	bool Point_HitCheck_AABB(
		const Math::Vector3& p, const AABB& aabb);
	//������AABB
	bool Lay_HitCheck_AABB(
		const Lay& lay, const AABB& aabb);
	//AABB��AABB
	bool AABB_HitCheck_AABB(
		const AABB& a, const AABB& b);
	//�_��OBB
	bool Point_HitCheck_OBB(
		const Math::Vector3& p, const OBB& obb);
	//����OBB
	bool Sphere_HitCheck_OBB(
		const Sphere& sp, const OBB& obb);
	//������OBB
	bool Lay_HitCheck_OBB(
		const Lay& lay, const OBB& obb);
	//���ʂ�OBB
	bool TriPlane_HitCheck_OBB(
		const TriPlane& tp, const OBB& obb);
	//OBB��OBB
	bool OBB_HitCheck_OBB(
		const OBB& a, const OBB& b);

	//OBB��AABB�Ɋ܂܂�邩
	bool OBB_Contains_AABB(const OBB& obb, const AABB& aabb);
	//a��b�Ɋ܂܂�邩
	bool OBB_Contains_OBB(const OBB& a, const OBB& b);


	//�^�����f��
	//�������̓��m�i���́j
	void Rebound_Exercise(
		Math::Vector3* get_vel_a, Math::Vector3* get_vel_b,
		const Math::Vector3& vel_a, const Math::Vector3& vel_b,
		const Math::Vector3& pos_a, const Math::Vector3& pos_b,
		float mass_a, float mass_b, float rb_coef_a, float rb_coef_b);
	//���ʂƓ������́i���́j
	void Reflection_Exercise(
		Math::Vector3* get_vel,
		const Math::Vector3& vel, float rb_coef,
		const Math::Vector3& normal);
	//���ʂɕǂ���i�n�ʂ�����j
	void WallSlide_Exercise(
		Math::Vector3* get_vel,
		const Math::Vector3& vel, const Math::Vector3& normal);

}//namespace Physics

#endif