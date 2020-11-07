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

	//重力加速度
	static float gravity = -980.f;
	static Math::Vector2 gravity_2D(0.f, 980.f);
	static Math::Vector3 gravity_3D(0.f, -980.f, 0.f);
	//動く物体の衝突判定の制度
	static float collision_quality = 1.f;

	//サポート関数
	//点から線分への最短ベクトル
	void ShortVec_Of_Point_And_Lay(
		Math::Vector3* short_vec,
		const Math::Vector3& pos,
		const Lay& lay);
	//点とAABBの最短距離
	float ShortLen_Of_Point_And_AABB(
		const Math::Vector3& pos, const AABB& aabb);
	//点とOBBの最短距離
	float ShortLen_Of_Point_And_OBB(
		const Math::Vector3& pos, const OBB& obb);
	//相対ベクトル
	void Relative_Vector(
		Math::Vector3* rtv_vec, 
		const Math::Vector3& vec, const Math::Vector3& angle);
	//軸単位の角度
	float angle_x(const Math::Vector3& vec);
	float angle_y(const Math::Vector3& vec);
	float angle_z(const Math::Vector3& vec);
	//ベクトルの角度
	void vec_angle(Math::Vector3* angle, const Math::Vector3& vec);
	//視野内判定
	//3次元
	bool In_View(
		const Math::Vector3& vec,
		const Math::Vector3& angle,
		float view_angle_x, float view_angle_y,
		float view_lengh);

	//衝突判定(基本)
	//球と球
	bool Sphere_HitCheck_Sphere(
		const Sphere& a, const Sphere& b);
	//線分と球
	bool Lay_HitCheck_Sphere(
		const Lay& lay, const Sphere& sp);
	//線分と球: 衝突位置も取得
	bool Lay_HitCheck_Sphere(
		Math::Vector3* hit_pos,
		const Lay& lay, const Sphere& sp);
	//線分と三角平面
	bool Lay_HitCheck_TriPlane(
		Math::Vector3* hit_pos,
		const Lay& lay, const TriPlane& tp);
	//球と三角平面
	bool Sphere_HitCheck_TriPlane(
		const Sphere& sp, const TriPlane& tp);
	//三角平面と三角平面
	bool TriPlane_HitCheck_TriPlane(
		const TriPlane& a, const TriPlane& b);
	//球とAABB
	bool Sphere_HitCheck_AABB(
		const Sphere& sp, const AABB& aabb);
	//点とAABB
	bool Point_HitCheck_AABB(
		const Math::Vector3& p, const AABB& aabb);
	//線分とAABB
	bool Lay_HitCheck_AABB(
		const Lay& lay, const AABB& aabb);
	//AABBとAABB
	bool AABB_HitCheck_AABB(
		const AABB& a, const AABB& b);
	//点とOBB
	bool Point_HitCheck_OBB(
		const Math::Vector3& p, const OBB& obb);
	//球とOBB
	bool Sphere_HitCheck_OBB(
		const Sphere& sp, const OBB& obb);
	//線分とOBB
	bool Lay_HitCheck_OBB(
		const Lay& lay, const OBB& obb);
	//平面とOBB
	bool TriPlane_HitCheck_OBB(
		const TriPlane& tp, const OBB& obb);
	//OBBとOBB
	bool OBB_HitCheck_OBB(
		const OBB& a, const OBB& b);

	//OBBがAABBに含まれるか
	bool OBB_Contains_AABB(const OBB& obb, const AABB& aabb);
	//aがbに含まれるか
	bool OBB_Contains_OBB(const OBB& a, const OBB& b);


	//運動モデル
	//動く物体同士（球体）
	void Rebound_Exercise(
		Math::Vector3* get_vel_a, Math::Vector3* get_vel_b,
		const Math::Vector3& vel_a, const Math::Vector3& vel_b,
		const Math::Vector3& pos_a, const Math::Vector3& pos_b,
		float mass_a, float mass_b, float rb_coef_a, float rb_coef_b);
	//平面と動く物体（球体）
	void Reflection_Exercise(
		Math::Vector3* get_vel,
		const Math::Vector3& vel, float rb_coef,
		const Math::Vector3& normal);
	//平面に壁ずり（地面を歩く）
	void WallSlide_Exercise(
		Math::Vector3* get_vel,
		const Math::Vector3& vel, const Math::Vector3& normal);

}//namespace Physics

#endif