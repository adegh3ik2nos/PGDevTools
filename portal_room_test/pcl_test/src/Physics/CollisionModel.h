#ifndef INCLUDED_PHYSICS_COLLISIONMODEL_H
#define INCLUDED_PHYSICS_COLLISIONMODEL_H

#include "src/Math/Vector3.h"
#include "src/Math/Matrix44.h"

namespace Physics {

	//è’ìÀÉÇÉfÉã
	//ãÖ
	class Sphere {
	public:
		Sphere();
		Sphere(const Math::Vector3& pos, float rds);
		Sphere(const Sphere& a);
		~Sphere();
		Sphere& operator=(const Sphere& a);

		Math::Vector3 mPos;
		float mRds;
	};
	//ê¸ï™
	class Lay {
	public:
		Lay();
		Lay(const Math::Vector3& pos, const Math::Vector3& vec);
		Lay(const Lay& a);
		~Lay();
		Lay& operator=(const Lay& a);

		Math::Vector3 mPos, mVec;
	};
	//éOäpïΩñ 
	class TriPlane {
	public:
		TriPlane();
		TriPlane(
			const Math::Vector3& p0,
			const Math::Vector3& p1,
			const Math::Vector3& p2);
		TriPlane(const TriPlane& a);
		~TriPlane();
		TriPlane& operator=(const TriPlane& a);

		Math::Vector3 mP0, mP1, mP2, mNormal;
		float mD;
	};
	//AABB
	class AABB {
	public:
		AABB();
		AABB(const Math::Vector3& min, const Math::Vector3& max);
		AABB(const AABB& a);
		~AABB();
		AABB& operator=(const AABB& a);

		Math::Vector3 mMin, mMax;
	};
	//OBB
	class OBB {
	public:
		OBB();
		OBB(const Math::Vector3& pos, const Math::Vector3& half_size,
			const Math::Matrix44& rot);
		OBB(const OBB& a);
		~OBB();
		OBB& operator=(const OBB& a);

		float CalcVolume() const
		{
			return (mHalfSize.x * mHalfSize.y * mHalfSize.z) * 8.f;
		}

		Math::Vector3 mPos, mHalfSize;
		Math::Matrix44 mRot, mIvsRot;
	};

}//namespace Physics

#endif