#ifndef INCLUDED_PHYSICS_COLLISIONMODEL_H
#define INCLUDED_PHYSICS_COLLISIONMODEL_H

#include "src/Math/Vector3.h"
#include "src/Math/Matrix44.h"

namespace Physics
{
	class OBB;
}

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
		AABB(const Physics::OBB& obb);
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

		std::vector<Math::Vector3> GetVertices() const
		{
			Math::Vector3 _edges[3]{ Math::Vector3(mHalfSize.x * 2.f, 0, 0), Math::Vector3(0, mHalfSize.y * 2.f, 0), Math::Vector3(0, 0, mHalfSize.z * 2.f) };
			_edges[0] *= mRot;
			_edges[1] *= mRot;
			_edges[2] *= mRot;
			Math::Vector3 _origin = mPos;

			std::vector<Math::Vector3> vertices(8);
			vertices[0] = _origin + (_edges[0] * 0.5f) + (_edges[1] * 0.5f) + (_edges[2] * 0.5f);
			vertices[1] = vertices[0] - _edges[2];
			vertices[2] = vertices[0] - _edges[0];
			vertices[3] = vertices[2] - _edges[2];

			vertices[4] = vertices[0] - _edges[1];
			vertices[5] = vertices[4] - _edges[2];
			vertices[6] = vertices[4] - _edges[0];
			vertices[7] = vertices[6] - _edges[2];

			return vertices;
		}

		Math::Vector3 mPos, mHalfSize;
		Math::Matrix44 mRot, mIvsRot;
	};

}//namespace Physics

#endif