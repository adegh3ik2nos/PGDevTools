#include "src/Physics/CollisionModel.h"

namespace Physics {

	//è’ìÀÉÇÉfÉã
	//ãÖ
	Sphere::Sphere() {}
	Sphere::Sphere(const Math::Vector3& pos, float rds) :
		mPos(pos), mRds(rds) {}
	Sphere::Sphere(const Sphere& a) :
		mPos(a.mPos), mRds(a.mRds) {}
	Sphere::~Sphere() {}
	Sphere& Sphere::operator=(const Sphere& a) {
		mPos = a.mPos;
		mRds = a.mRds;
		return *this;
	}
	//ê¸ï™
	Lay::Lay(){}
	Lay::Lay(const Math::Vector3& pos, const Math::Vector3& vec) :
		mPos(pos), mVec(vec) {}
	Lay::Lay(const Lay& a) :
		mPos(a.mPos), mVec(a.mVec) {}
	Lay::~Lay() {}
	Lay& Lay::operator=(const Lay& a) {
		mPos = a.mPos;
		mVec = a.mVec;
		return *this;
	}
	//éOäpïΩñ 
	TriPlane::TriPlane(){}
	TriPlane::TriPlane(
		const Math::Vector3& p0,
		const Math::Vector3& p1,
		const Math::Vector3& p2) :
		mP0(p0), mP1(p1), mP2(p2),
		mNormal(0.f, 0.f, 0.f),
		mD(0.f) {
		mNormal.set_cross(p1 - p0, p2 - p1);
		mNormal.set_normalize();
		mD = -mNormal.dot(p0);
	}
	TriPlane::TriPlane(const TriPlane& a) :
		mP0(a.mP0), mP1(a.mP1), mP2(a.mP2),
		mNormal(a.mNormal),
		mD(a.mD) {}
	TriPlane::~TriPlane() {}
	TriPlane& TriPlane::operator=(const TriPlane& a) {
		mP0 = a.mP0;
		mP1 = a.mP1;
		mP2 = a.mP2;
		mNormal = a.mNormal;
		mD = a.mD;
		return *this;
	}
	//AABB
	AABB::AABB() {}
	AABB::AABB(const Math::Vector3& min, const Math::Vector3& max) :
		mMin(min),
		mMax(max) {}
	AABB::AABB(const Physics::OBB & obb)
		: mMin()
		, mMax()
	{
		Math::Vector3 _edges[3]{ Math::Vector3(obb.mHalfSize.x * 2.f, 0, 0), Math::Vector3(0, obb.mHalfSize.y * 2.f, 0), Math::Vector3(0, 0, obb.mHalfSize.z * 2.f) };
		_edges[0] *= obb.mRot;
		_edges[1] *= obb.mRot;
		_edges[2] *= obb.mRot;
		Math::Vector3 _origin = obb.mPos;

		Math::Vector3 vertices[8]{};
		vertices[0] = _origin + (_edges[0] * 0.5f) + (_edges[1] * 0.5f) + (_edges[2] * 0.5f);
		vertices[1] = vertices[0] - _edges[2];
		vertices[2] = vertices[0] - _edges[0];
		vertices[3] = vertices[2] - _edges[2];

		vertices[4] = vertices[0] - _edges[1];
		vertices[5] = vertices[4] - _edges[2];
		vertices[6] = vertices[4] - _edges[0];
		vertices[7] = vertices[6] - _edges[2];

		mMin = mMax = obb.mPos;
		for (const Math::Vector3& v : vertices)
		{
			mMin.x = std::min(v.x, mMin.x);
			mMin.y = std::min(v.y, mMin.y);
			mMin.z = std::min(v.z, mMin.z);

			mMax.x = std::max(v.x, mMax.x);
			mMax.y = std::max(v.y, mMax.y);
			mMax.z = std::max(v.z, mMax.z);
		}
	}
	AABB::AABB(const AABB& a):
		mMin(a.mMin),
		mMax(a.mMax){}
	AABB::~AABB() {}
	AABB& AABB::operator=(const AABB& a) {
		mMin = a.mMin;
		mMax = a.mMax;
		return *this;
	}
	//OBB
	OBB::OBB() {}
	OBB::OBB(const Math::Vector3& pos, const Math::Vector3& half_size,
		const Math::Matrix44& rot) :
		mPos(pos),
		mHalfSize(half_size),
		mRot(rot),
		mIvsRot(rot) {
		mIvsRot.set_inverse();
	}
	OBB::OBB(const OBB& a) :
		mPos(a.mPos),
		mHalfSize(a.mHalfSize),
		mRot(a.mRot),
		mIvsRot(a.mIvsRot) {}
	OBB::~OBB() {}
	OBB& OBB::operator=(const OBB& a) {
		mPos = a.mPos;
		mHalfSize = a.mHalfSize;
		mRot = a.mRot;
		mIvsRot = a.mIvsRot;
		return *this;
	}

}//namespace Physics