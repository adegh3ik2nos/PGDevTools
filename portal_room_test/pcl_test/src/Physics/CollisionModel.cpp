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