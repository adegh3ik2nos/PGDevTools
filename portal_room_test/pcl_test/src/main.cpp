#include <iostream>  
#include <pcl/ModelCoefficients.h>  
#include <pcl/io/pcd_io.h>  
#include <pcl/point_types.h>  
#include <pcl/sample_consensus/method_types.h>  
#include <pcl/sample_consensus/model_types.h>  
#include <pcl/visualization/cloud_viewer.h>  
#include <pcl/ModelCoefficients.h>  
#include <pcl/point_types.h>  
#include <pcl/io/pcd_io.h>  
#include <pcl/features/normal_3d.h>  
#include <pcl/filters/extract_indices.h>  
#include <pcl/filters/voxel_grid.h>  
#include <pcl/kdtree/kdtree.h>  
#include <pcl/sample_consensus/method_types.h>  
#include <pcl/sample_consensus/model_types.h>  
#include <pcl/segmentation/sac_segmentation.h>  
#include <pcl/segmentation/extract_clusters.h>  
#include <pcl/visualization/cloud_viewer.h>  
#include <pcl/surface/convex_hull.h>
#include "./rabv.hpp"
#include <iostream>
#include <cmath>
#include "Math/Vector3.h"
#include "Physics/CollisionModel.h"
#include "Physics/Physics.h"
#include <string>
#include <cstdio>
#include <vector>

using namespace std;
using namespace pcl;
using namespace Math;
using namespace Physics;

template <typename ... Args>
std::string format(const std::string& fmt, Args ... args)
{
	size_t len = std::snprintf(nullptr, 0, fmt.c_str(), args ...);
	std::vector<char> buf(len + 1);
	std::snprintf(&buf[0], len + 1, fmt.c_str(), args ...);
	return std::string(&buf[0], &buf[0] + len);
}

struct Poly
{
	int index[3];
};

void Draw(const vector<OBB>& obbList, const vector<Vector3>& vertices, const vector<Poly>& floor, const vector<Poly>& wall)
{
	auto rab = rabv::Rab::create();

	rabv::Lines line1;
	rabv::Lines line2;
	line2.color = rabv::Color(0, 255, 0);
	for (int i = 0; i < obbList.size(); ++i)
	{
		const OBB& obb = obbList[i];
		Vector3 _edges[3]{ Vector3(obb.mHalfSize.x * 2.f, 0, 0), Vector3(0, obb.mHalfSize.y * 2.f, 0), Vector3(0, 0, obb.mHalfSize.z * 2.f) };
		_edges[0] *= obb.mRot;
		_edges[1] *= obb.mRot;
		_edges[2] *= obb.mRot;
		Vector3 _origin = obb.mPos;

		Vector3 ftr = _origin + (_edges[0] * 0.5f) + (_edges[1] * 0.5f) + (_edges[2] * 0.5f);
		Vector3 fbr = ftr - _edges[2];
		Vector3 ftl = ftr - _edges[0];
		Vector3 fbl = ftl - _edges[2];

		Vector3 btr = ftr - _edges[1];
		Vector3 bbr = btr - _edges[2];
		Vector3 btl = btr - _edges[0];
		Vector3 bbl = btl - _edges[2];

		line1.addLine(ftr.ToPointXYZ(), fbr.ToPointXYZ());
		line1.addLine(ftr.ToPointXYZ(), ftl.ToPointXYZ());
		line1.addLine(ftl.ToPointXYZ(), fbl.ToPointXYZ());
		line1.addLine(fbl.ToPointXYZ(), fbr.ToPointXYZ());
		line1.addLine(btr.ToPointXYZ(), bbr.ToPointXYZ());
		line1.addLine(btr.ToPointXYZ(), btl.ToPointXYZ());
		line1.addLine(btl.ToPointXYZ(), bbl.ToPointXYZ());
		line1.addLine(bbl.ToPointXYZ(), bbr.ToPointXYZ());
		line1.addLine(ftr.ToPointXYZ(), btr.ToPointXYZ());
		line1.addLine(ftl.ToPointXYZ(), btl.ToPointXYZ());
		line1.addLine(fbr.ToPointXYZ(), bbr.ToPointXYZ());
		line1.addLine(fbl.ToPointXYZ(), bbl.ToPointXYZ());

		line2.addLine(_origin.ToPointXYZ(), (_origin + (_edges[0] * 0.5f)).ToPointXYZ());
		line2.addLine(_origin.ToPointXYZ(), (_origin + (_edges[1] * 0.5f)).ToPointXYZ());
		line2.addLine(_origin.ToPointXYZ(), (_origin + (_edges[2] * 0.5f)).ToPointXYZ());

	}
	rab->addLines("line0", line1);
	rab->addLines("line1", line2);

	rabv::Lines line3;
	line3.color = rabv::Color(255, 255, 0);
	for (const Poly& p : floor)
	{
		line3.addLine(vertices[p.index[0]].ToPointXYZ(), vertices[p.index[1]].ToPointXYZ());
		line3.addLine(vertices[p.index[1]].ToPointXYZ(), vertices[p.index[2]].ToPointXYZ());
		line3.addLine(vertices[p.index[0]].ToPointXYZ(), vertices[p.index[2]].ToPointXYZ());
	}
	rab->addLines("line2", line3);

	rabv::Lines line4;
	line4.color = rabv::Color(255, 0, 255);
	for (const Poly& p : wall)
	{
		line4.addLine(vertices[p.index[0]].ToPointXYZ(), vertices[p.index[1]].ToPointXYZ());
		line4.addLine(vertices[p.index[1]].ToPointXYZ(), vertices[p.index[2]].ToPointXYZ());
		line4.addLine(vertices[p.index[0]].ToPointXYZ(), vertices[p.index[2]].ToPointXYZ());
	}
	rab->addLines("line3", line4);

	// 9. Visualze the rab data while the window is closed
	const auto& viewer1 = rabv::Viewer::create(
		"Viewer1",	// Title of viewer
		rab			// Rab data
	);
	viewer1->spinLoop();
}

int
main(int argc, char** argv)
{
	vector<Vector3> vertices;
	Vector3 samp[]
	{
		Vector3(-100, 0, -100),//0
		Vector3(-20, 0, -100),
		Vector3(-100, 0, 100),
		Vector3(-20, 0, 100),

		Vector3(-20, 0, -20),//4
		Vector3(20, 0, -20),
		Vector3(-20, 0, 20),
		Vector3(20, 0, 20),

		Vector3(20, 0, -100),//8
		Vector3(100, 0, -100),
		Vector3(20, 0, 100),
		Vector3(100, 0, 100),

		Vector3(-100, 100, -100),//12
		Vector3(-20, 100, -100),
		Vector3(-100, 100, 100),
		Vector3(-20, 100, 100),

		Vector3(-20, 100, -20),//16
		Vector3(20, 100, -20),
		Vector3(-20, 100, 20),
		Vector3(20, 100, 20),

		Vector3(20, 100, -100),//20
		Vector3(100, 100, -100),
		Vector3(20, 100, 100),
		Vector3(100, 100, 100),
	};
	for (const Vector3& v : samp)
	{
		vertices.emplace_back(v);
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->points.resize(vertices.size());
	for (int i = 0; i < vertices.size(); ++i)
	{
		cloud->points[i] = (vertices[i]).ToPointXYZ();
	}

	vector<Poly> polys;
	Poly samp2[]
	{
		{0, 1, 2},
		{1, 2, 3},

		{4, 5, 6},
		{5, 6, 7},

		{8, 9, 10},
		{9, 10, 11},

		{0, 1, 12},
		{1, 12, 13},

		{0, 2, 12},
		{2, 12, 14},

		{2, 3, 14},
		{3, 14, 15},

		{1, 4, 13},
		{4, 13, 16},

		{3, 6, 18},
		{3, 18, 15},

		{4, 5, 16},
		{5, 16, 17},

		{6, 7, 18},
		{7, 18, 19},

		{8, 9, 20},
		{9, 20, 21},

		{10, 11, 22},
		{11, 22, 23},

		{5, 8, 17},
		{8, 17, 20},

		{10, 22, 7},
		{22, 7, 19},
	};
	for (const Poly& p : samp2)
	{
		polys.push_back(p);
	}

	AABB all;
	for (const Vector3& v : vertices)
	{
		all.mMin.x = min(v.x, all.mMin.x);
		all.mMin.y = min(v.y, all.mMin.y);
		all.mMin.z = min(v.z, all.mMin.z);

		all.mMax.x = max(v.x, all.mMax.x);
		all.mMax.y = max(v.y, all.mMax.y);
		all.mMax.z = max(v.z, all.mMax.z);
	}


	vector<Poly> floorPoly;
	for (const Poly& p : polys)
	{
		Vector3 a = vertices[p.index[1]] - vertices[p.index[0]];
		Vector3 b = vertices[p.index[2]] - vertices[p.index[0]];
		float dot = abs(a.cross(b).normalize().dot(Vector3(0, 1, 0)));

		float eps = 0.1f;
		if ((1.f - eps) <= dot && dot <= (1.f + eps))
		{
			floorPoly.push_back(p);//ÇŸÇ⁄ïΩñ Ç»ÇÃÇ≈è∞
		}
	}

	vector<Poly> wallPoly;
	for (const Poly& p : polys)
	{
		Vector3 a = vertices[p.index[1]] - vertices[p.index[0]];
		Vector3 b = vertices[p.index[2]] - vertices[p.index[0]];
		float dot = abs(a.cross(b).dot(Vector3(0, 1, 0)));

		float eps = 0.1f;
		if ((0.f - eps) <= dot && dot <= (0.f + eps))
		{
			wallPoly.push_back(p);//ÇŸÇ⁄êÇíºÇ»ÇÃÇ≈ï«
		}
	}

	vector<OBB> rooms;
	for (const Poly& p : floorPoly)
	{
		Vector3 c = (vertices[p.index[0]] + vertices[p.index[1]] + vertices[p.index[2]]) / 3.f;
		c.y += 0.2f;
		rooms.push_back(OBB(c, Vector3(0.5f, 0.1f, 0.5f), Matrix44::identity()));
	}

	//0Å`89ìxÇ‹Ç≈YâÒì]Ç≥ÇπÇ¬Ç¬ÅAXZïΩñ Ç≈É|ÉäÉSÉìÇ…ìñÇΩÇÈÇ‹Ç≈çLÇ∞ÇƒÅAÇ‡Ç¡Ç∆Ç‡ëÃêœÇ™ëÂÇ´Ç≠Ç»ÇÈäpìxÇå©Ç¬ÇØÇÈ
	for (OBB& obb : rooms)
	{
		float volume = 0;
		for (int t = 0; t < 90; ++t)//äpìx
		{
			Matrix44 rot;
			rot.set_rotate_y(t);
			OBB tmp = obb;
			tmp.mRot = rot;
			tmp.mIvsRot = rot.inverse();

			//XZïΩñ Ç≈ägí£
			float addv = 0.1f;
			for(int x = 0; x <= 1000; ++x)
			{
				OBB roll = tmp;
				float add = static_cast<float>(x) * addv;
				tmp.mHalfSize.x += add;
				tmp.mPos.x -= add;
				bool isHit = false;

				for (const Poly& p : wallPoly)
				{
					TriPlane tp(vertices[p.index[0]], vertices[p.index[1]], vertices[p.index[2]]);
					if (TriPlane_HitCheck_OBB(tp, tmp))
					{
						isHit = true;
						break;
					}
				}
				{
					Vector3 _edges[3]{ Vector3(tmp.mHalfSize.x * 2.f, 0, 0), Vector3(0, tmp.mHalfSize.y * 2.f, 0), Vector3(0, 0, tmp.mHalfSize.z * 2.f) };
					_edges[0] *= tmp.mRot;
					_edges[1] *= tmp.mRot;
					_edges[2] *= tmp.mRot;
					Vector3 _origin = tmp.mPos;

					Vector3 ftr = _origin + (_edges[0] * 0.5f) + (_edges[1] * 0.5f) + (_edges[2] * 0.5f);
					Vector3 fbr = ftr - _edges[2];
					Vector3 ftl = ftr - _edges[0];
					Vector3 fbl = ftl - _edges[2];

					Vector3 btr = ftr - _edges[1];
					Vector3 bbr = btr - _edges[2];
					Vector3 btl = btr - _edges[0];
					Vector3 bbl = btl - _edges[2];

					isHit |= !Point_HitCheck_AABB(ftr, all);
					isHit |= !Point_HitCheck_AABB(fbr, all);
					isHit |= !Point_HitCheck_AABB(ftl, all);
					isHit |= !Point_HitCheck_AABB(fbl, all);

					isHit |= !Point_HitCheck_AABB(btr, all);
					isHit |= !Point_HitCheck_AABB(bbr, all);
					isHit |= !Point_HitCheck_AABB(btl, all);
					isHit |= !Point_HitCheck_AABB(bbl, all);
				}

				if (isHit)
				{
					tmp = roll;
					break;
				}
			}
			for (int x = 0; x <= 1000; ++x)
			{
				OBB roll = tmp;
				float add = static_cast<float>(x) * addv;
				tmp.mHalfSize.x += add;
				tmp.mPos.x += add;
				bool isHit = false;

				for (const Poly& p : wallPoly)
				{
					TriPlane tp(vertices[p.index[0]], vertices[p.index[1]], vertices[p.index[2]]);
					if (TriPlane_HitCheck_OBB(tp, tmp))
					{
						isHit = true;
						break;
					}
				}

				{
					Vector3 _edges[3]{ Vector3(tmp.mHalfSize.x * 2.f, 0, 0), Vector3(0, tmp.mHalfSize.y * 2.f, 0), Vector3(0, 0, tmp.mHalfSize.z * 2.f) };
					_edges[0] *= tmp.mRot;
					_edges[1] *= tmp.mRot;
					_edges[2] *= tmp.mRot;
					Vector3 _origin = tmp.mPos;

					Vector3 ftr = _origin + (_edges[0] * 0.5f) + (_edges[1] * 0.5f) + (_edges[2] * 0.5f);
					Vector3 fbr = ftr - _edges[2];
					Vector3 ftl = ftr - _edges[0];
					Vector3 fbl = ftl - _edges[2];

					Vector3 btr = ftr - _edges[1];
					Vector3 bbr = btr - _edges[2];
					Vector3 btl = btr - _edges[0];
					Vector3 bbl = btl - _edges[2];

					isHit |= !Point_HitCheck_AABB(ftr, all);
					isHit |= !Point_HitCheck_AABB(fbr, all);
					isHit |= !Point_HitCheck_AABB(ftl, all);
					isHit |= !Point_HitCheck_AABB(fbl, all);

					isHit |= !Point_HitCheck_AABB(btr, all);
					isHit |= !Point_HitCheck_AABB(bbr, all);
					isHit |= !Point_HitCheck_AABB(btl, all);
					isHit |= !Point_HitCheck_AABB(bbl, all);
				}

				if (isHit)
				{
					tmp = roll;
					break;
				}
			}
			for (int z = 0; z <= 1000; ++z)
			{
				OBB roll = tmp;
				float add = static_cast<float>(z) * addv;
				tmp.mHalfSize.z += add;
				tmp.mPos.z -= add;
				bool isHit = false;

				for (const Poly& p : wallPoly)
				{
					TriPlane tp(vertices[p.index[0]], vertices[p.index[1]], vertices[p.index[2]]);
					if (TriPlane_HitCheck_OBB(tp, tmp))
					{
						isHit = true;
						break;
					}
				}

				{
					Vector3 _edges[3]{ Vector3(tmp.mHalfSize.x * 2.f, 0, 0), Vector3(0, tmp.mHalfSize.y * 2.f, 0), Vector3(0, 0, tmp.mHalfSize.z * 2.f) };
					_edges[0] *= tmp.mRot;
					_edges[1] *= tmp.mRot;
					_edges[2] *= tmp.mRot;
					Vector3 _origin = tmp.mPos;

					Vector3 ftr = _origin + (_edges[0] * 0.5f) + (_edges[1] * 0.5f) + (_edges[2] * 0.5f);
					Vector3 fbr = ftr - _edges[2];
					Vector3 ftl = ftr - _edges[0];
					Vector3 fbl = ftl - _edges[2];

					Vector3 btr = ftr - _edges[1];
					Vector3 bbr = btr - _edges[2];
					Vector3 btl = btr - _edges[0];
					Vector3 bbl = btl - _edges[2];

					isHit |= !Point_HitCheck_AABB(ftr, all);
					isHit |= !Point_HitCheck_AABB(fbr, all);
					isHit |= !Point_HitCheck_AABB(ftl, all);
					isHit |= !Point_HitCheck_AABB(fbl, all);

					isHit |= !Point_HitCheck_AABB(btr, all);
					isHit |= !Point_HitCheck_AABB(bbr, all);
					isHit |= !Point_HitCheck_AABB(btl, all);
					isHit |= !Point_HitCheck_AABB(bbl, all);
				}

				if (isHit)
				{
					tmp = roll;
					break;
				}
			}
			for (int z = 0; z <= 1000; ++z)
			{
				OBB roll = tmp;
				float add = static_cast<float>(z) * addv;
				tmp.mHalfSize.z += add;
				tmp.mPos.z += add;
				bool isHit = false;

				for (const Poly& p : wallPoly)
				{
					TriPlane tp(vertices[p.index[0]], vertices[p.index[1]], vertices[p.index[2]]);
					if (TriPlane_HitCheck_OBB(tp, tmp))
					{
						isHit = true;
						break;
					}
				}

				{
					Vector3 _edges[3]{ Vector3(tmp.mHalfSize.x * 2.f, 0, 0), Vector3(0, tmp.mHalfSize.y * 2.f, 0), Vector3(0, 0, tmp.mHalfSize.z * 2.f) };
					_edges[0] *= tmp.mRot;
					_edges[1] *= tmp.mRot;
					_edges[2] *= tmp.mRot;
					Vector3 _origin = tmp.mPos;

					Vector3 ftr = _origin + (_edges[0] * 0.5f) + (_edges[1] * 0.5f) + (_edges[2] * 0.5f);
					Vector3 fbr = ftr - _edges[2];
					Vector3 ftl = ftr - _edges[0];
					Vector3 fbl = ftl - _edges[2];

					Vector3 btr = ftr - _edges[1];
					Vector3 bbr = btr - _edges[2];
					Vector3 btl = btr - _edges[0];
					Vector3 bbl = btl - _edges[2];

					isHit |= !Point_HitCheck_AABB(ftr, all);
					isHit |= !Point_HitCheck_AABB(fbr, all);
					isHit |= !Point_HitCheck_AABB(ftl, all);
					isHit |= !Point_HitCheck_AABB(fbl, all);

					isHit |= !Point_HitCheck_AABB(btr, all);
					isHit |= !Point_HitCheck_AABB(bbr, all);
					isHit |= !Point_HitCheck_AABB(btl, all);
					isHit |= !Point_HitCheck_AABB(bbl, all);
				}

				if (isHit)
				{
					tmp = roll;
					break;
				}
			}

			float v = tmp.CalcVolume();
			if (v > volume)
			{
				volume = v;
				obb = tmp;
			}
		}
	}

	Draw(rooms, vertices, floorPoly, wallPoly);

	return (0);
}