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
#include "src/Math/Vector3.h"
#include "src/Physics/CollisionModel.h"
#include "src/Physics/Physics.h"
#include <string>
#include <cstdio>
#include <vector>
#include <algorithm>

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

struct Point
{
	pcl::PointXYZ pos;

	Point(pcl::PointXYZ _pos)
		: pos(_pos)
	{
	}
};
struct Cluster
{
	vector<Poly> polys;
	double e;
};

double Func_E(const Cluster& c, const vector<Vector3>& vertices)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	cloud->points.resize(c.polys.size() * 3);
	for (int i = 0; i < c.polys.size(); ++i)
	{
		Math::Vector3 xyz_i0 = vertices[c.polys[i].index[0]];
		Math::Vector3 xyz_i1 = vertices[c.polys[i].index[1]];
		Math::Vector3 xyz_i2 = vertices[c.polys[i].index[2]];

		cloud->points[i * 3 + 0] = xyz_i0.ToPointXYZ();
		cloud->points[i * 3 + 1] = xyz_i1.ToPointXYZ();
		cloud->points[i * 3 + 2] = xyz_i2.ToPointXYZ();
	}

	pcl::PointCloud<pcl::PointXYZ> hull;
	std::vector<pcl::Vertices> polygons;

	pcl::ConvexHull<pcl::PointXYZ> chull;
	chull.setInputCloud(cloud);
	chull.setDimension(2);
	chull.setComputeAreaVolume(true);
	chull.reconstruct(hull, polygons);
	double e = chull.getTotalArea();

	double vol = chull.getTotalVolume();
	assert(vol <= 0);


	return e;
}

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

//全体を囲むAABB、床ポリゴン、壁ポリゴンを取得
bool Func1(AABB& all, vector<Poly>& floorPoly, vector<Poly>& wallPoly
, const vector<Vector3>& vertices, const vector<Poly>& polys)
{
	all.mMin = all.mMax = vertices[0];
	for (const Vector3& v : vertices)
	{
		all.mMin.x = min(v.x, all.mMin.x);
		all.mMin.y = min(v.y, all.mMin.y);
		all.mMin.z = min(v.z, all.mMin.z);

		all.mMax.x = max(v.x, all.mMax.x);
		all.mMax.y = max(v.y, all.mMax.y);
		all.mMax.z = max(v.z, all.mMax.z);
	}


	for (const Poly& p : polys)
	{
		Vector3 a = vertices[p.index[1]] - vertices[p.index[0]];
		Vector3 b = vertices[p.index[2]] - vertices[p.index[0]];
		float dot = abs(a.cross(b).normalize().dot(Vector3(0, 1, 0)));

		float eps = 0.1f;
		if ((1.f - eps) <= dot && dot <= (1.f + eps))
		{
			floorPoly.push_back(p);//ほぼ平面なので床
		}
	}

	for (const Poly& p : polys)
	{
		Vector3 a = vertices[p.index[1]] - vertices[p.index[0]];
		Vector3 b = vertices[p.index[2]] - vertices[p.index[0]];
		float dot = abs(a.cross(b).dot(Vector3(0, 1, 0)));

		float eps = 0.1f;
		if ((0.f - eps) <= dot && dot <= (0.f + eps))
		{
			wallPoly.push_back(p);//ほぼ垂直なので壁
		}
	}

	return true;
}

//細分化して初期ルームを取得
bool Func2(vector<OBB>& rooms
	, const AABB& all)
{
	constexpr int div = 10;
	Vector3 vhs(0.5f, 0.5f, 0.5f);

	float xl = ((all.mMax.x - all.mMin.x) / div);
	float yl = ((all.mMax.y - all.mMin.y) / div);
	float zl = ((all.mMax.z - all.mMin.z) / div);

	float xlhs = xl * 0.5f;
	float ylhs = yl * 0.5f;
	float zlhs = zl * 0.5f;

	for (int x = 0; x < div; ++x)
	{
		for (int y = 0; y < div; ++y)
		{
			for (int z = 0; z < div; ++z)
			{
				Vector3 c(
					xlhs + static_cast<float>(x) * xl,
					ylhs + static_cast<float>(y) * yl,
					zlhs + static_cast<float>(z) * zl);
				rooms.push_back(OBB(all.mMin + c, vhs, Matrix44::identity()));
			}
		}
	}

	return true;
}

//ポリゴンに当たったり、屋外に置かれているルームの無効化
bool Func3(vector<OBB>& rooms
	, const AABB& all
	, const vector<Poly>& floorPoly, const vector<Poly>& wallPoly, const vector<Vector3>& vertices)
{
	//床、壁に当たるルームを削除
	vector<int> eraseList;
	{
		for (int r = 0; r < rooms.size(); ++r)
		{
			OBB& obb = rooms[r];
			for (const Poly& p : wallPoly)
			{
				TriPlane tp(vertices[p.index[0]], vertices[p.index[1]], vertices[p.index[2]]);
				if (TriPlane_HitCheck_OBB(tp, obb))
				{
					eraseList.push_back(r);
				}
			}
			for (const Poly& p : floorPoly)
			{
				TriPlane tp(vertices[p.index[0]], vertices[p.index[1]], vertices[p.index[2]]);
				if (TriPlane_HitCheck_OBB(tp, obb))
				{
					eraseList.push_back(r);
				}
			}
		}
	}

	//ルーム直下にレイを飛ばして床のポリゴンと判定、ひとつも当たらないものに関しては削除
	{
		const float lay_len = 50;
		for (int r = 0; r < rooms.size(); ++r)
		{
			OBB& obb = rooms[r];

			bool isHit = false;
			for (const Poly& p : floorPoly)
			{
				TriPlane tp(vertices[p.index[0]], vertices[p.index[1]], vertices[p.index[2]]);
				Lay lay(obb.mPos, Vector3(0, -lay_len, 0));
				if (Lay_HitCheck_TriPlane(nullptr, lay, tp))
				{
					isHit = true;
					break;
				}
			}

			if (!isHit)
			{
				eraseList.push_back(r);
			}
		}
	}

	std::sort(eraseList.begin(), eraseList.end(), std::greater<int>());
	eraseList.erase(std::unique(eraseList.begin(), eraseList.end()), eraseList.end());
	for (int idx : eraseList)
	{
		rooms.erase(rooms.begin() + idx);
	}


	return true;
}

//ルームを部屋から出ないように拡張していく
bool Func4(vector<OBB>& rooms
, const AABB& all
, const vector<Poly>& floorPoly, const vector<Poly>& wallPoly, const vector<Vector3>& vertices)
{
	//0～89度までY回転させつつ、軸を伸ばして体積が最大になる角度を探す
	//それからXYZをそれぞれポリゴンに当たるまで広げる
	for (int r = 0; r < rooms.size(); ++r)
	{
		OBB& obb = rooms[r];
		constexpr float addv = 0.05f;

		float volume = 0;
		for (int t = 0; t < 90; ++t)//角度
		{
			Matrix44 rot;
			rot.set_rotate_y(t);
			OBB tmp = obb;
			tmp.mRot = rot;
			tmp.mIvsRot = rot.inverse();

			//壁との当たりまで拡張
			for (int l = 0; l <= 1000; ++l)
			{
				OBB roll = tmp;
				float add = static_cast<float>(l) * addv;
				tmp.mHalfSize += tmp.mHalfSize.normalize() * add;

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
				for (int r2 = 0; r2 < r && !isHit; ++r2)
				{
					if (r == r2) { continue; }

					if (OBB_HitCheck_OBB(tmp, rooms[r2]))
					{
						isHit = true;
					}
				}
				bool isOut = !OBB_Contains_AABB(tmp, all);

				if (isHit || isOut)
				{
					tmp = roll;
					break;
				}
			}

			float v = tmp.CalcVolume();
			if (v > volume)
			{
				volume = v;
				obb.mRot = tmp.mRot;
				obb.mIvsRot = tmp.mIvsRot;
			}
		}

		//壁との当たりまで拡張
		for (int x = 0; x <= 1000; ++x)
		{
			OBB roll = obb;
			float add = static_cast<float>(x) * addv;
			obb.mHalfSize.x += add;
			obb.mPos.x -= add;

			bool isHit = false;
			for (const Poly& p : wallPoly)
			{
				TriPlane tp(vertices[p.index[0]], vertices[p.index[1]], vertices[p.index[2]]);
				if (TriPlane_HitCheck_OBB(tp, obb))
				{
					isHit = true;
					break;
				}
			}
			for (int r2 = 0; r2 < r && !isHit; ++r2)
			{
				if (r == r2) { continue; }

				if (OBB_HitCheck_OBB(obb, rooms[r2]))
				{
					isHit = true;
				}
			}
			bool isOut = !OBB_Contains_AABB(obb, all);

			if (isHit || isOut)
			{
				obb = roll;
				break;
			}
		}
		for (int x = 0; x <= 1000; ++x)
		{
			OBB roll = obb;
			float add = static_cast<float>(x) * addv;
			obb.mHalfSize.x += add;
			obb.mPos.x += add;

			bool isHit = false;
			for (const Poly& p : wallPoly)
			{
				TriPlane tp(vertices[p.index[0]], vertices[p.index[1]], vertices[p.index[2]]);
				if (TriPlane_HitCheck_OBB(tp, obb))
				{
					isHit = true;
					break;
				}
			}
			for (int r2 = 0; r2 < r && !isHit; ++r2)
			{
				if (r == r2) { continue; }

				if (OBB_HitCheck_OBB(obb, rooms[r2]))
				{
					isHit = true;
				}
			}
			bool isOut = !OBB_Contains_AABB(obb, all);

			if (isHit || isOut)
			{
				obb = roll;
				break;
			}
		}
		for (int z = 0; z <= 1000; ++z)
		{
			OBB roll = obb;
			float add = static_cast<float>(z) * addv;
			obb.mHalfSize.z += add;
			obb.mPos.z -= add;

			bool isHit = false;
			for (const Poly& p : wallPoly)
			{
				TriPlane tp(vertices[p.index[0]], vertices[p.index[1]], vertices[p.index[2]]);
				if (TriPlane_HitCheck_OBB(tp, obb))
				{
					isHit = true;
					break;
				}
			}
			for (int r2 = 0; r2 < r && !isHit; ++r2)
			{
				if (r == r2) { continue; }

				if (OBB_HitCheck_OBB(obb, rooms[r2]))
				{
					isHit = true;
				}
			}
			bool isOut = !OBB_Contains_AABB(obb, all);

			if (isHit || isOut)
			{
				obb = roll;
				break;
			}
		}
		for (int z = 0; z <= 1000; ++z)
		{
			OBB roll = obb;
			float add = static_cast<float>(z) * addv;
			obb.mHalfSize.z += add;
			obb.mPos.z += add;

			bool isHit = false;
			for (const Poly& p : wallPoly)
			{
				TriPlane tp(vertices[p.index[0]], vertices[p.index[1]], vertices[p.index[2]]);
				if (TriPlane_HitCheck_OBB(tp, obb))
				{
					isHit = true;
					break;
				}
			}
			for (int r2 = 0; r2 < r && !isHit; ++r2)
			{
				if (r == r2) { continue; }

				if (OBB_HitCheck_OBB(obb, rooms[r2]))
				{
					isHit = true;
				}
			}
			bool isOut = !OBB_Contains_AABB(obb, all);

			if (isHit || isOut)
			{
				obb = roll;
				break;
			}
		}

		//床・天井との当たりまで拡張
		for (int y = 0; y <= 1000; ++y)
		{
			OBB roll = obb;
			float add = static_cast<float>(y) * addv;
			obb.mHalfSize.y += add;
			obb.mPos.y -= add;

			bool isHit = false;
			for (const Poly& p : floorPoly)
			{
				TriPlane tp(vertices[p.index[0]], vertices[p.index[1]], vertices[p.index[2]]);
				if (TriPlane_HitCheck_OBB(tp, obb))
				{
					isHit = true;
					break;
				}
			}
			for (int r2 = 0; r2 < r && !isHit; ++r2)
			{
				if (r == r2) { continue; }

				if (OBB_HitCheck_OBB(obb, rooms[r2]))
				{
					isHit = true;
				}
			}
			bool isOut = !OBB_Contains_AABB(obb, all);

			if (isHit || isOut)
			{
				obb = roll;
				break;
			}
		}
		for (int y = 0; y <= 1000; ++y)
		{
			OBB roll = obb;
			float add = static_cast<float>(y) * addv;
			obb.mHalfSize.y += add;
			obb.mPos.y += add;

			bool isHit = false;
			for (const Poly& p : floorPoly)
			{
				TriPlane tp(vertices[p.index[0]], vertices[p.index[1]], vertices[p.index[2]]);
				if (TriPlane_HitCheck_OBB(tp, obb))
				{
					isHit = true;
					break;
				}
			}
			for (int r2 = 0; r2 < r && !isHit; ++r2)
			{
				if (r == r2) { continue; }

				if (OBB_HitCheck_OBB(obb, rooms[r2]))
				{
					isHit = true;
				}
			}
			bool isOut = !OBB_Contains_AABB(obb, all);

			if (isHit || isOut)
			{
				obb = roll;
				break;
			}
		}
	}

	return true;
}

//余分なルームの削除、連結
bool Func5(vector<OBB>& rooms)
{
	vector<int> eraseList;
	for (int i = 0; i < rooms.size(); ++i)
	{
		for (int j = 0; j < rooms.size(); ++j)
		{
			if (i == j) { continue; }

			if (OBB_Contains_OBB(rooms[i], rooms[j]))
			{
				eraseList.push_back(i);
			}
		}
	}

	std::sort(eraseList.begin(), eraseList.end(), std::greater<int>());
	eraseList.erase(std::unique(eraseList.begin(), eraseList.end()), eraseList.end());

	for (int idx : eraseList)
	{
		rooms.erase(rooms.begin() + idx);
	}

	return true;
}

//現在のルームサイズを初期化して、分割数が最小になるよう試す
//順列でFunc2～5を試す？
bool Func6(vector<OBB>& rooms
	, const AABB& all
	, const vector<Poly>& floorPoly, const vector<Poly>& wallPoly, const vector<Vector3>& vertices)
{
	vector<OBB> firstRooms = rooms;
	int divNum = firstRooms.size();

	vector<int> indices(firstRooms.size());
	std::iota(indices.begin(), indices.end(), 0);
	while (next_permutation(indices.begin(), indices.end()))//一番最初の順列のパターンは既に分割済みだから無視でいい
	{
		//並び変えたルームを作る
		vector<OBB> tmpRooms;
		tmpRooms.reserve(5000);
		for (int i = 0; i < indices.size(); ++i)
		{
			AABB tmpAll(firstRooms[indices[i]]);
			if (Func2(tmpRooms, tmpAll))
			{
				if (Func3(tmpRooms, tmpAll, floorPoly, wallPoly, vertices))
				{

				}
			}
		}

		//分割のし直し
		if (Func4(tmpRooms, all, floorPoly, wallPoly, vertices))
		{
			if (Func5(tmpRooms))
			{
				if (tmpRooms.size() < divNum)
				{
					divNum = tmpRooms.size();
					rooms = tmpRooms;
				}
			}
		}
	};

	return true;
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
	vector<Poly> floorPoly;
	vector<Poly> wallPoly;
	if (Func1(all, floorPoly, wallPoly, vertices, polys))
	{
		vector<OBB> rooms;
		if (Func2(rooms, all))
		{
			if (Func3(rooms, all, floorPoly, wallPoly, vertices))
			{
				if (Func4(rooms, all, floorPoly, wallPoly, vertices))
				{
					if (Func5(rooms))
					{
						if (Func6(rooms, all, floorPoly, wallPoly, vertices))
						{
							Draw(rooms, vertices, floorPoly, wallPoly);
						}
					}
				}
			}
		}
	}


	return (0);
}