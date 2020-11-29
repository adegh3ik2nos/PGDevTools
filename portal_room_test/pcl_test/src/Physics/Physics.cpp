#include "src/Physics/Physics.h"

#include "src/Physics/CollisionModel.h"
#include "src/Math/Math.h"
#include "src/Math/Matrix44.h"
#include "src/Math/Vector3.h"

using namespace Math;

namespace Physics {


	//�_��������ւ̍ŒZ�x�N�g��
	void ShortVec_Of_Point_And_Lay(
		Math::Vector3* short_vec,
		const Math::Vector3& pos,
		const Physics::Lay& lay) {
		//�|�C���^��nullptr��������I��
		if (!short_vec) return;
		//�����ƍŒZ�x�N�g���̂Ԃ���ʒu�̎n�_����̔䗦
		float t = lay.mVec.normalize().dot(pos - lay.mPos) / lay.mVec.length();
		//�ŒZ�x�N�g���ƌ����ʒu
		Math::Vector3 short_pos;
		if (t < 0.f) { short_pos = lay.mPos; }
		else if (t > 1.f) { short_pos = lay.mPos + lay.mVec; }
		else { short_pos = lay.mPos + t * lay.mVec; }
		//�ŒZ�x�N�g��
		*short_vec = short_pos - pos;
	}
	//�_��AABB�̍ŒZ����
	float ShortLen_Of_Point_And_AABB(
		const Math::Vector3& pos, const AABB& aabb) {
		//������2��
		float len_2 = 0.f;
		//�����Ƃ̋�����2������
		//X��
		float v = pos.x;
		float min = aabb.mMin.x;
		float max = aabb.mMax.x;
		if (v < min) { len_2 += (v - min) * (v - min); }
		else if (v > max) { len_2 += (v - max) * (v - max); }
		//Y��
		v = pos.y;
		min = aabb.mMin.y;
		max = aabb.mMax.y;
		if (v < min) { len_2 += (v - min) * (v - min); }
		else if (v > max) { len_2 += (v - max) * (v - max); }
		//Z��
		v = pos.z;
		min = aabb.mMin.z;
		max = aabb.mMax.z;
		if (v < min) { len_2 += (v - min) * (v - min); }
		else if (v > max) { len_2 += (v - max) * (v - max); }
		//������ċ����ɂ��ĕԂ�
		return Math::Sqrt(len_2);
	}
	//�_��OBB�̍ŒZ����
	float ShortLen_Of_Point_And_OBB(
		const Math::Vector3& pos, const OBB& obb) {
		//�t�s��œ_�𑊑Έʒu�ɂ���
		Math::Vector3 rtv_pos;
		obb.mIvsRot.multiply(&rtv_pos, pos - obb.mPos);
		//OBB��AABB�ɒ����Ĕ���
		return ShortLen_Of_Point_And_AABB(
			rtv_pos, AABB(-obb.mHalfSize, obb.mHalfSize));
	}
	//���΃x�N�g��
	void Relative_Vector(
		Math::Vector3* rtv_vec,
		const Math::Vector3& vec, const Math::Vector3& angle) {
		//���g���Ȃ���ΏI��
		if (!rtv_vec)return;
		//���Ή�
		Math::Matrix44 rot;
		rot.set_rotate_z(-angle.z);
		rot.rotate_x(-angle.x);
		rot.rotate_y(-angle.y);
		rot.multiply(rtv_vec, vec);
	}
	//���P�ʂ̊p�x
	float angle_x(const Math::Vector3& vec) {
		float ax = Math::Atan2(vec.y, -vec.z);
		return ax;
	}
	float angle_y(const Math::Vector3& vec) {
		float ay = Math::Atan2(vec.x, vec.z) + 180.f;
		if (ay > 180.f) { ay -= 360.f; }
		return ay;
	}
	float angle_z(const Math::Vector3& vec) {
		float az = Math::Atan2(vec.y, vec.x) + 180.f;
		if (az > 180.f) { az -= 360.f; }
		return az;
	}
	//�x�N�g���̊p�x
	void vec_angle(Math::Vector3* angle, const Math::Vector3& vec) {
		//��Ȃ�I��
		if (!angle) return;
		//�p�x�擾
		float l = Math::Sqrt(vec.x*vec.x + vec.z*vec.z);
		float ax = Math::Atan2(vec.y, l);
		float ay = Math::Atan2(vec.x, vec.z) + 180.f;
		//�p�x�␳
		if (ay > 180.f) { ay -= 360.f; }
		*angle = Math::Vector3(ax, ay, 0.f);
	}
	//���������
	bool In_View(
		const Math::Vector3& vec,
		const Math::Vector3& angle,
		float view_angle_x, float view_angle_y,
		float view_lengh){
		//��������
		float length = vec.square_length();
		if (length > view_lengh * view_lengh) return false;
		//�p�x�𔻒�
		Math::Vector3 rot_vec;
		Relative_Vector(&rot_vec, vec, angle);
		//X��]
		float ax = 0.f;
		length = Math::Vector2(rot_vec.y, rot_vec.z).square_length();
		if (length > 0.f) { ax = angle_x(rot_vec); }
		ax = Math::Fabs(ax) * 2.f;
		//Y��]
		float ay = 0.f;
		length = Math::Vector2(rot_vec.x, rot_vec.z).square_length();
		if (length > 0.f) { ay = angle_y(rot_vec); }
		ay = Math::Fabs(ay) * 2.f;
		return ax <= view_angle_x && ay <= view_angle_y;
	}

	//�Փ˔���(��{)
	bool Sphere_HitCheck_Sphere(
		const Sphere& a, const Sphere& b) {
		//�v�Z���׌y���̂��ߋ������g���̂ł͂Ȃ�2����g���Ă���
		float ab_2 = (b.mPos - a.mPos).square_length();
		float tr_2 = (a.mRds + b.mRds) * (a.mRds + b.mRds);
		return ab_2 <= tr_2;
	}
	bool Lay_HitCheck_Sphere(
		const Lay& lay, const Sphere& sp) {
		//�����Ƃ̍ŒZ���������a�ȉ��Ȃ�Փ�
		Math::Vector3 short_vec;
		ShortVec_Of_Point_And_Lay(
			&short_vec, sp.mPos, lay);
		return short_vec.length() <= sp.mRds;
	}
	bool Lay_HitCheck_Sphere(
		Math::Vector3* hit_pos,
		const Lay& lay, const Sphere& sp) {
		//���̒��S��������̎n�_�Ɍ������x�N�g��
		Math::Vector3 sp_lay = lay.mPos - sp.mPos;
		//���ꂼ��̌W�����o��
		float a = lay.mVec.square_length();
		float b = 2.f * lay.mVec.dot(sp_lay);
		float c = sp_lay.square_length() - sp.mRds * sp.mRds;
		//���ʎ�
		float d = b * b - 4.f * a * c;
		//�d�Ȃ�Ȃ��ꍇ��false�ŕԂ�
		if (d < 0.f) return false;
		//�����_���o��
		d = Math::Sqrt(d);
		//�Փ˓_�̔䗦
		float t0 = (-b + d) / (2.f * a);
		float t1 = (-b - d) / (2.f * a);
		//�Ŋ��Ŕ͈͓���t��T��
		float t = 2.f;
		if (t0 >= 0.f && t0 <= 1.f && t0 < t) { t = t0; }
		if (t1 >= 0.f && t1 <= 1.f && t1 < t) { t = t1; }
		if (t > 1.f) return false;
		//�Ŋ��̏Փ˓_
		if (hit_pos) {
			*hit_pos = lay.mPos + t * lay.mVec;
		}
		return true;
	}
	bool Lay_HitCheck_TriPlane(
		Math::Vector3* hit_pos,
		const Lay& lay, const TriPlane& tp) {
		//��΂ɓ͂��Ȃ���ΏI��
		float len_tp_lp = tp.mNormal.dot(lay.mPos) + tp.mD;
		if (len_tp_lp > lay.mVec.length()) return false;
		//���s�Ȓʉ߂ł���ΏI��
		float dot_nv = tp.mNormal.dot(lay.mVec);
		if (dot_nv == 0.f) return false;
		//���������ʂ�ʉ߂��邩
		float t = -len_tp_lp / dot_nv;
		if (t < 0.f || t > 1.f) return false;
		//�Փˈʒu
		Math::Vector3 hp = lay.mPos + t * lay.mVec;
		//�Փˈʒu���O�p���ʓ���
		Math::Vector3 crs0 = (tp.mP1 - tp.mP0).cross(hp - tp.mP1);
		Math::Vector3 crs1 = (tp.mP2 - tp.mP1).cross(hp - tp.mP2);
		Math::Vector3 crs2 = (tp.mP0 - tp.mP2).cross(hp - tp.mP0);
		float dot0 = tp.mNormal.dot(crs0);
		float dot1 = tp.mNormal.dot(crs1);
		float dot2 = tp.mNormal.dot(crs2);
		if (dot0 < 0.f || dot1 < 0.f || dot2 < 0.f) return false;
		//�Փˈʒu�擾
		if (hit_pos) {
			*hit_pos = hp;
		}
		return true;
	}
	bool Sphere_HitCheck_TriPlane(
		const Sphere& sp, const TriPlane& tp) {
		//�������ʂɓ������Ă��Ȃ���� false
		float len_tp_sp = tp.mNormal.dot(sp.mPos) + tp.mD;
		if (Math::Fabs(len_tp_sp) > sp.mRds) return false;
		//�O�p���ʂ�3��
		Math::Vector3 vec_01 = tp.mP1 - tp.mP0;
		Math::Vector3 vec_12 = tp.mP2 - tp.mP1;
		Math::Vector3 vec_20 = tp.mP0 - tp.mP2;
		//�O�p���ʂ�3�ӂƓ������Ă���� true
		if (Lay_HitCheck_Sphere(Lay(tp.mP0, vec_01), sp)) return true;
		if (Lay_HitCheck_Sphere(Lay(tp.mP1, vec_12), sp)) return true;
		if (Lay_HitCheck_Sphere(Lay(tp.mP2, vec_20), sp)) return true;
		//�O�p���ʂ͈͓̔��ł���� true
		Math::Vector3 crs0 = (vec_01).cross(sp.mPos - tp.mP1);
		Math::Vector3 crs1 = (vec_12).cross(sp.mPos - tp.mP2);
		Math::Vector3 crs2 = (vec_20).cross(sp.mPos - tp.mP0);
		float dot0 = tp.mNormal.dot(crs0);
		float dot1 = tp.mNormal.dot(crs1);
		float dot2 = tp.mNormal.dot(crs2);
		if (dot0 >= 0.f && dot1 >= 0.f && dot2 >= 0.f) return true;
		//�O�p���ʂ̕ӂƂ��ʂƂ�������Ȃ���� false
		return false;
	}
	bool TriPlane_HitCheck_TriPlane(
		const TriPlane& a, const TriPlane& b) {
		//�g������
		Lay lay0_0(a.mP0, a.mP1 - a.mP0);
		Lay lay0_1(a.mP1, a.mP2 - a.mP1);
		Lay lay0_2(a.mP2, a.mP0 - a.mP2);
		bool hit0_0 = Lay_HitCheck_TriPlane(nullptr, lay0_0, b);
		bool hit0_1 = Lay_HitCheck_TriPlane(nullptr, lay0_1, b);
		bool hit0_2 = Lay_HitCheck_TriPlane(nullptr, lay0_2, b);
		//�Е���2�ӂ��O�p���ʂ��т��Ă��邩
		if (hit0_0 && hit0_1 ||
			hit0_1 && hit0_2 ||
			hit0_2 && hit0_0) {
			return true;
		}
		//�g������
		Lay lay1_0(b.mP0, b.mP1 - b.mP0);
		Lay lay1_1(b.mP1, b.mP2 - b.mP1);
		Lay lay1_2(b.mP2, b.mP0 - b.mP2);
		bool hit1_0 = Lay_HitCheck_TriPlane(nullptr, lay1_0, a);
		bool hit1_1 = Lay_HitCheck_TriPlane(nullptr, lay1_1, a);
		bool hit1_2 = Lay_HitCheck_TriPlane(nullptr, lay1_2, a);
		//�Е���2�ӂ��O�p���ʂ��т��Ă��邩
		if (hit1_0 && hit1_1 ||
			hit1_1 && hit1_2 ||
			hit1_2 && hit1_0) {
			return true;
		}
		//�݂���2�ӂÂт��Ă��邩
		if ((hit0_0 || hit0_1 || hit0_2) &&
			(hit1_0 || hit1_1 || hit1_2)) {
			return true;
		}
		return false;
	}
	bool Sphere_HitCheck_AABB(
		const Sphere& sp, const AABB& aabb) {
		//�ŒZ���������̔��a�ȉ��Ȃ�Փ�
		float length = ShortLen_Of_Point_And_AABB(sp.mPos, aabb);
		return length <= sp.mRds;
	}
	//�_��AABB
	bool Point_HitCheck_AABB(
		const Vector3& p, const AABB& aabb)
	{
		return (aabb.mMin.x <= p.x && aabb.mMax.x >= p.x)
			&& (aabb.mMin.y <= p.y && aabb.mMax.y >= p.y)
			&& (aabb.mMin.z <= p.z && aabb.mMax.z >= p.z);
	}
	bool Lay_HitCheck_AABB(
		const Lay& lay, const AABB& aabb) {
		//�����Ƃɔ���
		float t_min, t_max;
		//X��
		if (lay.mVec.x == 0.f) {
			//�͈͊O�Ȃ�I��
			if (lay.mPos.x < aabb.mMin.x || lay.mPos.x > aabb.mMax.x) return false;
			t_min = 0.f;
			t_max = 1.f;
		}
		else {
			//X���ł͈͔̔䗦
			float t0 = (aabb.mMin.x - lay.mPos.x) / lay.mVec.x;
			float t1 = (aabb.mMax.x - lay.mPos.x) / lay.mVec.x;
			if (t0 < t1) {
				t_min = t0;
				t_max = t1;
			}
			else {
				t_min = t1;
				t_max = t0;
			}
			//���Ԃ�Ȃ���ΏI��
			if (t_min > 1.f || t_max < 0.f) return false;
		}
		float min = t_min;
		float max = t_max;
		//Y��
		if (lay.mVec.y == 0.f) {
			//�͈͊O�Ȃ�I��
			if (lay.mPos.y < aabb.mMin.y || lay.mPos.y > aabb.mMax.y) return false;
			t_min = 0.f;
			t_max = 1.f;
		}
		else {
			//X���ł͈͔̔䗦
			float t0 = (aabb.mMin.y - lay.mPos.y) / lay.mVec.y;
			float t1 = (aabb.mMax.y - lay.mPos.y) / lay.mVec.y;
			if (t0 < t1) {
				t_min = t0;
				t_max = t1;
			}
			else {
				t_min = t1;
				t_max = t0;
			}
			//���Ԃ�Ȃ���ΏI��
			if (t_min > 1.f || t_max < 0.f) return false;
		}
		//X���Ƃ��Ԃ�Ȃ���ΏI��
		if (t_min > max || t_max < min) return false;
		//X���Ƃ��Ԃ�̈�ɕ␳
		if (t_min > min) { min = t_min; }
		if (t_max < max) { max = t_max; }
		//Z��
		if (lay.mVec.z == 0.f) {
			//�͈͊O�Ȃ�I��
			if (lay.mPos.z < aabb.mMin.z || lay.mPos.z > aabb.mMax.z) return false;
			t_min = 0.f;
			t_max = 1.f;
		}
		else {
			//X���ł͈͔̔䗦
			float t0 = (aabb.mMin.z - lay.mPos.z) / lay.mVec.z;
			float t1 = (aabb.mMax.z - lay.mPos.z) / lay.mVec.z;
			if (t0 < t1) {
				t_min = t0;
				t_max = t1;
			}
			else {
				t_min = t1;
				t_max = t0;
			}
			//���Ԃ�Ȃ���ΏI��
			if (t_min > 1.f || t_max < 0.f) return false;
		}
		//XY���Ƃ��Ԃ�Ȃ���ΏI��
		if (t_min > max || t_max < min) return false;
		//XY���Ƃ��Ԃ�̈�ɕ␳
		if (t_min > min) { min = t_min; }
		if (t_max < max) { max = t_max; }

		//���ʗ̈�Ŕ͈̓`�F�b�N
		return min <= 1.f && max >= 0.f;
	}
	bool AABB_HitCheck_AABB(
		const AABB& a, const AABB& b) {
		//�S�Ă̎���1���ł����Ԃ�Ȃ�Փ�
		bool hit =
			a.mMin.x <= b.mMax.x && a.mMax.x >= b.mMin.x &&
			a.mMin.y <= b.mMax.y && a.mMax.y >= b.mMin.y &&
			a.mMin.z <= b.mMax.z && a.mMax.z >= b.mMin.z;
		return hit;
	}
	//�_��OBB
	bool Point_HitCheck_OBB(
		const Math::Vector3& p, const OBB& obb)
	{
		//�ꉞ�덷�܂�
		return (0.f + FLT_EPSILON) >= ShortLen_Of_Point_And_OBB(p, obb);
	}
	bool Sphere_HitCheck_OBB(
		const Sphere& sp, const OBB& obb) {
		//�ŒZ���������̔��a�ȉ��Ȃ�Փ�
		float length = ShortLen_Of_Point_And_OBB(sp.mPos, obb);
		return length <= sp.mRds;
	}
	bool Lay_HitCheck_OBB(
		const Lay& lay, const OBB& obb) {
		//�t�s��Ő����̎n�_�𑊑Έʒu�ɂ���
		Lay rtv_lay;
		obb.mIvsRot.multiply(&rtv_lay.mPos, lay.mPos - obb.mPos);
		obb.mIvsRot.multiply(&rtv_lay.mVec, lay.mVec);
		//OBB��AABB�ɒ����Ĕ���
		return Lay_HitCheck_AABB(rtv_lay, AABB(-obb.mHalfSize, obb.mHalfSize));
	}
	bool TriPlane_HitCheck_OBB(
		const TriPlane& tp, const OBB& obb) {
		//�������ʂƂ̔���
		//obb�̕����x�N�g��
		Math::Vector3 dir[3] = {
			Math::Vector3(obb.mHalfSize.x, 0.f, 0.f),
			Math::Vector3(0.f, obb.mHalfSize.y, 0.f),
			Math::Vector3(0.f, 0.f, obb.mHalfSize.z),
		};
		//obb�����݂̕����ɒ���
		for (int n = 0; n < 3; ++n) {
			obb.mRot.multiply(&dir[n], dir[n]);
		}
		//���e����obb�̑傫���̔���
		float len_a = 0.f;
		for (int n = 0; n < 3; ++n) {
			len_a += Math::Fabs(tp.mNormal.dot(dir[n]));
		}
		//���e����obb�̒��S�_�Ɩʂ̋���
		float len_b = Math::Fabs(tp.mNormal.dot(obb.mPos - tp.mP0));
		//���e���������̂ق����傫����� false
		if (len_b > len_a) return false;
		//�O�p���ʂ�3��
		Math::Vector3 vec_01 = tp.mP1 - tp.mP0;
		Math::Vector3 vec_12 = tp.mP2 - tp.mP1;
		Math::Vector3 vec_20 = tp.mP0 - tp.mP2;
		//�O�p���ʂ�3�ӂƓ������Ă���� true
		if (Lay_HitCheck_OBB(Lay(tp.mP0, vec_01), obb)) return true;
		if (Lay_HitCheck_OBB(Lay(tp.mP1, vec_12), obb)) return true;
		if (Lay_HitCheck_OBB(Lay(tp.mP2, vec_20), obb)) return true;
		//�O�p���ʂ͈͓̔��ł���� true
		Math::Vector3 crs0 = (vec_01).cross(obb.mPos - tp.mP1);
		Math::Vector3 crs1 = (vec_12).cross(obb.mPos - tp.mP2);
		Math::Vector3 crs2 = (vec_20).cross(obb.mPos - tp.mP0);
		float dot0 = tp.mNormal.dot(crs0);
		float dot1 = tp.mNormal.dot(crs1);
		float dot2 = tp.mNormal.dot(crs2);
		if (dot0 >= 0.f && dot1 >= 0.f && dot2 >= 0.f) return true;
		//�O�p���ʂ̕ӂƂ��ʂƂ�������Ȃ���� false
		return false;
	}
	bool OBB_HitCheck_OBB(const OBB& a, const OBB& b) {
		//OBB_a�̕����x�N�g��
		Math::Vector3 a_dir[3] = {
			Math::Vector3(a.mHalfSize.x, 0.f, 0.f),
			Math::Vector3(0.f, a.mHalfSize.y, 0.f),
			Math::Vector3(0.f, 0.f, a.mHalfSize.z),
		};
		//OBB_a�����݂̕����ɒ���
		for (int an = 0; an < 3; ++an) {
			a.mRot.multiply(&a_dir[an], a_dir[an]);
		}
		//OBB_b�̕����x�N�g��
		Math::Vector3 b_dir[3] = {
			Math::Vector3(b.mHalfSize.x, 0.f, 0.f),
			Math::Vector3(0.f, b.mHalfSize.y, 0.f),
			Math::Vector3(0.f, 0.f, b.mHalfSize.z),
		};
		//OBB_b�����݂̕����ɒ���
		for (int bn = 0; bn < 3; ++bn) {
			b.mRot.multiply(&b_dir[bn], b_dir[bn]);
		}
		//OBB_a��OBB_b�̃x�N�g����
		Math::Vector3 interval = a.mPos - b.mPos;
		//����
		//OBB_a�̕������Ŕ���
		for (int an = 0; an < 3; ++an) {
			//OBB_a�̓��e�������߂�
			float len_a = a_dir[an].length();
			//OBB_b�̓��e�������߂�
			Math::Vector3 nml_dir = a_dir[an].normalize();
			float len_b = 0.f;
			for (int bn = 0; bn < 3; ++bn) {
				len_b += Math::Fabs(nml_dir.dot(b_dir[bn]));
			}
			//�x�N�g�����̓��e��
			float len = Math::Fabs(nml_dir.dot(interval));
			//����Ă���ΏI��
			if (len > len_a + len_b) return false;
		}
		//OBB_b�̕������Ŕ���
		for (int bn = 0; bn < 3; ++bn) {
			//OBB_b�̓��e�������߂�
			float len_b = b_dir[bn].length();
			//OBB_a�̓��e�������߂�
			Math::Vector3 nml_dir = b_dir[bn].normalize();
			float len_a = 0.f;
			for (int an = 0; an < 3; ++an) {
				len_a += Math::Fabs(nml_dir.dot(a_dir[an]));
			}
			//�x�N�g�����̓��e��
			float len = Math::Fabs(nml_dir.dot(interval));
			//����Ă���ΏI��
			if (len > len_a + len_b) return false;
		}
		//OBB_a��OBB_b�̊O�ς𕪗����Ƃ��Ĕ���
		for (int an = 0; an < 3; ++an) {
			for (int bn = 0; bn < 3; ++bn) {
				//���e���̎�
				Math::Vector3 crs_dir = a_dir[an].cross(b_dir[bn]).normalize();
				//a�̓��e�������߂�
				float len_a = 0.f;
				for (int o = 0; o < 3; ++o) {
					//�@���͐����œ��ό��ʂ�0�Ȃ̂ŃX���[
					if (o == an) continue;
					//���e����ǉ�
					len_a += Math::Fabs(crs_dir.dot(a_dir[o]));
				}
				//b�̓��e��
				float len_b = 0.f;
				for (int o = 0; o < 3; ++o) {
					//�@���͐����œ��ό��ʂ�0�Ȃ̂ŃX���[
					if (o == bn) continue;
					//���e����ǉ�
					len_b += Math::Fabs(crs_dir.dot(b_dir[o]));
				}
				//�x�N�g�����̓��e��
				float len = Math::Fabs(crs_dir.dot(interval));
				//����Ă���ΏI��
				if (len > len_a + len_b) return false;
			}
		}
		//�������ʂ��Ȃ���Γ������Ă���
		return true;
	}


	//OBB��AABB�Ɋ܂܂�邩
	bool OBB_Contains_AABB(const OBB& obb, const AABB& aabb)
	{
		Math::Vector3 _edges[3]{ Math::Vector3(obb.mHalfSize.x * 2.f, 0, 0), Math::Vector3(0, obb.mHalfSize.y * 2.f, 0), Math::Vector3(0, 0, obb.mHalfSize.z * 2.f) };
		_edges[0] *= obb.mRot;
		_edges[1] *= obb.mRot;
		_edges[2] *= obb.mRot;
		Math::Vector3 _origin = obb.mPos;

		Math::Vector3 ftr = _origin + (_edges[0] * 0.5f) + (_edges[1] * 0.5f) + (_edges[2] * 0.5f);
		Math::Vector3 fbr = ftr - _edges[2];
		Math::Vector3 ftl = ftr - _edges[0];
		Math::Vector3 fbl = ftl - _edges[2];

		Math::Vector3 btr = ftr - _edges[1];
		Math::Vector3 bbr = btr - _edges[2];
		Math::Vector3 btl = btr - _edges[0];
		Math::Vector3 bbl = btl - _edges[2];

		bool isHit = true;
		isHit &= Point_HitCheck_AABB(ftr, aabb);
		isHit &= Point_HitCheck_AABB(fbr, aabb);
		isHit &= Point_HitCheck_AABB(ftl, aabb);
		isHit &= Point_HitCheck_AABB(fbl, aabb);

		isHit &= Point_HitCheck_AABB(btr, aabb);
		isHit &= Point_HitCheck_AABB(bbr, aabb);
		isHit &= Point_HitCheck_AABB(btl, aabb);
		isHit &= Point_HitCheck_AABB(bbl, aabb);

		return isHit;
	}

	//a��b�Ɋ܂܂�邩
	bool OBB_Contains_OBB(const OBB& a, const OBB& b)
	{
		Math::Vector3 _edges[3]{ Math::Vector3(a.mHalfSize.x * 2.f, 0, 0), Math::Vector3(0, a.mHalfSize.y * 2.f, 0), Math::Vector3(0, 0, a.mHalfSize.z * 2.f) };
		_edges[0] *= a.mRot * b.mIvsRot;
		_edges[1] *= a.mRot * b.mIvsRot;
		_edges[2] *= a.mRot * b.mIvsRot;
		Math::Vector3 _origin = a.mPos;

		Math::Vector3 ftr = _origin + (_edges[0] * 0.5f) + (_edges[1] * 0.5f) + (_edges[2] * 0.5f);
		Math::Vector3 fbr = ftr - _edges[2];
		Math::Vector3 ftl = ftr - _edges[0];
		Math::Vector3 fbl = ftl - _edges[2];

		Math::Vector3 btr = ftr - _edges[1];
		Math::Vector3 bbr = btr - _edges[2];
		Math::Vector3 btl = btr - _edges[0];
		Math::Vector3 bbl = btl - _edges[2];

		bool isHit = true;
		isHit &= Point_HitCheck_OBB(ftr, b);
		isHit &= Point_HitCheck_OBB(fbr, b);
		isHit &= Point_HitCheck_OBB(ftl, b);
		isHit &= Point_HitCheck_OBB(fbl, b);

		isHit &= Point_HitCheck_OBB(btr, b);
		isHit &= Point_HitCheck_OBB(bbr, b);
		isHit &= Point_HitCheck_OBB(btl, b);
		isHit &= Point_HitCheck_OBB(bbl, b);

		return isHit;
	}


	//�^�����f��
	void Rebound_Exercise(
		Math::Vector3* get_vel_a, Math::Vector3* get_vel_b,
		const Math::Vector3& vel_a, const Math::Vector3& vel_b,
		const Math::Vector3& pos_a, const Math::Vector3& pos_b,
		float mass_a, float mass_b, float rb_coef_a, float rb_coef_b) {
		if (!get_vel_a && !get_vel_b) return;
		//���ꂼ��Ɍ������x�N�g��
		Math::Vector3 vec_ab(pos_a - pos_b);
		Math::Vector3 vec_ba(pos_b - pos_a);
		//�Ƃ肠�������ʁA�����W���𖳎��������˃x�N�g�������߂�
		//����0
		vec_ab.set_normalize();
		float length = vec_ab.dot(vel_a);
		Math::Vector3 deciA2 = vec_ab * length;
		Math::Vector3 A1 = vel_a - deciA2;
		//����1
		vec_ba.set_normalize();
		length = vec_ba.dot(vel_b);
		Math::Vector3 deciB2 = vec_ba * length;
		Math::Vector3 B1 = vel_b - deciB2;
		//���ʂ����ł͂Ȃ��ꍇ�̒e���Փ˂̍ŏI���x: ���ʏՓ˂͉^���ʕۑ��̖@��
		Math::Vector3 a(deciA2 * mass_a + deciB2 * mass_b);
		Math::Vector3 b((deciA2 - deciB2) * -(rb_coef_a * rb_coef_b));
		Math::Vector3 A2 = (a + b * mass_b) / (mass_a + mass_b);
		Math::Vector3 B2 = A2 - b;
		//���������̂����ꂼ��̔��˃x�N�g��
		if (get_vel_a) { *get_vel_a = A1 + A2; }
		if (get_vel_b) { *get_vel_b = B1 + B2; }
	}
	void Reflection_Exercise(
		Math::Vector3* get_vel,
		const Math::Vector3& vel, float rb_coef,
		const Math::Vector3& normal) {
		if (!get_vel) return;
		//���������̔��˃x�N�g��: ���̂܂�
		Math::Vector3 ref = normal * -normal.dot(vel);
		//���������̔��˃x�N�g��: �����W���l��
		Math::Vector3 n_ref = ref * rb_coef;
		//���������̔��˃x�N�g��
		Math::Vector3 h_ref = vel + ref;
		//���ꂼ��̔��˃x�N�g�����������x��ύX
		*get_vel = n_ref + h_ref;
	}
	void WallSlide_Exercise(
		Math::Vector3* get_vel,
		const Math::Vector3& vel, const Math::Vector3& normal) {
		//���������̔��˃x�N�g��
		Math::Vector3 n_ref = normal * normal.dot(vel);
		//���������̔��˃x�N�g��
		Math::Vector3 h_ref = vel - n_ref;
		//�����������g���Ί���悤�ȓ����ɂȂ�
		*get_vel = h_ref;
	}

}//namespace Physics