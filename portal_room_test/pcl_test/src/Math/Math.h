#ifndef INCLUDED_MATH_MATH_H
#define INCLUDED_MATH_MATH_H

namespace Math {
	class Vector3;
}

namespace Math{

	//�~����
	static const float pi = 3.14159265358979323846f;
	//�p�x�̒P�ʕϊ��Ɏg��
	static const float to_deg = 180.f / pi;
	static const float to_rad = pi / 180.f;

	//���W�A������x�ɒ���
	float To_Deg(float rad);
	//�x���烉�W�A���ɒ���
	float To_Rad(float deg);

	//�x�����炤sin�֐�
	float Sin(float deg);
	//�x�����炤cos�֐�
	float Cos(float deg);
	//�x�����炤tan�֐�
	float Tan(float deg);
	//float�����炤asin�֐�
	float Asin(float a);
	//float�����炤acos�֐�
	float Acos(float a);
	//float�����炢�x��Ԃ�atan2�֐�
	float Atan2(float y, float x);
	//float�����炤sqrt�֐�
	float Sqrt(float a);
	//float�����炤fabs�֐�
	float Fabs(float a);
	//float�����炤pow�֐� n��p��
	float Pow(float n, float p);
	//int�����炤pow�֐� n��p��
	int Pow(int n, int p);
	//float�����炤exp�֐�
	float Exp(float x);

}//namespace Math

#endif