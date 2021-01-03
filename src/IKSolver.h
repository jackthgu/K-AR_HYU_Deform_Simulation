#ifndef IKSOLVE_H___324
#define IKSOLVE_H___324
#pragma once
#include "BaseLib/motion/IKSolver.h"

class Motion;
class vector3;
class intvectorn;
class quaterN;
class MotionLoader;
class Bone;

namespace MotionUtil
{

	// deprecated. use createFullbodyIk_LimbIK or IKSolveAnalytic instead.
	// LimbIK만을 지원한다.
	class IKSolver
	{
	public:
		IKSolver(){}
		virtual~ IKSolver(){}
		// setPose가 된 skeleton이 입력. exact IK. 다리 길이 늘이기가 아직 구현 안됨.
		virtual void limbIK(const MotionLoader& skeleton, int con, const vector3& input_goal, intvectorn& index, quaterN& delta_rot);
		// setPose가 된 skeleton이 입력. apporoximate IK.

		// approximation 1: hip에서 골까지의 거리가 쭉펴진거리의 lengthGoal.start()비율보다 멀면, IK의 importance가 줄어든다. lengthGoal.end()에서 importance가 0이된다.
		// approximation 2: ankle에서 골까지의 거리가 장딴지 길이의 distGoal.start()비율보다 멀면, IK의 importance가 줄어든다.
		virtual void limbIK_heu(const MotionLoader& skeleton, int con, const vector3& input_goal, intvectorn& index, quaterN& delta_rot, interval const& lengthGoal, interval const& distGoal);
		static bool isIKpossible(const MotionLoader& skeleton, int con, const vector3& input_goal, m_real lengthGoal=0.95, m_real distGoal=0.3);

		// 현재 다리가 펴져있는 정도를 계산한다. 1이 완전히 펴진상태를 뜻한다.
		static m_real calcIKlength(const MotionLoader& skeleton, int con);

		static m_real calcAnkleAngle(const MotionLoader& skeleton, int con);

		static void setLimb(int con, int& index_up, int& index_mid, int& index_low, vector3& axis, const MotionLoader& skeleton);
		// solve the shoulder joint angle with the axial constraints
		// or is the original shoulder joint angle
		// ref is a solution
		// axis is the direction vector from shoulder to the goal position for wrist
		inline static quater findNearest( const quater& ori, const quater& ref, const vector3& axis );
		inline static void angleRot(m_real& theta, vector3 const& v1, vector3 const& v2, quater* qk=NULL );

		inline static int limbIK( const vector3& goal, const vector3& sh, const vector3& v1, const vector3& v2,
			quater& qq1, quater& qq2, m_real ii , const vector3& axis =vector3(1,0,0), vector3* v3=NULL, vector3* v4=NULL);




	};
}
#endif
