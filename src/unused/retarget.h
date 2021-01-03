#ifndef RETARGET_H__
#define RETARGET_H__
#pragma once
#include "BaseLib/motion/Retarget.h"
namespace m0
{
	struct inpaint: public _op
	{
		inpaint(){}
		virtual void calc(matrixn& c) const ;
		static void _inpaint(vectorn const & w, matrixn& c);
	};

	struct inpaint0quat: public _op
	{
		mutable matrixn cforward, cbackward;// temporary
		inpaint0quat(){}
		virtual void calc(matrixn& c) const ;
	};
}

namespace MotionUtil
{

	class RetargetSimple
	{
		static quater quater_transition0(const quater& aa, const quater& bb, int i, int duration);
		static quater quater_transition(const quater& aa, const quater& b, int i, int duration);
		static quater quater_transition_auto(const quater& a, const quater& b, int i, int duration);

		// all temporary variables 
		intvectorn index;
		quaterN prevDelta_rot;
		quaterN delta_rot;

	public:
		RetargetSimple();
		virtual ~RetargetSimple();

		// start: gap start, end: gap end
		static void fillGap(Motion& mot, int start, int end, intvectorn const& index, 
										quaterN const& prevDelta_rot, quaterN const& delta_rot);
		// 동작의 [startSafe, endSafe)를 조정해서 [startFrame, endFrame) 사이에서 정확하게 constraint가 만족되도록 한다.
		virtual void retarget(Motion& mot, int startSafe, int startFrame, int endFrame, int endSafe, int con, const matrixn& footPrints, const intvectorn& interval);
	};

	
	// 처음에 한번만 생성해 놓아야 한다. 매 타임스텝 생성하면 매우 느림. (config를 constraint.lua로 부터 로딩하기 때문.)
	class RetargetOnline
	{
		m_real toe_height_thr;
		m_real toe_speed_thr;
		m_real heel_height_thr;
		m_real heel_speed_thr;
		m_real shin_length;
		m_real desired_min_height;
		m_real desired_max_height;
		m_real mLengthThr;
		m_real mDistThr;
		int eHowToChooseConstraint;
		int bCleanup;

		RetargetSimple retargetMethod;
	public:
		// mot은 예제 모션이다. 
		RetargetOnline(Motion const& mot);

		// source는 생성된 모션. 
		// startSafe부터 nextPlayEnd까지는 정확히 constriant를 만족시킨다. startSafe앞쪽으로는 건드리지 않고,
		// nextPlayEnd이후로는 error를 propagate하는데 쓴다. (constriant는 정확히 retarget하지 않는다. 나중에 어차피 바로잡을꺼니까.)
		void retarget(Motion& source, int startSafe, int nextPlayEnd);
	};

	

}
#endif
