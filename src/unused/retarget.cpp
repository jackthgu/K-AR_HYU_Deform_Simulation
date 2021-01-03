#include "stdafx.h"
#include "BaseLib/motion/FootPrint.h"
#include "BaseLib/motion/Retarget.h"
#include "BaseLib/motion/IKSolver.h"
#include "BaseLib/math/OperatorStitch.h"
#include "BaseLib/math/Operator.h"
#include "BaseLib/math/nr/nr.h"
#include "BaseLib/math/Operator.h"
#include "BaseLib/math/conversion.h"
#include "BaseLib/math/intervals.h"
#include "retarget.h"
#include "BaseLib/motion/IKSolver.h"
#include "BaseLib/motion/Motion.h"
#include "BaseLib/motion/MotionLoader.h"
#include "MainLib/WrapperLua/LUAwrapper.h"
#include "./IKSolver.h"
//#include "WrapperLua/OR_LUA_Stack.h"
using namespace MotionUtil;


void m0::inpaint::calc(matrixn& c) const 
{
	int nsample=c.rows();
	vectorn weight(nsample-2);

	m_real middle=m_real(nsample-3)/2.0;
	for(int i=0; i<nsample-2; i++)
	{
		m_real w=1.0-ABS((m_real(i)-middle)/middle);
		w=w+1.0/10.0;
		weight[i]=w;
	}
	_inpaint(weight, c);
}

void m0::inpaint0quat::calc(matrixn& c) const 
{
	int nsample=c.rows();

	cforward.setSameSize(c);
	cbackward.setSameSize(c);

	int njoint=c.cols()/4;
	c.setSize(c.rows(), c.cols());
	for(int j=0; j<njoint; j++)
	{
		// inpaint는 0,1, row-2, row-1만 사용하므로 이들만 align
		quaterNView aa(c.range(0, c.rows(), j*4, (j+1)*4).toQuaterN());
		quaterNView bb(cforward.range(0, c.rows(), j*4, (j+1)*4).toQuaterN());
		quaterNView cc(cbackward.range(0, c.rows(), j*4, (j+1)*4).toQuaterN());
	
		bb.row(0)=aa.row(0);
		bb.row(1)=aa.row(1);
		cc.row(0).identity();
		cc.row(1).identity();

		cc.row(aa.rows()-2)=aa.row(aa.rows()-2);
		cc.row(aa.rows()-1)=aa.row(aa.rows()-1);
		bb.row(aa.rows()-2).identity();
		bb.row(aa.rows()-1).identity();
	}

	vectorn w1, w2;
	m_real strength;

	strength=MIN(10.0, nsample);		// 간격이 짧으면, 부드럽게 이어붙인다.
	w1.linspace(1.0/strength, 1.0, nsample-2);
	w2.linspace(1.0, 1.0/strength, nsample-2);

	 m0::inpaint::_inpaint(w1,cforward);
	 m0::inpaint::_inpaint(w2,cbackward);

	for(int i=0; i<c.rows(); i++)
	{
		c.row(i).each2(s2::AVG, cforward.row(i), cbackward.row(i));
	}
}


void m0::inpaint::_inpaint(vectorn const & weight, matrixn& c)
{
	/* LAGRANGIAN MULTIPLIER version*/
	//    Augmented* x= d , w=(X, lamda)
	//   (H A^T) (X    )= (d)
	//   (A 0  ) (lamda)
	int nsample=c.rows();

	int numCon=4;
	matrixn Augmented(c.rows()+numCon, c.rows()+numCon);
	matrixnView H=Augmented.range(0,c.rows(), 0, c.rows());
	matrixnView A=Augmented.range(c.rows(), c.rows()+numCon, 0, c.rows());
	matrixnView At=Augmented.range(0, c.rows(), c.rows(), c.rows()+numCon);

	vectorn x, d(c.rows()+numCon);
	// set hessian matrix. ( sum_0^{n-3} {(x[i]-2*x[i+1]+x[i+2]-cc[i])^2} 의 hessian(=gradient의 계수).
	HessianQuadratic h(c.rows());
	intvectorn index;
	vectorn coef;

	for(int i=0; i<nsample-2; i++)
	{
		index.setValues(3, i, i+1, i+2);
		coef.setValues(4, 1.0, -2.0, 1.0, 0.0);		
		coef*=weight[i];
		h.addSquared(index, coef);
	}

	H=h.H;

	// set constraint matrix (constraint Ax=b)
	A.setAllValue(0.0);
	A[0][0]=1.0;
	A[1][1]=1.0;
	A[2][c.rows()-2]=1.0;
	A[3][c.rows()-1]=1.0;

	At.transpose(A);

	Augmented.range(c.rows(), c.rows()+numCon, c.rows(), c.rows()+numCon).setAllValue(0.0);
#define USE_LUDCMP
#ifdef USE_LUDCMP
	DP p;
	matrixn LU(Augmented);
	Vec_INT indx(Augmented.rows());
	NR::ludcmp(LU,indx,p);
#else
	matrixn invAugmented;
	invAugmented.inverse(Augmented);
#endif

	for(int dim=0; dim<c.cols(); dim++)
	{
#define f(xxx) c(xxx,dim)
		d.range(0, c.rows())=h.R;

		// set constraint
		d[c.rows()]=f(0);
		d[c.rows()+1]=f(1);
		d[c.rows()+2]=f(c.rows()-2);
		d[c.rows()+3]=f(c.rows()-1);

#ifdef USE_LUDCMP
		x=d;
		NR::lubksb(LU,indx,x);
#else
		x.multmat(invAugmented, d);
#endif
		// save results.
		for(int i=0; i<c.rows(); i++)
			f(i)=x[i];
	}
}



void quaterNN_op0(m0::_op const& op, matrixn& c)
{
	int njoint=c.cols()/4;
	c.setSize(c.rows(), c.cols());
	for(int j=0; j<njoint; j++)
	{
		// inpaint는 0,1, row-2, row-1만 사용하므로 이들만 align
		quaterNView aa(c.range(0, c.rows(), j*4, (j+1)*4).toQuaterN());
		aa.row(1).align(aa.row(0));
		aa.row(aa.rows()-2).align(aa.row(1));
		aa.row(aa.rows()-1).align(aa.row(aa.rows()-2));
	}

	op.calc(c);

	for(int j=0; j<njoint; j++)
	{
		quaterNView cc(c.range(0, c.rows(), j*4, (j+1)*4).toQuaterN());
		for(int i=0; i<cc.rows(); i++)
			cc.row(i).normalize();
	}
}


RetargetSimple::RetargetSimple()
{
}
RetargetSimple::~RetargetSimple()
{
}

quater RetargetSimple::quater_transition0(const quater& aa, const quater& bb, int i, int duration)
{
	quater a(aa);
	quater b(bb);
	quater qid;
	qid.identity();
	a.align(qid);
	b.align(qid);
	// kovar paper (prefer identity quaternion (more safe))

	float totalTime=duration+1;
	float currTime;
	quater c, d, qi,out;
	qi.identity();
	
	currTime=(float)(i+1)/totalTime;
	float t=-2.f*CUBIC(currTime)+3.f*SQR(currTime);
	c.slerp(a, qi, t);
	d.slerp(qi, b, t);
	out.slerp(c, d, currTime);
	return out;
}

quater RetargetSimple::quater_transition(const quater& aa, const quater& b, int i, int duration)
{
	quater out;
	quater a(aa);
	a.align(b);

	float totalTime=duration+1;
	float currTime;

	currTime=(float)(i+1)/totalTime;
	float t=-2.f*CUBIC(currTime)+3.f*SQR(currTime);
	out.slerp(a, b, t);
	return out;
}

quater RetargetSimple::quater_transition_auto(const quater& a, const quater& b, int i, int duration)
{
	if(duration>6)
		return quater_transition0(a,b, i, duration);
	else
		return quater_transition(a, b, i, duration);
}

void RetargetSimple::fillGap(Motion& m_mot, int start, int end, intvectorn const& index, 
							  quaterN const& prevDelta_rot, quaterN const& delta_rot)
{
	int duration=end-start;

	// prevDelta_rot: IK result at start-1.
	// delta_rot: IK result at end
	end=MIN(end, m_mot.numFrames());

	for(int j=1; j<4; j++)
	{
		for(int iframe=start; iframe<end; iframe++)
		{
			quater displacement;
			displacement=quater_transition_auto(prevDelta_rot[j], delta_rot[j], iframe-start, duration);
			m_mot.pose(iframe).m_aRotations[index[j]].leftMult(displacement);
		}
	}
}


void RetargetSimple::retarget(Motion& m_mot, int startSafe, int startFrame, int endFrame, int endSafe, int con, const matrixn& footPrints, const intvectorn& interval)
{

	IKSolver ik;

	prevDelta_rot.setSize(4);
	prevDelta_rot[0].identity();
	prevDelta_rot[1].identity();
	prevDelta_rot[2].identity();
	prevDelta_rot[3].identity();

	intIntervals intervals;
	intervals.decodeFromVector(interval);

	int numFootPrint=intervals.size();

	bool bContinuedCon=false;
	for(int footStep=0; footStep<intervals.size(); footStep++)
	{
		if(intervals.start(footStep)>endFrame)
		{
			numFootPrint=footStep;
			break;
		}
		else if(intervals.end(footStep)>endFrame)
		{
			intervals.end(footStep)=endFrame;
			bContinuedCon=true;
		}
	}

	for(int footStep=0; footStep<numFootPrint; footStep++)
	{
		int left=intervals.start(footStep);
		int right=intervals.end(footStep);

		m_mot.skeleton().setPose(m_mot.pose(left));
		ik.limbIK(m_mot.skeleton(), con, footPrints.row(footStep).toVector3(), index, delta_rot);

		m_mot.pose(left).m_aRotations[index[1]].leftMult(delta_rot[1]);
		m_mot.pose(left).m_aRotations[index[2]].leftMult(delta_rot[2]);
		m_mot.pose(left).m_aRotations[index[3]].leftMult(delta_rot[3]);
		m_mot.pose(left).conPosition(con)=footPrints.row(footStep).toVector3();

		// fill gap "before" or "inbetween" footsteps
		int start=(footStep==0)?startSafe:intervals.end(footStep-1);
		int end=left;

		fillGap(m_mot, start, end, index, prevDelta_rot, delta_rot);

		if(right-left==1)
			prevDelta_rot=delta_rot;			
		// IK
		for(int i=left+1; i<right; i++)
		{
			m_mot.skeleton().setPose(m_mot.pose(i));

			ik.limbIK(m_mot.skeleton(), con, footPrints.row(footStep).toVector3(), index, prevDelta_rot);

			m_mot.pose(i).m_aRotations[index[1]].leftMult(prevDelta_rot[1]);
			m_mot.pose(i).m_aRotations[index[2]].leftMult(prevDelta_rot[2]);
			m_mot.pose(i).m_aRotations[index[3]].leftMult(prevDelta_rot[3]);
			m_mot.pose(i).conPosition(con)=footPrints.row(footStep).toVector3();
		}
	}

	if(numFootPrint)
	{
		if(!bContinuedCon)
		{
			// fill gap after.
			int start=intervals.end(numFootPrint-1);
			int end=endSafe;

			for(int j=1; j<4; j++)
			{
				if(end-start-1>0)
				{
					delta_rot[j].identity();
				}
			}

			fillGap(m_mot, start, end, index, prevDelta_rot, delta_rot);
		}
	}
}



RetargetOnline::RetargetOnline(Motion const& mot)
{
	LUAwrapper L;
	//L.setVal<TString>("motionId", mot.GetIdentifier());
	L.setString("motionId", mot.GetIdentifier());
	L.dofile("../Resource/constraintForSyn.lua");

	vector3 offset;
	mot.skeleton().getBoneByVoca(MotionLoader::LEFTKNEE).getOffset(offset);
	shin_length=offset.length();

	L.get_double("toe_height_thr",(double&) toe_height_thr);
	L.get_double("toe_speed_thr",toe_speed_thr);
	L.get_double("heel_height_thr",heel_height_thr);
	L.get_double("heel_speed_thr",heel_speed_thr);
	L.get_double("desired_min_height", desired_min_height);
	L.get_double("desired_max_height", desired_max_height);
	L.get_double("howToChooseConstraint",(double&) eHowToChooseConstraint);
	L.get_double("length_thr", mLengthThr);
	L.get_double("dist_thr", mDistThr);
	L.get_double("cleanup",(double&) bCleanup);

}

// startSafe부터 nextPlayEnd까지는 정확히 constriant를 만족시킨다. startSafe앞쪽으로는 건드리지 않고,
// nextPlayEnd이후로는 error를 propagate하는데 쓸수 있다.
void RetargetOnline::retarget(Motion& source, int startSafe, int nextPlayEnd)
{
	// constraint retargetting
	intvectorn aConstraint(2);
	aConstraint[0]=CONSTRAINT_LEFT_TOE;
	aConstraint[1]=CONSTRAINT_RIGHT_TOE;

	CTArray<matrixn> conPos;
	CTArray<intvectorn> interval;

	conPos.Init(2);
	interval.Init(2);

	MotionUtil::GetFootPrintOnline footprint(desired_min_height, desired_max_height, mLengthThr, mDistThr);

	for(int i=0; i<aConstraint.size(); i++)
		footprint.getFootPrints(source, startSafe, source.numFrames(), aConstraint[i], interval[i], conPos[i]);

	for(int i=0; i<aConstraint.size(); i++)
		retargetMethod.retarget(source, startSafe, startSafe, nextPlayEnd, source.numFrames(), aConstraint[i], conPos[i], interval[i]);
}

