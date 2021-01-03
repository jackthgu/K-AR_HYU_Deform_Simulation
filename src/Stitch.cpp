#include "stdafx.h"
#include "BaseLib/motion/Motion.h"
#include "./Stitch.h"
#include "BaseLib/motion/Retarget.h"
#include "BaseLib/motion/FootPrint.h"
#include "./Concat.h"
#include "BaseLib/motion/MotionUtil.h"
#include "./MotionUtilTwo.h"
#include "ClassificationLib/motion/TwoPosture.h"
#include "BaseLib/math/Operator.h"
#include "BaseLib/math/OperatorStitch.h"
#include "BaseLib/math/conversion.h"
#include "./OperatorSignal.h"
#include "FltkALL.h"
using namespace MotionUtil;

void quater_linstitch(m::stitchOp const& op, quaterN& c, quaterN const& a, quaterN const& b)
{
	quaterN aa(a.size()), bb(b.size());
	aa=a;
	bb=b;
	aa.align();
	bb.row(0).align(aa.row(aa.rows()-1));
	bb.align();
	c.setSize(aa.rows()+bb.rows()-1);

	op.calc(matView(c).lval(), matView(aa), matView(bb));

	for(int i=0; i<c.rows(); i++)
		c.row(i).normalize();	// renormalize
}

/*
void quaterNN_linstitch(m2::_op const& op, matrixn& c, matrixn & a, matrixn & b)
{
	int njoint=a.cols()/4;
	c.setSize(a.rows()+b.rows()-1, a.cols());
	for(int j=0; j<njoint; j++)
	{
		quaterNView aa(a.range(0, a.rows(), j*4, (j+1)*4).toQuaterN());
		quaterNView bb(b.range(0, b.rows(), j*4, (j+1)*4).toQuaterN());
		aa.align();
		bb.row(0).align(aa.row(aa.rows()-1));
		bb.align();
	}

	c.op2(op, a, b);

	for(int j=0; j<njoint; j++)
	{
		quaterNView cc(c.range(0, c.rows(), j*4, (j+1)*4).toQuaterN());
		for(int i=0; i<cc.rows(); i++)
			cc.row(i).normalize();
	}
}

void quaterNN_op1(m1::_op const& op, matrixn& c, matrixn &a)
{
	int njoint=a.cols()/4;
	c.setSize(a.rows(), a.cols());
	for(int j=0; j<njoint; j++)
	{
		// inpaint는 0,1, row-2, row-1만 사용하므로 이들만 align
		quaterNView aa(a.range(0, a.rows(), j*4, (j+1)*4).toQuaterN());
		aa.row(1).align(aa.row(0));
		aa.row(aa.rows()-2).align(aa.row(1));
		aa.row(aa.rows()-1).align(aa.row(aa.rows()-2));
	}

	c.op1(op, a);

	for(int j=0; j<njoint; j++)
	{
		quaterNView cc(c.range(0, c.rows(), j*4, (j+1)*4).toQuaterN());
		for(int i=0; i<cc.rows(); i++)
			cc.row(i).normalize();
	}

}
void MotionUtil::inpaint(int startInpaint, int endInpaint, Motion& front)
{
	MotionUtil::GetSignal frontGetSignal(front);
	MotionUtil::SetSignal frontSetSignal(front);

	int start=startInpaint-2;
	int end=endInpaint+2;

	matrixn trajA, traj;
	// stitch con pos
	frontGetSignal.constraintPositions(trajA, start, end);

	traj.op1(m1::inpaint(), trajA);

	frontSetSignal.constraintPositions(traj, start);

	if(dynamic_cast<TwoPosture*>(&front.pose(startInpaint-1)))
	{
		// inpaint opponent pos
		MotionUtilTwo::getOpponentPos(front, trajA, start, end);

		trajA.column(1).op0(v0::alignAngles(trajA[0][1]));
		traj.op1(m1::inpaint(),trajA);
		MotionUtilTwo::setOpponentPos(front, traj, start);
	}

	//stitch all the joints
	intvectorn ajoint;
	ajoint.colon(0,front.NumJoints());	
	frontGetSignal.jointAll(ajoint, trajA, start, end);
	quaterNN_op1(m1::inpaint0(), traj, trajA);
	frontSetSignal.jointAll(ajoint, traj, start);
}
*/
void MotionUtil::nostitch(Motion& front, Motion const& src, int srcfirst, int srclast)
{
	// 한프레임은 빼고 concat해야 stitch랑 타이밍이 맞는다. 이유는 stitch에서는 마지막 프레임이 첫 프레임과 같아지도록 하기 때문.
	srcfirst+=1;
	int lenOld=front.length();
	front.Concat(&src, srcfirst, srclast+1);
	front.ReconstructDataByDifference(lenOld);
}


// front.pose(frontEnd)=src.pose(srcStart)
// front.pose(frontEnd+1)=src.pose(srcStart+1)
// front.pose(frontEnd+2)=src.pose(srcStart+2)
// ... 
// front.pose(frontEnd+srcEnd-srcStart)=src.pose(srcEnd)
// 이런식이되도록 스티치 된다. 
//

void MotionUtil::stitchGlobal(int startSafe, int endSafe, Motion& front, Motion const& src, int srcStart, int srcEnd, int frontEnd, bool two)
{
	if(frontEnd>front.length())
		frontEnd=front.length();

	int addMount= srcEnd-srcStart+1;

	// linstitch의 경우 첫 두프레임은 바뀌지 않는다. 실제 safe보다 두프레임 앞을 startsafe로 놓는다.
	//startSafe=MAX(0, startSafe-2);
	//endSafe=MIN(frontEnd+srcEnd-srcStart, endSafe+2);

	// c1stitch의 경우 첫 한프레임은 바뀌지 않는다.
	startSafe=MAX(0, startSafe-1);
	endSafe=MIN(frontEnd+srcEnd-srcStart, endSafe+1);

	if(endSafe<frontEnd+3)
	{
		endSafe=frontEnd+3;
		Msg::verify(endSafe<frontEnd+srcEnd-srcStart,"stitchGlobal c");
	}

	front.changeLength(frontEnd+srcEnd-srcStart);

	int numJoint=front.NumJoints();
	for(int i=frontEnd+1; i<front.numFrames();i++)		
		front.pose(i).Clone(&src.pose(i-frontEnd+srcStart));

	// second stitch:
	int nForwardSafe=endSafe-frontEnd;

	ASSERT(addMount>=2);

	MotionUtil::GetSignal frontGetSignal(front);
	MotionUtil::SetSignal frontSetSignal(front);
	MotionUtil::GetSignal addGetSignal(src);

	//m2::linstitchPreprocess linstitcher(numFrameOld-startSafe, nForwardSafe);
	//m2::linstitchPreprocess linstitcher10(numFrameOld-startSafe, nForwardSafe, 10);
	m::c1stitchPreprocess linstitcher(frontEnd-startSafe+1, nForwardSafe);
	m::c1stitchPreprocess linstitcher10(frontEnd-startSafe+1, nForwardSafe,5);
	m::c1stitchPreprocess& linstitcherQ=linstitcher;
	//m2::linstitchPreprocessInc linstitcherQ(numFrameOld-startSafe, nForwardSafe, 5, 4);

	matrixn trajA, trajB, traj;
	{
		frontGetSignal.interframeDifference(trajA, startSafe, frontEnd+1);
		addGetSignal.interframeDifference(trajB, srcStart, nForwardSafe+srcStart);

		// linear delta terms (dv, dq): column 0~3 -> m2::c0stitch
		traj.setSize(trajA.rows()+nForwardSafe-1, trajA.cols());
		//matViewCol(traj, 0, 3).op2(m2::c0stitchPreserve2(), matViewCol(trajA, 0, 3), matViewCol(trajB, 0, 3));
		m::c0stitchPreserve2 tt1;
		tt1.calc(matViewCol(traj, 0, 3).lval(), matViewCol(trajA, 0, 3), matViewCol(trajB, 0, 3));

		// height: linstitch
		//traj.range(0, traj.rows(), 3, 4).op2(linstitcher, trajA.range(0, trajA.rows(), 3, 4), matViewCol(trajB, 3, 4));
		linstitcher.calc(traj.range(0, traj.rows(), 3, 4).lval(), trajA.range(0, trajA.rows(), 3, 4), matViewCol(trajB, 3, 4));

		// quaternion term: column 4~8 -> quaterN::c1stitch
		quaterNN_linstitch(linstitcherQ, matViewCol(traj, 4).lval(),matViewCol(trajA, 4).lval(), matViewCol(trajB, 4).lval());

		frontSetSignal.interframeDifference(traj, startSafe);
		front.ReconstructDataByDifference(startSafe-1);

		// stitch con pos
		//frontGetSignal.constraintPositions(trajA, startSafe, numFrameOld);
		//addGetSignal.constraintPositions(trajB, 0, add.numFrames());

		//traj.op2(m2::c0stitchPreserve2(), trajA, trajB);

		//frontSetSignal.constraintPositions(traj, startSafe);

	}
	vector3N vtrajA, vtrajB, vtraj;
	quaterN otrajA, otrajB, otraj;

	
	if(two && dynamic_cast<TwoPosture*>(&front.pose(0)))
	{
		// stitch opponent pos
		MotionUtilTwo::getOpponentPos(front, trajA, startSafe, frontEnd+1);
		MotionUtilTwo::getOpponentPos(src, trajB, srcStart, srcStart+nForwardSafe);

		//trajA.column(1).op0(v0::alignAngles(trajA[0][1]));
		v0::alignAngles a(trajA[trajA.rows()-1][1]);
		a.calc(trajA.column(1).lval());

		//traj.op2(linstitcher,trajA, trajB);
		linstitcher.calc(traj,trajA, trajB);
		MotionUtilTwo::setOpponentPos(front, traj, startSafe);
	}

	if(src.numTransJoints()>1)
	{
		intvectorn ajoint;
		ajoint.colon(1,src.numTransJoints());	
		frontGetSignal.transJointAll(ajoint, trajA, startSafe, frontEnd+1);
		addGetSignal.transJointAll(ajoint, trajB, srcStart, nForwardSafe+srcStart);
		traj.setSize(trajA.rows()+nForwardSafe-1, trajA.cols());
		
		// all DOF using linstitcher:
		//traj.op2(linstitcher, trajA, trajB);
		linstitcher.calc(traj,trajA, trajB);
		frontSetSignal.transJointAll(ajoint, traj, startSafe);
	}

	frontGetSignal.jointGlobalAll(MotionUtil::LOCAL_COORD, trajA, startSafe, frontEnd+1);
	addGetSignal.jointGlobalAll(MotionUtil::LOCAL_COORD, trajB, srcStart, srcStart+nForwardSafe);

	quaterNN_linstitch(linstitcherQ, traj, trajA, trajB);

	frontSetSignal.jointGlobalAll(MotionUtil::LOCAL_COORD,traj, startSafe);

}

void MotionUtil::stitchGlobalC2(int startSafe, int endSafe, Motion& front, Motion const& src, int srcStart, int srcEnd, int frontEnd, bool two)
{
	if(frontEnd>front.length())
		frontEnd=front.length();

	int addMount= srcEnd-srcStart+1;

	// linstitch의 경우 첫 두프레임은 바뀌지 않는다. 실제 safe보다 두프레임 앞을 startsafe로 놓는다.
	startSafe=MAX(0, startSafe-2);
	endSafe=MIN(frontEnd+srcEnd-srcStart, endSafe+2);

	// c1stitch의 경우 첫 한프레임은 바뀌지 않는다.
	//startSafe=MAX(0, startSafe-1);
	//endSafe=MIN(frontEnd+srcEnd-srcStart, endSafe+1);

	if(endSafe<frontEnd+3)
	{
		endSafe=frontEnd+3;
		Msg::verify(endSafe<frontEnd+srcEnd-srcStart,"stitchGlobal c");
	}

	front.changeLength(frontEnd+srcEnd-srcStart);

	int numJoint=front.NumJoints();
	for(int i=frontEnd+1; i<front.numFrames();i++)		
		front.pose(i).Clone(&src.pose(i-frontEnd+srcStart));

	// second stitch:
	int nForwardSafe=endSafe-frontEnd;

	ASSERT(addMount>=2);

	MotionUtil::GetSignal frontGetSignal(front);
	MotionUtil::SetSignal frontSetSignal(front);
	MotionUtil::GetSignal addGetSignal(src);

	m::linstitchPreprocess linstitcher(frontEnd-startSafe+1, nForwardSafe, 2, false);
	m::linstitchPreprocess linstitcher10(frontEnd-startSafe+1, nForwardSafe, 5, false);
	m::linstitchPreprocess& linstitcherQ=linstitcher10;
	//m2::linstitchPreprocessInc linstitcherQ(numFrameOld-startSafe, nForwardSafe, 5, 4);
	//m2::c1stitchPreprocess linstitcher(frontEnd-startSafe+1, nForwardSafe);
	//m2::c1stitchPreprocess linstitcher10(frontEnd-startSafe+1, nForwardSafe,5);
	//m2::c1stitchPreprocess& linstitcherQ=linstitcher;

	matrixn trajA, trajB, traj;
	{
		frontGetSignal.interframeDifference(trajA, startSafe, frontEnd+1);
		addGetSignal.interframeDifference(trajB, srcStart, nForwardSafe+srcStart);

		// linear delta terms (dv, dq): column 0~3 -> m2::c0stitch
		traj.setSize(trajA.rows()+nForwardSafe-1, trajA.cols());
	//	matViewCol(traj, 0, 3).op2(m2::c0stitchPreserve2(), matViewCol(trajA, 0, 3), matViewCol(trajB, 0, 3));
		m::c0stitchPreserve2 tt11;
			tt11.calc(matViewCol(traj, 0, 3).lval(), matViewCol(trajA, 0, 3), matViewCol(trajB, 0, 3));


		// height: linstitch
		//traj.range(0, traj.rows(), 3, 4).op2(linstitcher, trajA.range(0, trajA.rows(), 3, 4), matViewCol(trajB, 3, 4));
		linstitcher.calc(traj.range(0, traj.rows(), 3, 4).lval(), trajA.range(0, trajA.rows(), 3, 4), matViewCol(trajB, 3, 4));

		// quaternion term: column 4~8 -> quaterN::c1stitch
		quaterNN_linstitch(linstitcherQ, matViewCol(traj, 4).lval(),matViewCol(trajA, 4).lval(), matViewCol(trajB, 4).lval());

		frontSetSignal.interframeDifference(traj, startSafe);
		front.ReconstructDataByDifference(startSafe-1);

		// stitch con pos
		//frontGetSignal.constraintPositions(trajA, startSafe, numFrameOld);
		//addGetSignal.constraintPositions(trajB, 0, add.numFrames());

		//traj.op2(m2::c0stitchPreserve2(), trajA, trajB);

		//frontSetSignal.constraintPositions(traj, startSafe);

	}
	vector3N vtrajA, vtrajB, vtraj;
	quaterN otrajA, otrajB, otraj;

	
	if(two && dynamic_cast<TwoPosture*>(&front.pose(0)))
	{
		// stitch opponent pos
		MotionUtilTwo::getOpponentPos(front, trajA, startSafe, frontEnd+1);
		MotionUtilTwo::getOpponentPos(src, trajB, srcStart, srcStart+nForwardSafe);

		//trajA.column(1).op0(v0::alignAngles(trajA[0][1]));
		//trajB.column(1).op0(v0::alignAngles(trajA[trajA.rows()-1][1]));
		//traj.op2(linstitcher,trajA, trajB);
	
		v0::alignAngles a(trajA[0][1]);
		a.calc(trajA.column(1).lval());
		v0::alignAngles aa(trajB[trajA.rows()-1][1]);
		aa.calc(trajB.column(1).lval());
		linstitcher.calc(traj, trajA,trajB);
	
		MotionUtilTwo::setOpponentPos(front, traj, startSafe);
	}

	if(src.numTransJoints()>1)
	{
		intvectorn ajoint;
		ajoint.colon(1,src.numTransJoints());	
		frontGetSignal.transJointAll(ajoint, trajA, startSafe, frontEnd+1);
		addGetSignal.transJointAll(ajoint, trajB, srcStart, nForwardSafe+srcStart);
		traj.setSize(trajA.rows()+nForwardSafe-1, trajA.cols());
		
		// all DOF using linstitcher:
		//traj.op2(linstitcher, trajA, trajB);
		linstitcher.calc(traj, trajA,trajB);
		frontSetSignal.transJointAll(ajoint, traj, startSafe);
	}

	frontGetSignal.jointGlobalAll(MotionUtil::LOCAL_COORD, trajA, startSafe, frontEnd+1);
	addGetSignal.jointGlobalAll(MotionUtil::LOCAL_COORD, trajB, srcStart, srcStart+nForwardSafe);

	quaterNN_linstitch(linstitcherQ, traj, trajA, trajB);

	frontSetSignal.jointGlobalAll(MotionUtil::LOCAL_COORD,traj, startSafe);

}

void MotionUtil::stitchOnline(Motion& front, Motion const& add)
{
	int numFrameOld = front.numFrames();
	int addMount= add.numFrames();
	// linstitch의 경우 첫 두프레임은 바뀌지 않는다. 실제 safe보다 두프레임 앞을 startsafe로 놓는다.
	int startSafe=MAX(0, numFrameOld-2);

	front.Resize(front.numFrames()+addMount-1);	

	int numJoint=front.NumJoints();
	for(int i=numFrameOld; i<front.numFrames();i++)		
		front.pose(i).Clone(&add.pose(i-numFrameOld+1));
	
	int nForwardSafe=addMount;	// 입력으로 들어온 prevSafe, afterSafe는무시하고, 최대한 길게 잡고 간다.

	front.setDiscontinuity(numFrameOld, false);
	
	ASSERT(addMount>=2);

	MotionUtil::GetSignal frontGetSignal(front);
	MotionUtil::SetSignal frontSetSignal(front);
	MotionUtil::GetSignal addGetSignal(add);

	m::linstitchOnline linstitcherOnline(2.0);
	matrixn trajA, trajB, traj;	
	{
		frontGetSignal.interframeDifference(trajA, startSafe, numFrameOld);
		addGetSignal.interframeDifference(trajB, 0, nForwardSafe);

		// linear delta terms (dv, dq): column 0~3 -> m2::c0stitch
		traj.setSize(trajA.rows()+nForwardSafe-1, trajA.cols());
		//matViewCol(traj, 0, 3).op2(linstitcherOnline, matViewCol(trajA, 0, 3), matViewCol(trajB, 0, 3));
		linstitcherOnline.calc(matViewCol(traj, 0, 3).lval(), matViewCol(trajA, 0, 3), matViewCol(trajB, 0, 3));

		// height: linstitch
		//matViewCol(traj, 3, 4).op2(linstitcherOnline, matViewCol(trajA, 3, 4), matViewCol(trajB, 3, 4));
		linstitcherOnline.calc(matViewCol(traj, 3, 4).lval(), matViewCol(trajA, 3, 4), matViewCol(trajB, 3, 4));

 		// quaternion term: column 4~8 -> quaterN::c1stitch
		quaterNN_linstitch(linstitcherOnline, matViewCol(traj, 4, 8).lval(), 
			matViewCol(trajA, 4,8 ).lval(), matViewCol(trajB, 4,8).lval());

		frontSetSignal.interframeDifference(traj, startSafe);
		front.ReconstructDataByDifference(startSafe-1);

		
		// stitch con pos
		//frontGetSignal.constraintPositions(trajA, startSafe, numFrameOld);
		//addGetSignal.constraintPositions(trajB, 0, nForwardSafe);

		//traj.op2(linstitcherOnline, trajA, trajB);

		//frontSetSignal.constraintPositions(traj, startSafe);
	}
	vector3N vtrajA, vtrajB, vtraj;
	quaterN otrajA, otrajB, otraj;

	// stitch opponent pos
	MotionUtilTwo::getOpponentPos(front, trajA, startSafe, numFrameOld);
 	MotionUtilTwo::getOpponentPos(add, trajB, 0, nForwardSafe);

//	trajA.column(1).op0(v0::alignAngles(trajA[0][1]));
//	trajB.column(1).op0(v0::alignAngles(trajA[trajA.rows()-1][1]));
//	traj.op2(linstitcherOnline,trajA, trajB);

		v0::alignAngles aa1(trajA[0][1]);
		aa1.calc(trajA.column(1).lval());
		v0::alignAngles aa2(trajA[trajA.rows()-1][1]);
		aa2.calc(trajB.column(1).lval());
	
		linstitcherOnline.calc(traj,trajA, trajB);
		
	MotionUtilTwo::setOpponentPos(front, traj, startSafe);

	//stitch all the remaining joint

	// rotational joints
	intvectorn ajoint;
	ajoint.colon(1,add.NumJoints());	
	frontGetSignal.jointAll(ajoint, trajA, startSafe, numFrameOld);
	addGetSignal.jointAll(ajoint, trajB, 0, nForwardSafe);
	quaterNN_linstitch(linstitcherOnline, traj, trajA, trajB);
	frontSetSignal.jointAll(ajoint, traj, startSafe);

	// translational joints
	if(add.numTransJoints()>1)
	{
		intvectorn ajoint;
		ajoint.colon(1,add.numTransJoints());	
		frontGetSignal.transJointAll(ajoint, trajA, startSafe, numFrameOld);
		addGetSignal.transJointAll(ajoint, trajB, 0, nForwardSafe);
		traj.setSize(trajA.rows()+nForwardSafe-1, trajA.cols());
		//traj.op2(linstitcherOnline, trajA, trajB);
		
		linstitcherOnline.calc(traj,trajA, trajB);
		frontSetSignal.transJointAll(ajoint, traj, startSafe);
	}
}

void MotionUtil::stitchBlend(int startSafe, int endSafe, Motion& front, Motion const& add, int addStart, int addEnd, int frontEnd, int tail)
{
	// front
	//						frontEnd
	// -------------------------|----|
	//                            front.numFrames();

	// add				    ----|------------------|----...
	//						addstart	         addend
	
	//                startSafe
	// blend		     |-----------|

	// 여유분의 길이는 tail로 한다.
	// results (when tail==5)
	//  -----------------=============-------------|----|
	Msg::error("stitchBlend not implemented yet");
}

char footState(Posture const& pose)
{
	bool lcon=pose.constraint[CONSTRAINT_LEFT_TOE];
	bool rcon=pose.constraint[CONSTRAINT_RIGHT_TOE];

	if(lcon)
	{
		if(rcon)
			return 'D';
		else
			return 'L';
	}
	else
	{
		if(rcon)
			return 'R';
	}

	return 'F';
}

m_real footDist(Motion const& src, int srcf, Motion const& tgt, int tgtf, bool bCheckOne=true)
{
	TString aa("LL");
	TString bb("LL");
	
	aa[0]=footState(src.pose(srcf-1));
	aa[1]=footState(src.pose(srcf));
	bb[0]=footState(tgt.pose(tgtf-1));
	bb[1]=footState(tgt.pose(tgtf));


	if(aa==bb)	// 두프레임 연속으로 같은것을 선호한다.
	{
		if(aa[0]=='F' && aa[1]!='F')
			return 1.0;

		if(aa[1]=='F' && aa[0]!='F')
			return 10.0;

		if(aa[0]!=aa[1]);
			return 100.0;

		if(aa!="FF")	// FF 보단 차라리 L이 낫다.
			return 1000.0;
	}

	if(bCheckOne)
	{
		m_real d1=footDist(src, srcf-1, tgt, tgtf,false);
		m_real d2=footDist(src, srcf, tgt, tgtf-1,false);
		m_real d=MIN(d1, d2);
		if(d<10000.0)
			return d*2.0;
	}
	else return 10000.0;

	char a=aa[1];
	char b=bb[1];
	if(a==b)
	{
		if(a=='L' || a=='R')
			return 10000.0;

		if(a=='D')
			return 100000.0;

		if(aa==bb)
			return 1000000.0;	// FF
		if(a=='F')
			return 10000000.0;
	}

	return 10000000;
}

void MotionUtil::stitchTransition(int startSafe, int endSafe, Motion& front, int transitionFrom, Motion const& src, int srcStart, int srcEnd, int frontEnd)
{
	//                           
	//                                   frontEnd  frontLen=src.pose(transitionFrom)
	//  front...............................|........|
	//  src                                 |..................|
	//                                 srcStart              srcEnd
	//  searchRange               |                  
	//					      startSafe
	//									|               |
	//								startTransition   endTransition

	if(frontEnd>front.length())
		frontEnd=front.length();

	int frontLen=front.length();
	front.changeLength(frontEnd+srcEnd-srcStart);	

	int startTransition=(frontEnd+startSafe)/2;
	int endTransition=(frontEnd+endSafe)/2;
	int numJoint=front.NumJoints();

	
	for(int i=frontLen+1; i<=endTransition;i++)		
		//front.pose2(i).Clone(&src.pose2(i-frontLen+transitionFrom));
		pose2(front,i).Clone(&pose2(src,i-frontLen+transitionFrom));
	for(int i=endTransition+1; i<front.numFrames(); i++)
		//front.pose2(i).Clone(&src.pose2(i-frontEnd+srcStart));
		pose2(front,i).Clone(&pose2(src,i-frontEnd+srcStart));

	vectorn _transitionCosts(endTransition-startTransition+1);
	vectornView transitionCosts=vecViewOffset(_transitionCosts, startTransition);
	
	m_real maxDist=MAX(endTransition-frontEnd, frontEnd-startSafe);

	for(int i=startTransition; i<=endTransition; i++)
	{
		m_real dist=m_real(ABS(i-frontEnd))/maxDist;
		m_real overhead=(SQR(dist)+2.0)/2.0;	// 2 to 3.. 멀면 1.5배정도 cost부가
		transitionCosts[i]=overhead*footDist(front, i, src, i-frontEnd+srcStart);
	}

	int argMin=_transitionCosts.argMin()+startTransition;

	printf("argMin %d\n", argMin-frontEnd);
	
	if(0)
	{
		//test code
		for(int i=argMin; i<front.numFrames(); i++)
//			front.pose2(i).Clone(&src.pose2(i-frontEnd+srcStart));
		pose2(front,i).Clone(&pose2(src,i-frontEnd+srcStart));
		front.ReconstructDataByDifference(argMin-1);
	}
	else
		stitchGlobal(startSafe, endSafe, front, src, srcStart-(frontEnd-argMin), srcEnd, argMin);
}

void MotionUtil::stitchLocal(int prevSafe, int afterSafe, Motion& front, Motion const& add)
{
	int numFrameOld = front.numFrames();
	int addMount= add.numFrames();
	// linstitch의 경우 첫 두프레임은 바뀌지 않는다. 실제 safe보다 두프레임 앞을 startsafe로 놓는다.
	int startSafe=MAX(0, numFrameOld-prevSafe-2);

	front.Resize(front.numFrames()+addMount-1);	

	int numJoint=front.NumJoints();
	for(int i=numFrameOld; i<front.numFrames();i++)		
	//	front.pose2(i).Clone(&add.pose2(i-numFrameOld+1));
		pose2(front,i).Clone(&pose2(add,i-numFrameOld+1));
	// second stitch:
	int nForwardSafe=MIN(addMount, afterSafe+2);

//	front.discontinuity().clearAt(numFrameOld);
	front.setDiscontinuity(numFrameOld, false);


	ASSERT(addMount>=2);

	MotionUtil::GetSignal frontGetSignal(front);
	MotionUtil::SetSignal frontSetSignal(front);
	MotionUtil::GetSignal addGetSignal(add);

	//m2::linstitchPreprocess linstitcher(numFrameOld-startSafe, nForwardSafe);
	//m2::linstitchPreprocess linstitcher10(numFrameOld-startSafe, nForwardSafe, 10);
	m::c1stitchPreprocess linstitcher(numFrameOld-startSafe, nForwardSafe);
	m::c1stitchPreprocess linstitcher10(numFrameOld-startSafe, nForwardSafe,5);
	

	matrixn trajA, trajB, traj;
	{
		frontGetSignal.interframeDifference(trajA, startSafe, numFrameOld);
		addGetSignal.interframeDifference(trajB, 0, nForwardSafe);

		// linear delta terms (dv, dq): column 0~3 -> m2::c0stitch
		traj.setSize(trajA.rows()+nForwardSafe-1, trajA.cols());
		//matViewCol(traj, 0, 3).op2(m::c0stitchPreserve2(), matViewCol(trajA, 0, 3), matViewCol(trajB, 0, 3));
		m::c0stitchPreserve2 tat;
		tat.calc(matViewCol(traj, 0, 3).lval(), matViewCol(trajA, 0, 3), matViewCol(trajB, 0, 3));
		// height: linstitch
		//matViewCol(traj, 3, 4).op2(linstitcher, matViewCol(trajA, 3, 4), matViewCol(trajB, 3, 4));
		linstitcher.calc(matViewCol(traj, 3, 4).lval(), matViewCol(trajA, 3, 4), matViewCol(trajB, 3, 4));
		// quaternion term: column 4~8 -> quaterN::c1stitch
		quaterNN_linstitch(linstitcher, matViewCol(traj, 4).lval(),matViewCol(trajA, 4).lval(), matViewCol(trajB, 4).lval());

		frontSetSignal.interframeDifference(traj, startSafe);
		front.ReconstructDataByDifference(startSafe-1);

		// stitch con pos
		//frontGetSignal.constraintPositions(trajA, startSafe, numFrameOld);
		//addGetSignal.constraintPositions(trajB, 0, nForwardSafe);

		//// c0stich
		//traj.op2(m2::c0stitchPreserve2(), trajA, trajB);

		//frontSetSignal.constraintPositions(traj, startSafe);
	}
	vector3N vtrajA, vtrajB, vtraj;
	quaterN otrajA, otrajB, otraj;

	// stitch opponent pos
	MotionUtilTwo::getOpponentPos(front, trajA, startSafe, numFrameOld);
	MotionUtilTwo::getOpponentPos(add, trajB, 0, nForwardSafe);

//	trajA.column(1).op0(v0::alignAngles(trajA[0][1]));
//	trajB.column(1).op0(v0::alignAngles(trajA[trajA.rows()-1][1]));
//	traj.op2(linstitcher,trajA, trajB);
	
		v0::alignAngles aaa1(trajA[0][1]);
		aaa1.calc(trajA.column(1).lval());
		v0::alignAngles aaa2(trajA[trajA.rows()-1][1]);
		aaa2.calc(trajB.column(1).lval());

	linstitcher.calc(traj,trajA, trajB);

	MotionUtilTwo::setOpponentPos(front, traj, startSafe);

	//stitch all the remaining joint

	// rotational joints
	intvectorn ajoint;
	ajoint.colon(1,add.NumJoints());	
	frontGetSignal.jointAll(ajoint, trajA, startSafe, numFrameOld);
	addGetSignal.jointAll(ajoint, trajB, 0, nForwardSafe);
	quaterNN_linstitch(linstitcher, traj, trajA, trajB);
	//traj.op2(m2::stitchQuaterNN(&quaterN::c0stitch, 1), trajA, trajB);

	frontSetSignal.jointAll(ajoint, traj, startSafe);

	// translational joints
	if(add.numTransJoints()>1)
	{
		intvectorn ajoint;
		ajoint.colon(1,add.numTransJoints());	
		frontGetSignal.transJointAll(ajoint, trajA, startSafe, numFrameOld);
		addGetSignal.transJointAll(ajoint, trajB, 0, nForwardSafe);
		traj.setSize(trajA.rows()+nForwardSafe-1, trajA.cols());
		
		
		// all DOF using linstitcher:
	//	traj.op2(linstitcher, trajA, trajB);

	linstitcher.calc(traj,trajA, trajB);

		// y using c0 stitch, others using linstitcher:
		//matViewCol(traj, 0,1).op2(linstitcher, matViewCol(trajA, 0,1), matViewCol(trajB, 0,1));
		//matViewCol(traj, 1,2).op2(m2::c0stitch(), matViewCol(trajA, 1,2), matViewCol(trajB, 1,2));
		//matViewCol(traj, 2).op2(linstitcher, matViewCol(trajA, 2), matViewCol(trajB, 2));


		// all DOF using c0stitch
		//traj.op2(m2::c0stitchPreserve2(), trajA, trajB);
		frontSetSignal.transJointAll(ajoint, traj, startSafe);
	}
}

void MotionUtil::stitchGlobal(int prevSafe, int afterSafe, Motion& front, Motion const& add)
{
	int numFrameOld = front.numFrames();
	int addMount= add.numFrames();
	// linstitch의 경우 첫 두프레임은 바뀌지 않는다. 실제 safe보다 두프레임 앞을 startsafe로 놓는다.
	int startSafe=MAX(0, numFrameOld-prevSafe-2);

	front.Resize(front.numFrames()+addMount-1);	

	int numJoint=front.NumJoints();
	for(int i=numFrameOld; i<front.numFrames();i++)		
	//front.pose2(i).Clone(&add.pose2(i-numFrameOld+1));
	pose2(front,i).Clone(&pose2(add,i-numFrameOld+1));

	// second stitch:
	int nForwardSafe=MIN(addMount, afterSafe+2);

	front.setDiscontinuity(numFrameOld, false);
	//front.discontinuity().clearAt(numFrameOld);

	ASSERT(addMount>=2);

	MotionUtil::GetSignal frontGetSignal(front);
	MotionUtil::SetSignal frontSetSignal(front);
	MotionUtil::GetSignal addGetSignal(add);

//	m2::linstitchPreprocess linstitcher(numFrameOld-startSafe, nForwardSafe);
//	m2::linstitchPreprocess linstitcher10(numFrameOld-startSafe, nForwardSafe, 10);
	m::c1stitchPreprocess linstitcher(numFrameOld-startSafe, nForwardSafe);
	m::c1stitchPreprocess linstitcher10(numFrameOld-startSafe, nForwardSafe,5);

	matrixn trajA, trajB, traj;
	{
		frontGetSignal.interframeDifference(trajA, startSafe, numFrameOld);
		addGetSignal.interframeDifference(trajB, 0, nForwardSafe);

		// linear delta terms (dv, dq): column 0~3 -> m2::c0stitch
		traj.setSize(trajA.rows()+nForwardSafe-1, trajA.cols());
		//matViewCol(traj, 0, 3).op2(m2::c0stitchPreserve2(), matViewCol(trajA, 0, 3), matViewCol(trajB, 0, 3));
		m::c0stitchPreserve2 tat;
		tat.calc(matViewCol(traj, 0, 3).lval(), matViewCol(trajA, 0, 3), matViewCol(trajB, 0, 3));

		// height: linstitch
		//traj.range(0, traj.rows(), 3, 4).op2(linstitcher, trajA.range(0, trajA.rows(), 3, 4), matViewCol(trajB, 3, 4));

		linstitcher.calc(matViewCol(traj, 3, 4).lval(), matViewCol(trajA, 3, 4), matViewCol(trajB, 3, 4));
	 	// quaternion term: column 4~8 -> quaterN::c1stitch
		quaterNN_linstitch(linstitcher, matViewCol(traj, 4).lval(),matViewCol(trajA, 4).lval(), matViewCol(trajB, 4).lval());

		frontSetSignal.interframeDifference(traj, startSafe);
		front.ReconstructDataByDifference(startSafe-1);

		// stitch con pos
		//
		//frontGetSignal.constraintPositions(trajA, startSafe, numFrameOld);
		//addGetSignal.constraintPositions(trajB, 0, add.numFrames());

		//traj.op2(m2::c0stitchPreserve2(), trajA, trajB);

		//frontSetSignal.constraintPositions(traj, startSafe);

	}
	vector3N vtrajA, vtrajB, vtraj;
	quaterN otrajA, otrajB, otraj;

	// stitch opponent pos
	MotionUtilTwo::getOpponentPos(front, trajA, startSafe, numFrameOld);
	MotionUtilTwo::getOpponentPos(add, trajB, 0, nForwardSafe);

	//trajA.column(1).op0(v0::alignAngles(trajA[0][1]));
	//trajB.column(1).op0(v0::alignAngles(trajA[trajA.rows()-1][1]));
	//traj.op2(linstitcher,trajA, trajB);

		v0::alignAngles aaa1(trajA[0][1]);
		aaa1.calc(trajA.column(1).lval());
		v0::alignAngles aaa2(trajA[trajA.rows()-1][1]);
		aaa2.calc(trajB.column(1).lval());
		linstitcher.calc(traj,trajA, trajB);
	
	MotionUtilTwo::setOpponentPos(front, traj, startSafe);

	if(add.numTransJoints()>1)
	{
		intvectorn ajoint;
		ajoint.colon(1,add.numTransJoints());	
		frontGetSignal.transJointAll(ajoint, trajA, startSafe, numFrameOld);
		addGetSignal.transJointAll(ajoint, trajB, 0, nForwardSafe);
		traj.setSize(trajA.rows()+nForwardSafe-1, trajA.cols());
		
		// all DOF using linstitcher:
	//	traj.op2(linstitcher, trajA, trajB);
		
		linstitcher.calc(traj,trajA, trajB);
		frontSetSignal.transJointAll(ajoint, traj, startSafe);
	}

	frontGetSignal.jointGlobalAll(MotionUtil::LOCAL_COORD, trajA, startSafe, numFrameOld);
	addGetSignal.jointGlobalAll(MotionUtil::LOCAL_COORD, trajB, 0, nForwardSafe);

	quaterNN_linstitch(linstitcher, traj, trajA, trajB);

	frontSetSignal.jointGlobalAll(MotionUtil::LOCAL_COORD,traj, startSafe);
}


//////////////////////////////////////////////////////////
// 아래는 쓰지 말것.
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
void C1stitch::stitch(int safe, Motion& front, const Motion& add)
{
	int numFrameOld=front.numFrames();

	int startFrame=0;
	int endFrame=add.numFrames();
	int addMount= endFrame-startFrame;

	int nStitchSafe=safe;	
	int nForwardSafe=MIN(addMount, nStitchSafe);

	m_concator.concat(front, add);
	
	vector3N displacementMap;
	quaterN displacementMapQ;
	front.setDiscontinuity(numFrameOld, false);

	ASSERT(addMount>=2);
	ASSERT(numFrameOld>=2);
	
	// root trajectory stitching
	displacementMap.displacement(front.pose(numFrameOld-2).m_aTranslations[0], 
										front.pose(numFrameOld-1).m_aTranslations[0], 
										front.pose(numFrameOld).m_aTranslations[0],
										front.pose(numFrameOld+1).m_aTranslations[0], -1*nStitchSafe, nForwardSafe);

	for(int i=-1*nStitchSafe; i<nForwardSafe; i++)
		front.pose(numFrameOld+i).m_aTranslations[0]+=displacementMap[i+nStitchSafe];

	// joint orientations stitching
	for(int j=0; j< front.NumJoints(); j++)
	{
		displacementMapQ.displacement(front.pose(numFrameOld-2).m_aRotations[j], 
											front.pose(numFrameOld-1).m_aRotations[j], 
											front.pose(numFrameOld).m_aRotations[j],
											front.pose(numFrameOld+1).m_aRotations[j], -1*nStitchSafe, nForwardSafe);

		
		for(int i=-1*nStitchSafe; i<nForwardSafe; i++)
		{
			front.pose(numFrameOld+i).m_aRotations[j].leftMult(displacementMapQ[i+nStitchSafe]);
		}
	}
}

void C1stitch::stitchOnline(int safe, Motion& front, const Motion& add)
{
	int startFrame=0;
	int endFrame=add.numFrames();

	int numFrameOld = front.numFrames();
	front.Concat(&add, startFrame, endFrame);
	
	int addMount= endFrame-startFrame;
	
	//!< one way stitching by taesoo
	
	// root trajectory stitching
	front.ReconstructDataByDifference(numFrameOld-1);

//	front.UpdatePQ(numFrameOld-1, INT_MAX);
	ASSERT(addMount>=2);
	ASSERT(numFrameOld>=2);
	
	
	// root trajectory stitching
	vector3N displacementMap;
	displacementMap.displacementOnline(front.pose(numFrameOld-2).m_aTranslations[0], 
											front.pose(numFrameOld-1).m_aTranslations[0], 
											front.pose(numFrameOld).m_aTranslations[0],
											front.pose(numFrameOld+1).m_aTranslations[0], addMount);

    for(int i=0; i<addMount-1; i++)
	{
		front.pose(numFrameOld+i).m_aTranslations[0]+=displacementMap[i];
	}

	// joint orientations stitching
	quaterN displacementMapQ;
	for(int j=0; j< front.NumJoints(); j++)
	{
		displacementMapQ.displacementOnline(front.pose(numFrameOld-2).m_aRotations[j], 
											front.pose(numFrameOld-1).m_aRotations[j], 
											front.pose(numFrameOld).m_aRotations[j],
											front.pose(numFrameOld+1).m_aRotations[j], addMount);

		
		for(int i=0; i<addMount-1; i++)
		{
			front.pose(numFrameOld+i).m_aRotations[j].leftMult(displacementMapQ[i]);
		}
	}
}

void C0stitch ::stitchUtil(int safe, Motion& front, int numFrameOld)
{
	{
		int addMount= front.numFrames()-numFrameOld;

		int nStitchSafe=safe;	
		int nForwardSafe=MIN(addMount, nStitchSafe);

		front.setDiscontinuity(numFrameOld, false);

		ASSERT(addMount>=2);
		ASSERT(numFrameOld>=2);

		// root trajectory stitching
		MotionUtil::GetSignal frontGetSignal(front);
		MotionUtil::SetSignal frontSetSignal(front);
		vector3N rootTraj;
		quaterN rootOTraj;
		frontGetSignal.root(rootTraj, numFrameOld-nStitchSafe, numFrameOld+nForwardSafe);
		frontGetSignal.joint(0, rootOTraj,numFrameOld-nStitchSafe, numFrameOld+nForwardSafe);

		rootTraj.c0stitch(nStitchSafe);
		rootTraj.c1stitch(nStitchSafe);
		rootOTraj.c0stitch(nStitchSafe);
		//rootOTraj.c1stitch(nStitchSafe);
		//rootOTraj.decomposeStitch(nStitchSafe);	
		//rootOTraj.hermite(nStitchSafe);
		frontSetSignal.root(rootTraj, numFrameOld-nStitchSafe);
		frontSetSignal.joint(0, rootOTraj,numFrameOld-nStitchSafe);
	}

	{
		int addMount= front.numFrames()-numFrameOld;

		int nStitchSafe=safe;	
		int nForwardSafe=MIN(addMount, nStitchSafe);

		quaterN traj;

		MotionUtil::GetSignal frontGetSignal(front);
		MotionUtil::SetSignal frontSetSignal(front);

		// joint orientations stitching
		for(int j=1; j< front.NumJoints(); j++)
		{
			frontGetSignal.joint(j, traj,numFrameOld-nStitchSafe, numFrameOld+nForwardSafe);
			traj.c0stitch(nStitchSafe);
			frontSetSignal.joint(j, traj,numFrameOld-nStitchSafe);
		}
	}

}

void linstitch::stitchUtilRoot(int safe, Motion& front, int numFrameOld)
{
	if(safe<2)
	{
		printf("stitch impossible (stichUtilRoot)\n");
		return;
	}
	int addMount= front.numFrames()-numFrameOld;

	int nStitchSafe=safe;	
	int nForwardSafe=MIN(addMount, nStitchSafe);

	front.setDiscontinuity(numFrameOld, false);

	ASSERT(addMount>=2);
	ASSERT(numFrameOld>=2);

	// root trajectory stitching
	MotionUtil::GetSignal frontGetSignal(front);
	MotionUtil::SetSignal frontSetSignal(front);
	vector3N rootTraj;
	quaterN rootOTraj;
	frontGetSignal.root(rootTraj, numFrameOld-nStitchSafe, numFrameOld+nForwardSafe);
	frontGetSignal.joint(0, rootOTraj,numFrameOld-nStitchSafe, numFrameOld+nForwardSafe);

	rootTraj.linstitch(nStitchSafe);
	rootOTraj.linstitch(nStitchSafe);

	frontSetSignal.root(rootTraj, numFrameOld-nStitchSafe);
	frontSetSignal.joint(0, rootOTraj,numFrameOld-nStitchSafe);
}
	
void linstitch::stitchUtilJoint(int safe, Motion& front, int numFrameOld)
{
	int addMount= front.numFrames()-numFrameOld;

	int nStitchSafe=safe;	
	int nForwardSafe=MIN(addMount, nStitchSafe);

	MotionUtil::GetSignal frontGetSignal(front);
	MotionUtil::SetSignal frontSetSignal(front);
	quaterN OTraj;
	for(int j=1; j<front.NumJoints(); j++)
	{
		frontGetSignal.joint(j, OTraj,numFrameOld-nStitchSafe, numFrameOld+nForwardSafe);
		OTraj.linstitch(nStitchSafe);
		frontSetSignal.joint(j, OTraj,numFrameOld-nStitchSafe);
	}
}

	
void C1stitch2::stitchUtilRoot(int safe, Motion& front, int numFrameOld)
{
	if(safe<2)
	{
		printf("stitch impossible (stichUtilRoot)\n");
		return;
	}
	int addMount= front.numFrames()-numFrameOld;

	int nStitchSafe=safe;	
	int nForwardSafe=MIN(addMount, nStitchSafe);

	front.setDiscontinuity(numFrameOld, false);

	ASSERT(addMount>=2);
	ASSERT(numFrameOld>=2);

	// root trajectory stitching
	MotionUtil::GetSignal frontGetSignal(front);
	MotionUtil::SetSignal frontSetSignal(front);
	vector3N rootTraj;
	quaterN rootOTraj;
	frontGetSignal.root(rootTraj, numFrameOld-nStitchSafe, numFrameOld+nForwardSafe);
	frontGetSignal.joint(0, rootOTraj,numFrameOld-nStitchSafe, numFrameOld+nForwardSafe);

	rootTraj.c0stitch(nStitchSafe);
	rootTraj.c1stitch(nStitchSafe);
	rootOTraj.c0stitch(nStitchSafe);
	rootOTraj.c1stitch(nStitchSafe);
	//rootOTraj.decomposeStitch(nStitchSafe);	
	//rootOTraj.hermite(nStitchSafe);

	frontSetSignal.root(rootTraj, numFrameOld-nStitchSafe);
	frontSetSignal.joint(0, rootOTraj,numFrameOld-nStitchSafe);
}

void C1stitch2::stitchUtilJoint(int safe, Motion& front, int numFrameOld)
{
	int addMount= front.numFrames()-numFrameOld;

	int nStitchSafe=safe;	
	int nForwardSafe=MIN(addMount, nStitchSafe);

//#define OLD_METHOD
#ifdef OLD_METHOD
	quaterN displacementMapQ;

	// joint orientations stitching
	for(int j=1; j< front.NumJoints(); j++)
	{
		displacementMapQ.displacement(front.pose(numFrameOld-2).m_aRotations[j], 
			front.pose(numFrameOld-1).m_aRotations[j], 
			front.pose(numFrameOld).m_aRotations[j],
			front.pose(numFrameOld+1).m_aRotations[j], -1*nStitchSafe, nForwardSafe);

		for(int i=-1*nStitchSafe; i<nForwardSafe; i++)
			front.pose(numFrameOld+i).m_aRotations[j].leftMult(displacementMapQ[i+nStitchSafe]);
	}
#else
	MotionUtil::GetSignal frontGetSignal(front);
	MotionUtil::SetSignal frontSetSignal(front);
	quaterN OTraj;
	for(int j=1; j<front.NumJoints(); j++)
	{
		frontGetSignal.joint(j, OTraj,numFrameOld-nStitchSafe, numFrameOld+nForwardSafe);
#ifdef USE_LINSTITCH
		OTraj.linstitch(nStitchSafe);
#else
		OTraj.c0stitch(nStitchSafe);
		OTraj.c1stitch(nStitchSafe);
#endif
		frontSetSignal.joint(j, OTraj,numFrameOld-nStitchSafe);
	}
#endif
}

void C1stitch2::stitchUtil(int safe, Motion& front, int numFrameOld)
{
	stitchUtilRoot(safe, front, numFrameOld);
	stitchUtilJoint(safe, front, numFrameOld);
}

void C1stitch2::stitch(int safe, Motion& front, const Motion& add)
{
	int numFrameOld=front.numFrames();

	m_concator.concat(front, add);

	stitchUtil(safe, front, numFrameOld);

}


void C1stitchReconstruct ::stitch(int safe, Motion& front, const Motion& add)
{
	int numFrameOld=front.numFrames();

	int startFrame=0;
	int endFrame=add.numFrames();
	int addMount= endFrame-startFrame;

	int nStitchSafe=safe;	
	int nForwardSafe=MIN(addMount, nStitchSafe);

	front.Concat(&add, startFrame, endFrame);

	//!< two way stitching by taesoo
	
	// root trajectory stitching
//	ReconstructDataByDifference(numFrameOld-1);
	front.setDiscontinuity(numFrameOld, false);

//	front.UpdatePQ(numFrameOld-1, INT_MAX);
	ASSERT(addMount>=2);
	ASSERT(numFrameOld>=2);
	

	// root trajectory stitching
	vector3N displacementMap;
	displacementMap.displacement(front.pose(numFrameOld-2).m_dv, 
											front.pose(numFrameOld-1).m_dv, 
											front.pose(numFrameOld).m_dv,
											front.pose(numFrameOld+1).m_dv, -1*nStitchSafe, nForwardSafe);

    for(int i=-1*nStitchSafe; i<nForwardSafe; i++)
		front.pose(numFrameOld+i).m_dv+=displacementMap[i+nStitchSafe];

	displacementMap.displacement(vector3(0,front.pose(numFrameOld-2).m_offset_y, 0), 
									vector3(0,front.pose(numFrameOld-1).m_offset_y, 0), 
									vector3(0,front.pose(numFrameOld).m_offset_y, 0),
									vector3(0,front.pose(numFrameOld+1).m_offset_y, 0), -1*nStitchSafe, nForwardSafe);

    for(int i=-1*nStitchSafe; i<nForwardSafe; i++)
		front.pose(numFrameOld+i).m_offset_y+=displacementMap[i+nStitchSafe].y;

	quaterN displacementMapQ;

	displacementMapQ.displacement(front.pose(numFrameOld-2).m_dq, 
										front.pose(numFrameOld-1).m_dq, 
										front.pose(numFrameOld).m_dq,
										front.pose(numFrameOld+1).m_dq, -1*nStitchSafe, nForwardSafe);

		
	for(int i=-1*nStitchSafe; i<nForwardSafe; i++)
		front.pose(numFrameOld+i).m_dq.leftMult(displacementMapQ[i+nStitchSafe]);


	displacementMapQ.displacement(front.pose(numFrameOld-2).m_offset_q, 
										front.pose(numFrameOld-1).m_offset_q, 
										front.pose(numFrameOld).m_offset_q,
										front.pose(numFrameOld+1).m_offset_q, -1*nStitchSafe, nForwardSafe);
		
	for(int i=-1*nStitchSafe; i<nForwardSafe; i++)
		front.pose(numFrameOld+i).m_offset_q.leftMult(displacementMapQ[i+nStitchSafe]);

	front.ReconstructDataByDifference(numFrameOld-nStitchSafe-1);

	// joint orientations stitching
	for(int j=1; j< front.NumJoints(); j++)
	{
		displacementMapQ.displacement(front.pose(numFrameOld-2).m_aRotations[j], 
											front.pose(numFrameOld-1).m_aRotations[j], 
											front.pose(numFrameOld).m_aRotations[j],
											front.pose(numFrameOld+1).m_aRotations[j], -1*nStitchSafe, nForwardSafe);

		
		for(int i=-1*nStitchSafe; i<nForwardSafe; i++)
		{
			front.pose(numFrameOld+i).m_aRotations[j].leftMult(displacementMapQ[i+nStitchSafe]);
		}
	}
}

void StitchRetarget::stitch(int nPrevSafe, Motion& front, const Motion& add)
{
	int numFrameOld = front.numFrames();
	int addMount= add.numFrames();
	int startSafe=numFrameOld-nPrevSafe;
	int nCurrSafe=addMount/2;// -> 이게 다음번 stitch때 prevSafe로 입력되서 들어온다.

	float angle=0;
	vector3 axis(0,1,0);
	for(int i=0; i<add.numFrames(); i++)
		angle+=add.pose(i).m_dq.rotationAngle(axis);

  	
	// first concat
	MotionUtil::ReconstructConcat a;
		
//	MotionUtil::CalcFootPrintOnline footprint;
	MotionUtil::GetFootPrintOnline footprint;
	a.concat(front, add);	
	
	
	// stitch and then retarget
	MotionUtil::C1stitch2::stitchUtil(nPrevSafe, front, numFrameOld);

	// constraint retargetting
	intvectorn aConstraint(2);
	aConstraint[0]=CONSTRAINT_LEFT_TOE;
	aConstraint[1]=CONSTRAINT_RIGHT_TOE;

	CTArray<matrixn> conPos;
	CTArray<intvectorn> interval;

	conPos.Init(2);
	interval.Init(2);
	
	MotionUtil::Retarget rt(front);

	for(int i=0; i<aConstraint.size(); i++)
		footprint.getFootPrints(front, startSafe, front.numFrames(), aConstraint[i], interval[i], conPos[i]);

	for(int i=0; i<aConstraint.size(); i++)
		rt.retarget(startSafe, startSafe, front.numFrames()-nCurrSafe, front.numFrames()+10, aConstraint[i], conPos[i], interval[i]);

}



void NoStitch::stitch(int safe, Motion& front, const Motion& add)	{ m_concator.concat(front, add);}


