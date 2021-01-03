#include "stdafx.h"
#include "BaseLib/motion/Motion.h"
#include "BaseLib/motion/MotionLoader.h"
#include "BaseLib/image/ImageProcessor.h"
#include "./MotionUtilTwo.h"
#include "BaseLib/math/Operator.h"
#include "./OperatorSignal.h"
#include "BaseLib/math/conversion.h"
#include "FltkALL.h"
void MotionUtilTwo::getOpponentPos(Motion const& m_Motion, vector3N& out, int start, int end)
{
	if(end>m_Motion.numFrames()) 
		end=m_Motion.numFrames();

	out.setSize(end-start);
	for(int i=start; i<end; i++)
		out[i-start]=pose2(m_Motion,i).m_oppenentPos;
}

void MotionUtilTwo::setOpponentPos(Motion& m_Motion, const vector3N& in, int start)
{
	for(int i=0; i<in.rows(); i++)
		pose2(m_Motion,i+start).m_oppenentPos=in[i];
}


void MotionUtilTwo::getOpponentPos(Motion const& m_Motion, matrixn& out, int start, int end)
{
	if(end>m_Motion.numFrames()) 
		end=m_Motion.numFrames();

	out.setSize(end-start, 2);

	quater q;
	vector3 v;
	vector3 front(0,0,1);
	for(int i=start; i<end; i++)
	{
		out[i-start][0]=pose2(m_Motion,i).m_oppenentPos.length();
		v.divide(pose2(m_Motion,i).m_oppenentPos, out[i-start][0]);
		q.axisToAxis(front, v);
		out[i-start][1]=q.rotationAngleAboutAxis(vector3(0,1,0));
	}

	//out.column(1).op0(v0::alignAngles(out[0][1]));
	v0::alignAngles a(out[0][1]);
	a.calc(out.column(1).lval());
}

void MotionUtilTwo::setOpponentPos(Motion& m_Motion, const matrixn& in, int start)
{
	m_real len;
	quater q;
	vector3 v;
	vector3 front(0,0,1);
	for(int i=0; i<in.rows(); i++)
	{
		v.rotate(quater(in[i][1], vector3(0,1,0)), front);
		//m_Motion.pose2(i+start).m_oppenentPos.mult(v, in[i][0]);
		pose2(m_Motion,i+start).m_oppenentPos.mult(v, in[i][0]);
	}
}
