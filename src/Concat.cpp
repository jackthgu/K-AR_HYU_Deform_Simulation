#include "stdafx.h"
#include "BaseLib/motion/Motion.h"
#include "./Concat.h"
#include "BaseLib/motion/FootPrint.h"
#include "BaseLib/motion/MotionUtil.h"
#include "BaseLib/math/Filter.h"

using namespace MotionUtil;

void ReconstructConcat::concat(Motion& front, const Motion& add) const
{
	int numFrameOld=front.numFrames();
	front.Concat(&add);

	if(numFrameOld!=0)
		front.ReconstructDataByDifference(numFrameOld-1);
}

void ReconstructAdjustConcat::concat(Motion& front, const Motion& add) const
{
	int numFrameOld=front.numFrames();
	front.Concat(&add);

	quater q_y, q_z;
	
	q_y.setRotation(vector3(0,1,0), adjustAngleYFirst);
	front.pose(numFrameOld).m_dq.leftMult(q_y);

	q_y.setRotation(vector3(0,1,0), adjustAngleY/(float)add.numFrames());
	q_z.setRotation(vector3(0,0,1), adjustAngleZ);

	for(int i=numFrameOld; i<front.numFrames(); i++)
	{
		front.pose(i).m_dq.leftMult(q_y);
		front.pose(i).m_offset_q.leftMult(q_z);
	}

	if(numFrameOld!=0)
		front.ReconstructDataByDifference(numFrameOld-1);
}

void ReconstructAdjustConcatTwoWay::concat(Motion& front, const Motion& add) const
{
	int numFrameOld=front.numFrames();
	front.Concat(&add);
	quater q_y;
	if(safe==0)
	{
		q_y.setRotation(vector3(0,1,0), adjustAngleY);
		front.pose(numFrameOld).m_dq.leftMult(q_y);
		if(numFrameOld!=0)
			front.ReconstructDataByDifference(numFrameOld-1);
	}
	else
	{
		int kernelsize=safe*2+1;
		vectorn kernel;
		Filter::GetGaussFilter(kernelsize, kernel);
		
		for(int i=-1*safe; i<=safe; i++)
		{
			q_y.setRotation(vector3(0,1,0), adjustAngleY*kernel[i+safe]);
			front.pose(i+numFrameOld).m_dq.leftMult(q_y);
		}
		front.ReconstructDataByDifference(numFrameOld-safe-1);
	}

}

void RootTrajectoryConcat::concat(Motion& front, const Motion& add) const
{
	int numFrameOld=front.numFrames();

	front.Concat(&add);

	if(numFrameOld!=0)
	{
		vector3 dvFront, dvAdd;
		dvFront.difference(front.pose(numFrameOld-2).m_aTranslations[0], front.pose(numFrameOld-1).m_aTranslations[0]);
		dvAdd.difference(front.pose(numFrameOld).m_aTranslations[0], front.pose(numFrameOld+1).m_aTranslations[0]);

		// 너무 느리면 root orientation을 사용하도록 해야할 듯.

        dvFront.y=0;
		dvAdd.y=0;

		quater q;
		//q.setAxisRotation(vector3(0,1,0), dvAdd, dvFront);		
		q.axisToAxis(dvAdd, dvFront);

		MotionUtil::rotate(front, q, numFrameOld, front.numFrames());

		// translate
		vector3 translate;
		translate.difference(front.pose(numFrameOld).m_aTranslations[0], front.pose(numFrameOld-1).m_aTranslations[0]+dvFront);
		translate.y=0;

		MotionUtil::translate(front, translate, numFrameOld, front.numFrames());
	}	
}

void RootOrientationConcat::concat(Motion& front, const Motion& add) const
{
	int numFrameOld=front.numFrames();

	front.Concat(&add);

	if(numFrameOld!=0)
	{/*
		quater dqFront, dqAdd;
		dqFront.difference(front.pose(numFrameOld-2).m_aRotations[0], front.pose(numFrameOld-1).m_aRotations[0]);
		dqAdd.difference(front.pose(numFrameOld).m_aRotations[0], front.pose(numFrameOld+1).m_aRotations[0]);

		quater dq_mid;
		dq_mid.safeSlerp(dqFront, dqAdd, 0.5);

		quater target;
		target.mult(dq_mid, front.pose(numFrameOld-1).m_aRotations[0]);

		quater dif, difY, offset;
		dif.difference(front.pose(numFrameOld).m_aRotations[0], target);
		dif.decompose(difY, offset);

		front.rotate(numFrameOld, front.numFrames(), difY);*/

		quater roty_2, roty_1, roty, roty1, offset;
		front.pose(numFrameOld-2).m_aRotations[0].decompose(roty_2, offset);
		front.pose(numFrameOld-1).m_aRotations[0].decompose(roty_1, offset);
		front.pose(numFrameOld).m_aRotations[0].decompose(roty, offset);
		front.pose(numFrameOld+1).m_aRotations[0].decompose(roty1, offset);

		quater dqFront, dqAdd;
		dqFront.difference(roty_2, roty_1);
		dqAdd.difference(roty, roty1);

		quater dq_mid;
		dq_mid.safeSlerp(dqFront, dqAdd, 0.5);

		quater target;
		target.mult(dq_mid, roty_1);

		quater dif;
		dif.difference(roty, target);
		MotionUtil::rotate(front, dif, numFrameOld, front.numFrames());

		vector3 dvFront, dvAdd, dv;
		dvFront.difference(front.pose(numFrameOld-2).m_aTranslations[0], front.pose(numFrameOld-1).m_aTranslations[0]);
		dvAdd.difference(front.pose(numFrameOld).m_aTranslations[0], front.pose(numFrameOld+1).m_aTranslations[0]);
		dv.lerp(dvFront, dvAdd, 0.5);

		// translate
		vector3 translate;
		translate.difference(front.pose(numFrameOld).m_aTranslations[0], front.pose(numFrameOld-1).m_aTranslations[0]+dv);		
		translate.y=0;

		MotionUtil::translate(front, translate, numFrameOld, front.numFrames());
	}	
}

void FootAdjustConcat::concat(Motion& front, const Motion& add) const
{
	int numFrameOld=front.numFrames();
	mFirstConcat.concat(front, add);
	
	enum {LEFT, RIGHT, NUM_CON};

	int cnstr[NUM_CON];
	cnstr[0]=CONSTRAINT_LEFT_TOE;
	cnstr[1]=CONSTRAINT_RIGHT_TOE;
	
	int constraint1;
	int constraint2;
	matrixn footPos;
	vector3 footPos1[NUM_CON];
	vector3 footPos2[NUM_CON];
	bool needStitch[NUM_CON];
	
	int footStart, footEnd;
	intvectorn interval;

	for(int con=0; con<NUM_CON; con++)
	{
		constraint1=front.isConstraint(numFrameOld-front.NumFrames(0.12), numFrameOld, cnstr[con]);
		constraint2=front.isConstraint(numFrameOld,numFrameOld+front.NumFrames(0.12), cnstr[con]);

		if(constraint1!=-1 && constraint2!=-1)
		{
			needStitch[con]=true;
			mFootPrint.getFootInterval(front, constraint1, cnstr[con], footStart, footEnd);
			mFootPrint.getFootPrints(front, footStart, numFrameOld, cnstr[con], interval, footPos);

			if(footPos.rows()==0)
			{
				needStitch[con]=false;
			}
			else
			{
				footPos1[con]=footPos.row(footPos.rows()-1).toVector3();

				mFootPrint.getFootInterval(front, constraint2, cnstr[con], footStart, footEnd);
				mFootPrint.getFootPrints(front, numFrameOld, footEnd, cnstr[con], interval, footPos);
				
				if(footPos.rows()==0)
					needStitch[con]=false;
				else
					footPos2[con]=footPos.row(0).toVector3();
			}
		}
		else
			needStitch[con]=false;
	}

	
	if(needStitch[0] && needStitch[1])
	{
		// 두발의 방향과 센터를 align한다.
        vector3 delta1, delta2;		
		vector3 center1, center2;
		delta1.difference(footPos1[LEFT], footPos1[RIGHT]);
		delta2.difference(footPos2[LEFT], footPos2[RIGHT]);
		delta1.y=0;
		delta2.y=0;
		delta1.normalize();
		delta2.normalize();

		quater q;
		q.axisToAxis(delta2, delta1);

		center1.add(footPos1[LEFT], footPos1[RIGHT]);
		center2.add(footPos2[LEFT], footPos2[RIGHT]);
		center1/=2.0f;
		center2/=2.0f;

		/*
		front.translate(numFrameOld, front.numFrames(), center2*-1);
		front.rotate(numFrameOld, front.numFrames(), q);
		front.translate(numFrameOld, front.numFrames(), center1);
		*/
		
		MotionUtil::translate(front, center1-center2,numFrameOld, front.numFrames());
		
    }
	else if(needStitch[0] && !needStitch[1])
	{
		vector3 delta;
		delta.difference(footPos2[0], footPos1[0]);
		delta.y=0;
		MotionUtil::translate(front, delta,numFrameOld, front.numFrames());		
	}
	else if(!needStitch[0] && needStitch[1])
	{
		vector3 delta;
		delta.difference(footPos2[1], footPos1[1]);
		delta.y=0;
		MotionUtil::translate(front, delta, numFrameOld, front.numFrames());
	}
}
