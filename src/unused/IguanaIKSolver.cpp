#include "stdafx.h"
#include "MainLib/OgreFltk/FlLayout.h"
//#include "../mainlibpython/mainlibpython.h"
#include "MainLib/OgreFltk/MotionPanel.h"
#include "MainLib/OgreFltk/objectList.h"
#include "BaseLib/math/intervals.h"
#include "BaseLib/math/Metric.h"
#include "BaseLib/motion/MotionUtil.h"
#include "BaseLib/motion/MotionLoader.h"
#include "BaseLib/motion/FullbodyIK.h"
#include "BaseLib/utility/scoped_ptr.h"
#include "BaseLib/math/conversion.h"
#include "BaseLib/motion/FullbodyIK.h"
#include "Stitch.h"
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>
#include "btBulletDynamicsCommon.h"
#include "bulletTools.h"
#include "IguanaIKSolver.h"

static void addVertAxis(vectorn& pos)
{
	vector3 p1=pos.toVector3(0);
	vector3 p2=pos.toVector3(3);
	vector3 p3=pos.toVector3(6);
	vector3 p4=pos.toVector3(9);

	double len=0;
	vector3 dir1=p2-p1; len+=dir1.length(); dir1.normalize();
   	vector3 dir2=p4-p3; len+=dir2.length(); dir2.normalize();
	vector3 dir3=p3-p1; len+=dir3.length(); dir3.normalize();
   	vector3 dir4=p4-p2; len+=dir4.length(); dir4.normalize();
	len/=4.0;

	vector3 up;
	up.cross( (dir1+dir2)*0.5, (dir3+dir4)*0.5);
	
	pos.resize(pos.size()+3);
	pos.setVec3(pos.size()-3, (p1+p2+p3+p4)*0.25+up*len);
}

IguanaIKSolver ::IguanaIKSolver (MotionLoader& skel, btRigidBody* terrain)
	:skel(skel),
	mTerrain(terrain),
	numEE(5)
{
	ee.resize(numEE);

	ee[0].bone=skel.getBoneByVoca(MotionLoader::LEFTANKLE).child();
	ee[1].bone=skel.getBoneByVoca(MotionLoader::RIGHTANKLE).child();
	ee[2].bone=skel.getBoneByVoca(MotionLoader::LEFTWRIST).child();
	ee[3].bone=skel.getBoneByVoca(MotionLoader::RIGHTWRIST).child();
	ee[4].bone=skel.getBoneByVoca(MotionLoader::CHEST).child();

	ik.reset(MotionUtil::createFullbodyIk_LimbIK(skel, ee));
}

IguanaIKSolver::~IguanaIKSolver (){}

void IguanaIKSolver::getAffectedDOF(intvectorn & rot_joint_index, intvectorn & trans_joint_index)
{
	ik->getAffectedDOF(rot_joint_index, trans_joint_index);
	trans_joint_index.setSize(1);
	trans_joint_index[0]=0;		
}

// interface type 1. All derived classes should reimplement this interface.
void IguanaIKSolver::IKsolve(Posture const& input_pose, vector3N const& constraintPositions, intvectorn & rot_joint_index, quaterN& delta_rot, intvectorn & trans_joint_index, vector3N& delta_trans)
{
	Posture pose=input_pose;
	// 4다리의 예재 동작에서의 높이 구하기


	vectorn desired_height(numEE);
	vectorn curHeight(numEE);
	vectorn floorHeight(numEE);

	skel.setPose(pose);

	for(int i=0; i<numEE; i++)
		desired_height[i]=ee[i].bone->getTranslation().y;

	vector3N con(numEE);
	vector3N projectedCon(numEE);

#define DEBUG_DRAW
	for(int i=0; i<numEE; i++)
	{
		con[i]=ee[i].bone->getTranslation();
		projectedCon[i]=con[i];
		projectedCon[i].y=getTerrainHeight(mTerrain, ToBullet(con[i]))+desired_height[i];
#ifdef DEBUG_DRAW
		static ObjectList list;
		list.drawSphere(con[i], TString("ccon", i), "green", 3.0);
		list.drawSphere(projectedCon[i], TString("ppcon", i), "green", 3.0);
#endif
	}

	numEE=4;
	projectedCon.resize(numEE);
	con.resize(numEE);

	PointCloudMetric metric;
	vectorn pa;
	pa=vecView(projectedCon);
	addVertAxis(pa);
	vectorn pb;
	pb=vecView(con);
	addVertAxis(pb);
	metric.CalcDistance(pa, pb);

	transf newRoot;
	newRoot.mult(metric.m_transfB, pose.rootTransformation());


#define ROTATE_ROOT
#ifdef ROTATE_ROOT
	// root돌려준후 다시 4다리의 현재 동작에서 높이 구하기.
	pose.setRootTransformation(newRoot);
	skel.setPose(pose);
#endif

	numEE=5;	// 꼬리 추가됨.
	projectedCon.resize(numEE);
	con.resize(numEE);

	for(int i=0; i<numEE; i++)
	{
		con[i]=ee[i].bone->getTranslation();

#ifdef DEBUG_DRAW
		static ObjectList list;
		list.drawSphere(con[i], TString("con", i), "red", 3.0);
#endif
		con[i].y=getTerrainHeight(mTerrain, ToBullet(con[i]))+desired_height[i];
#ifdef DEBUG_DRAW
		list.drawSphere(con[i], TString("pcon", i), "blue", 3.0);
#endif
	}

	// 차이만큼 IK해주기.
	static scoped_ptr<MotionUtil::FullbodyIK> ik;

	if(!ik.get())
	{
		ik.reset(
				MotionUtil::createFullbodyIk_LimbIK(skel,ee));
		//MotionUtil::createFullbodyIk_MultiTarget(skel, ee));
		//MotionUtil::createFullbodyIk(skel, ee));
	}


	ik->IKsolve(pose, con, rot_joint_index, delta_rot);

#ifdef ROTATE_ROOT
	if(rot_joint_index[0]==0)
		delta_rot[0].difference(input_pose.m_aRotations[0], delta_rot[0]*pose.m_aRotations[0]);
	else
	{
		quater diff;
		diff.difference(input_pose.m_aRotations[0], delta_rot[0]*pose.m_aRotations[0]);
		rot_joint_index.pushFront(0);
		delta_rot.pushFront(diff);
	}

	trans_joint_index.setSize(1);
	trans_joint_index[0]=0;		
	delta_trans.setSize(1);
	delta_trans[0].difference(input_pose.m_aTranslations[0], pose.m_aTranslations[0]);
#endif

#define EXTRAPOLATE_TAIL
#ifdef EXTRAPOLATE_TAIL
	Bone* tail=ee[4].bone;
	Msg::verify(rot_joint_index[rot_joint_index.size()-1]==tail->parent()->rotJointIndex(),"tail test1");
	Msg::verify(rot_joint_index[rot_joint_index.size()-2]==tail->parent()->parent()->rotJointIndex(),"tail test2");
	printf("%d\n", tail->rotJointIndex());
	delta_rot[rot_joint_index.size()-2].scale(0.5);
	delta_rot[rot_joint_index.size()-1]=delta_rot[rot_joint_index.size()-2];
#endif
}
