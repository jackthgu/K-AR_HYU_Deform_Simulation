// -*- mode: c++; indent-tabs-mode: t; tab-width: 4; c-basic-offset: 4; -*-
/*
 * Copyright (c) 2008, Hanyang university.
 * All rights reserved. This program is made available under the terms of the
 * Eclipse Public License v1.0 which accompanies this distribution, and is
 * available at http://www.eclipse.org/legal/epl-v10.html
 * Contributors:
 * Taesoo Kwon
 */
/** @file DynamicsSimulator/server/DynamicsSimulator_TRL_penalty.cpp
 *
 */
#include "physicsLib.h"
#include "DynamicsSimulator.h"
#include "Body.h"
#include <vector>
#include <map>
#include <algorithm>

#include "DynamicsSimulator_TRL_penalty.h"
#include "Link.h"
#include "LinkTraverse.h"
#include "LinkPath.h"
#include "ModelLoaderUtil.h"
#include "../MainLib/OgreFltk/VRMLloader.h"
#include "ForwardDynamicsABM.h"
#include "TRL_common.h"

using namespace OpenHRP;
using namespace std;


// #define INTEGRATOR_DEBUG
static const int debugMode = false;
static const bool enableTimeMeasure = false;

DynamicsSimulator_TRL_penalty::DynamicsSimulator_TRL_penalty(bool usc)
:DynamicsSimulator_penaltyMethod(usc)
{
	if(debugMode){
		cout << "DynamicsSimulator_TRL_penalty::DynamicsSimulator_TRL_penalty()" << endl;
	}

	// default integration method
	world.setRungeKuttaMethod();
}

DynamicsSimulator_TRL_penalty::DynamicsSimulator_TRL_penalty(const char* coldet)
:DynamicsSimulator_penaltyMethod(coldet)
{
	if(debugMode){
		cout << "DynamicsSimulator_TRL_penalty::DynamicsSimulator_TRL_penalty()" << endl;
	}

	// default integration method
	world.setRungeKuttaMethod();
}


DynamicsSimulator_TRL_penalty::~DynamicsSimulator_TRL_penalty()
{
	if(debugMode){
		cout << "DynamicsSimulator_TRL_penalty::~DynamicsSimulator_TRL_penalty()" << endl;
	}
}

Liegroup::dse3 DynamicsSimulator_TRL_penalty::calcMomentumCOM(int ichara)
{
#if 0
	// use all joints including internal dummy joints
	TRL::BodyPtr cinfo=world.body(ichara);
	Liegroup::dse3 out;
	vector3 com=cinfo->calcCM();
	cinfo->calcTotalMomentum(out.F(), out.M());
	return out.dAd(transf(quater(1,0,0,0), com));
#else
	// use only those described in the wrl file.
	// slightly different result from the above, but is 
	// consistent with jacobian matrices. 
	VRMLloader* skel=_characters[ichara]->skeleton;
	TRL::BodyPtr cinfo=world.body(ichara);

	::vector3 com(0,0,0);
	Liegroup::dse3 H(0,0,0,0,0,0);

	m_real totalMass=0.0;
	for(int ibone=1; ibone<skel->numBone(); ibone++)
	{
		VRMLTransform& bone=skel->VRMLbone(ibone);
		ASSERT(bone.mSegment);
		double mass=bone.mSegment->mass;
		transf & G=getWorldState(ichara)._global(bone);
		com+=(G*bone.mSegment->centerOfMass)*mass;
		totalMass+=mass;
		quater invR=getWorldState(ichara)._global(bone).rotation.inverse();

		TRL::Link* link=getTRLlink(cinfo,bone.HRPjointIndex(bone.numHRPjoints()-1));
		Liegroup::se3 V(invR*link->w, invR*link->v);
		Liegroup::Inertia I(mass, bone.momentsOfInertia(), mass*bone.localCOM());
		H+=(I*V).inv_dAd(G);
	}

	com/=totalMass;

	Liegroup::dse3 v;
	v.dAd(transf(quater(1,0,0,0), com), H);


	return v;
#endif
}
void DynamicsSimulator_TRL_penalty::_registerCharacter
(
 const char *name,
 CharacterInfo const& chara
 )
{
	TRL::BodyPtr body = TRL::loadBodyFromCharacterInfo(&chara);

	if(body){
		body->name = name;
		if(debugMode){
			std::cout << "Loaded Model:\n" << *body << std::endl;
		}
		collisionDetector->addModel(name, chara);
		world.addBody(body);
		_characters.push_back(new DynamicsSimulator::Character(chara.loader));
	}
	int ichara=_characters.size()-1;
	TRL::BodyPtr cinfo=((DynamicsSimulator_TRL_penalty*)this)->world.body(ichara);

	VRMLTransform& rootb=chara.loader->VRMLbone(1);
	int si=1;
	if(rootb.HRPjointType(0)==HRP_JOINT::FREE)
	{
		TRL::Link* j=getTRLlink(cinfo,0);
		j->dqIndex=0;
		si=2;
	}

	for(int i=si, ni=chara.loader->numBone(); i<ni; i++)
	{
		VRMLTransform& b=chara.loader->VRMLbone(i);
		MotionDOFinfo &dofInfo=chara.loader->dofInfo;
		int sj=b.mJoint->jointStartId;
		int sDOF=dofInfo.startDQ(i);
		int nDOF=dofInfo.endDQ(i)-sDOF;
		for(int jj=0; jj<nDOF; jj++)
		{
			TRL::Link* j=getTRLlink(cinfo,jj+sj);
			j->dqIndex=sDOF+jj;
		}
	}
	for(int i=0; i<chara.loader->numHRPjoints() ; i++)
		cout << i << *getTRLlink(cinfo, i)<<endl;
}

void DynamicsSimulator_TRL_penalty::setTimestep(double ts)
{
	world.setTimeStep(ts);
	for(int i=0; i<world.numBodies(); i++)
	{
		TRL::ForwardDynamicsABM* fd=world.forwardDynamics(i);
		fd->setTimeStep(ts);
	}
}
double DynamicsSimulator_TRL_penalty::getTimestep()
{
	return world.timeStep();
}
void DynamicsSimulator_TRL_penalty::setCurrentTime(double t)
{
    world.setCurrentTime(t);
}
void DynamicsSimulator_TRL_penalty::init(
		double timeStep,
		OpenHRP::DynamicsSimulator::IntegrateMethod integrateOpt)
{	
    world.setTimeStep(timeStep);
    world.setCurrentTime(0.0);

	if(integrateOpt == OpenHRP::DynamicsSimulator::EULER){
		world.setEulerMethod();
	} else {
		world.setRungeKuttaMethod();
	}

	int n = world.numBodies();
	for(int i=0; i < n; ++i){
        world.body(i)->initializeConfiguration();
	}
	world.initialize();
	world.setGravityAcceleration(world.getGravityAcceleration());
}

void DynamicsSimulator_TRL_penalty::initSimulation()
{
	if(debugMode){
		cout << "DynamicsSimulator_TRL_penalty::initSimulation()" << endl;
	}

	int n = _characters.size();
	for(int i=0; i<n; i++){
		TRL::ForwardDynamicsABM* fd=world.forwardDynamics(i);
		if(!fd) Msg::error("DynamicsSimulator_TRL_penalty::init not called yet!");
		fd->calcABMPhase1();
		fd->calcABMPhase2Part1();
	}
	_updateCharacterPose();

	collisionDetector->queryContactDeterminationForDefinedPairs(_characters, *collisions);
	_contactForces.clear();
	_calcContactForce(*collisions);

}

//#include "../BaseLib/utility/QPerformanceTimer.h"

void DynamicsSimulator_TRL_penalty::getWorldVelocity(int ichara, VRMLTransform* b
			, ::vector3 const& localpos
			, ::vector3& velocity) const
{

	TRL::BodyPtr cinfo=((DynamicsSimulator_TRL_penalty*)this)->world.body(ichara);
	TRL::Link* link=getTRLlink(cinfo,b->HRPjointIndex(b->numHRPjoints()-1));

	velocity=link->v+cross(link->w, link->R * localpos);
	//RE::output(b->name(), "%s",velocity.output().ptr());
}	

void DynamicsSimulator_TRL_penalty::getWorldAngVel(int ichara, VRMLTransform* b, ::vector3& angvel) const
{ 
  	TRL::BodyPtr cinfo=((DynamicsSimulator_TRL_penalty*)this)->world.body(ichara);
	TRL::Link* link=getTRLlink(cinfo,b->HRPjointIndex(b->numHRPjoints()-1));
  angvel=link->w;
}

void DynamicsSimulator_TRL_penalty::getWorldAcceleration(int ichara,VRMLTransform* b
			, ::vector3 const& localpos
			, ::vector3& acc) const
{
	TRL::BodyPtr cinfo=((DynamicsSimulator_TRL_penalty*)this)->world.body(ichara);
	TRL::Link* link=getTRLlink(cinfo,b->HRPjointIndex(b->numHRPjoints()-1));

	acc=link->dv+cross(link->dw, link->R* localpos);
}

void DynamicsSimulator_TRL_penalty::addForceToBone
(int ichara, VRMLTransform* b, ::vector3 const& localpos, ::vector3 const& force)
{
	TRL::BodyPtr cinfo=world.body(ichara);
	TRL::Link* link=getTRLlink(cinfo,b->HRPjointIndex(b->numHRPjoints()-1));

	// TRL uses inertial frame for external forces.

	::vector3 gf=getWorldState(ichara)._global(*b).toGlobalDir(force);
	::vector3 gp=getWorldState(ichara)._global(*b).toGlobalPos(localpos);

	link->fext+=gf;
	link->tauext+=gp.cross(gf);
}

static void _addForceToLink(TRL::WorldBase& world, int ichara, VRMLTransform* b, ::vector3 const& f, ::vector3 const& p)
{
	TRL::BodyPtr cinfo=world.body(ichara);
	TRL::Link* link=getTRLlink(cinfo,b->HRPjointIndex(b->numHRPjoints()-1));

	vector3 f_total(0.0);
	vector3 tau_total(0.0);

	link->fext   += f;
	link->tauext += p.cross(f);
}


bool DynamicsSimulator_TRL_penalty::stepSimulation()
{

	// set external forces
	for(int i=0; i<_contactForces.size(); i++)
	{		
		ContactForce& c=_contactForces[i];
		addForceToBone(c.chara, c.bone,c.p,c.f);
		
		TRL::BodyPtr cinfo=world.body(c.chara);
		TRL::Link* link=getTRLlink(cinfo,c.bone->HRPjointIndex(c.bone->numHRPjoints()-1));

		// TRL uses inertial frame for external forces.

		::vector3 gtau=getWorldState(c.chara)._global(*c.bone).toGlobalDir(c.tau);
		link->tauext+=gtau;
	}
	world.calcNextState();	


	_updateCharacterPose();
	collisionDetector->queryContactDeterminationForDefinedPairs(_characters, *collisions);
	_contactForces.clear();
	_calcContactForce(*collisions);

	for(int i=0; i<world.numBodies(); i++)
		world.body(i)->clearExternalForces();
	return true;
}




void DynamicsSimulator_TRL_penalty::setGVector
(
 const ::vector3& wdata
 )
{
	// setGvector has to be called before world.initialize()
	world.setGravityAcceleration(wdata);

	if(debugMode){
		cout << "DynamicsSimulator_TRL_penalty::setGVector("
			 << wdata[0] << ", "
			 << wdata[1] << ", "
			 << wdata[2] << ")" << endl;
	}
}


void DynamicsSimulator_TRL_penalty::getGVector
(
 ::vector3& g
 )
{
	g = world.getGravityAcceleration();

}

void DynamicsSimulator_TRL_penalty::calcBodyJacobianAt(int ichar, int ibone, matrixn& J, vector3 const& localpos)
{
	vector3 targetPos= getWorldState(ichar).global(ibone)*localpos;
	TRL::BodyPtr cinfo = world.body(ichar);
	assert(cinfo);

	VRMLTransform* b;
	VRMLloader& skel=skeleton(ichar);
	b=&skel.VRMLbone(1);
	TRL::Link* baseLink=getTRLlink(cinfo,b->HRPjointIndex(b->numHRPjoints()-1));
	b=&skel.VRMLbone(ibone);
	TRL::Link* targetLink=getTRLlink(cinfo,b->HRPjointIndex(b->numHRPjoints()-1));
	TRL::JointPath path(baseLink, targetLink);
	int height = 6;
	int width = path.numJoints();
	matrixn subj;
	subj.setSize(height, width);

	path.calcJacobian(subj, targetPos);
	unsigned int nJ = cinfo->numJoints();
	int totaldof = nJ;
	if( !cinfo->isStatic() ) totaldof += 6;
	J.setSize(6, totaldof);
	J.setAllValue(0.0);
		
	//quater invR;
	//invR.inverse(getWorldState(ichar).global(ibone).rotation);

	matrix3& R=targetLink->R;
	matrix3 invR;
	invR.inverse(R);

	vector3 axis(0,0,0);
	vector3 lin;
	for(int i=0; i<3; i++)
	{
		axis[i]=1.0;
		lin.cross(axis, targetPos-baseLink->p);
		setVector3(invR*axis, J, 0, i); 
		setVector3(invR*axis, J, 3, i+3);
		setVector3(invR*lin, J, 3, i); 
		axis[i]=0.0;
	}
	for (int i=0, ni=path.numJoints(); i<ni; i++)
	{
		int jtype=path.joint(i)->jointType;
		ASSERT(jtype==TRL::Link::ROTATIONAL_JOINT || jtype==TRL::Link::SLIDE_JOINT);
		int dqIndex=path.joint(i)->dqIndex;
		J.column(dqIndex).setVec3(0, invR*subj.column(i).toVector3(0));
		J.column(dqIndex).setVec3(3, invR*subj.column(i).toVector3(3));
	}
}

void DynamicsSimulator_TRL_penalty::calcDotBodyJacobianAt(int ichar, int ibone, matrixn& J, matrixn& DJ, vector3 const& localpos)
{
	calcBodyJacobianAt(ichar, ibone, J, localpos);

	vector3 targetPos= getWorldState(ichar).global(ibone)*localpos;
	TRL::BodyPtr cinfo = world.body(ichar);
	assert(cinfo);

	VRMLTransform* b;
	VRMLloader& skel=skeleton(ichar);
	b=&skel.VRMLbone(1);
	TRL::Link* baseLink=getTRLlink(cinfo,b->HRPjointIndex(b->numHRPjoints()-1));
	b=&skel.VRMLbone(ibone);
	TRL::Link* targetLink=getTRLlink(cinfo,b->HRPjointIndex(b->numHRPjoints()-1));
	TRL::JointPath path(baseLink, targetLink);
	int height = 6;
	int width = path.numJoints();
	matrixn subj;
	subj.setSize(height, width);

	//vector3 targetPos= targetLink->p+targetLink->R*localpos;

	path.calcJdot(subj, targetPos);
	unsigned int nJ = cinfo->numJoints();
	int totaldof = nJ;
	if( !cinfo->isStatic() ) totaldof += 6;
	DJ.setSize(6, totaldof);
	DJ.setAllValue(0.0);

	
	matrix3& R=targetLink->R;
	matrix3 invR, dotInvR;
	invR.inverse(R);
	matrix3 dotR=Liegroup::skew(targetLink->w)*R;
	//matrix3 dotR=R*Liegroup::skew(invR*targetLink->w); // identical
	// R * invR = I
	// dotR*invR+R*dotInvR=0
	// dot(invR)=-invR*dotR*invR
	dotInvR=-1.0*invR*dotR*invR;
		
	vector3 dotaxis(0,0,0);
	vector3 axis(0,0,0);
	vector3 lin;
	vector3 dotlin;
	vector3 arm;
	for(int i=0; i<3; i++)
	{
		axis[i]=1.0;
		// lin= skew(axis)*arm
		// dotlin = skew(axis)*dotArm
		lin.cross(axis, targetPos-baseLink->p);
		dotlin.cross(axis, targetLink->v + cross(targetLink->w, targetPos-targetLink->p)-baseLink->v);
		setVector3(dotInvR*axis, DJ, 0, i); // dotaxis is zero
		setVector3(dotInvR*axis, DJ, 3, i+3);
		setVector3(invR*dotlin+dotInvR*lin, DJ, 3, i); 
		axis[i]=0.0;
	}
	for (int i=0, ni=path.numJoints(); i<ni; i++)
	{
		int jtype=path.joint(i)->jointType;
		ASSERT(jtype==TRL::Link::ROTATIONAL_JOINT || jtype==TRL::Link::SLIDE_JOINT);
		int dqIndex=path.joint(i)->dqIndex;
		DJ.column(dqIndex).setVec3(0, invR*subj.column(i).toVector3(0)+dotInvR*R*J.column(dqIndex).toVector3(0));
		DJ.column(dqIndex).setVec3(3, invR*subj.column(i).toVector3(3)+dotInvR*R*J.column(dqIndex).toVector3(3));
	}
}

void DynamicsSimulator_TRL_penalty::calcJacobianAt
(
 int ichar, 
 int ibone,
 matrixn& J, 
 vector3 const& localpos
 )
{
	vector3 targetPos= getWorldState(ichar).global(ibone)*localpos;
	_calcJacobianAt(ichar, ibone, J, targetPos);
}
void DynamicsSimulator_TRL_penalty::_calcJacobianAt(int ichar, int ibone, matrixn& J, vector3 const& targetPos)
{
	//vector3 targetPos== targetLink->p+targetLink->R*localpos;
	TRL::BodyPtr cinfo = world.body(ichar);
	assert(cinfo);

	VRMLTransform* b;
	VRMLloader& skel=skeleton(ichar);
	b=&skel.VRMLbone(1);
	TRL::Link* baseLink=getTRLlink(cinfo,b->HRPjointIndex(b->numHRPjoints()-1));
	b=&skel.VRMLbone(ibone);
	TRL::Link* targetLink=getTRLlink(cinfo,b->HRPjointIndex(b->numHRPjoints()-1));
	TRL::JointPath path(baseLink, targetLink);
	int height = 6;
	int width = path.numJoints();
	matrixn subj;
	subj.setSize(height, width);

	path.calcJacobian(subj, targetPos);
	unsigned int nJ = cinfo->numJoints();
	int totaldof = nJ;
	if( !cinfo->isStatic() ) totaldof += 6;
	J.setSize(6, totaldof);
	J.setAllValue(0.0);
		
	vector3 axis(0,0,0);
	vector3 axis0;
	vector3 lin;
	for(int i=0; i<3; i++)
	{
		axis[i]=1.0;
		//axis0=baseLink->R* axis; // gmbs convention (local axis)
		axis0=axis;                // sml convention (global axis)
		lin.cross(axis0, targetPos-baseLink->p);
#ifdef SWAP_FORCE_AND_TORQUE
		setVector3(axis0, J, 0, i); 
		setVector3(axis0, J, 3, i+3);
		setVector3(lin, J, 3, i); 


#else
		setVector3(axis0, J, 0, i+3); 
		setVector3(axis0, J, 3, i);
		setVector3(lin, J, 3, i+3); 
#endif
		axis[i]=0.0;
	}
	for (int i=0, ni=path.numJoints(); i<ni; i++)
	{
		int jtype=path.joint(i)->jointType;
		ASSERT(jtype==TRL::Link::ROTATIONAL_JOINT || jtype==TRL::Link::SLIDE_JOINT);
		J.column(path.joint(i)->dqIndex).assign(subj.column(i));
	}
}
void DynamicsSimulator_TRL_penalty::calcDotJacobianAt
(
 int ichar, 
 int ibone,
 matrixn& DJ, 
 vector3 const& localpos
 )
{
	vector3 targetPos= getWorldState(ichar).global(ibone)*localpos;
	_calcDotJacobianAt(ichar, ibone, DJ, targetPos);
}
void DynamicsSimulator_TRL_penalty::_calcDotJacobianAt(int ichar, int ibone, matrixn& DJ, vector3 const& targetPos)
{
	TRL::BodyPtr cinfo = world.body(ichar);
	assert(cinfo);

	VRMLTransform* b;
	VRMLloader& skel=skeleton(ichar);
	b=&skel.VRMLbone(1);
	TRL::Link* baseLink=getTRLlink(cinfo,b->HRPjointIndex(b->numHRPjoints()-1));
	b=&skel.VRMLbone(ibone);
	TRL::Link* targetLink=getTRLlink(cinfo,b->HRPjointIndex(b->numHRPjoints()-1));
	TRL::JointPath path(baseLink, targetLink);
	int height = 6;
	int width = path.numJoints();
	matrixn subj;
	subj.setSize(height, width);

	//vector3 targetPos= targetLink->p+targetLink->R*localpos;

	path.calcJdot(subj, targetPos);
	unsigned int nJ = cinfo->numJoints();
	int totaldof = nJ;
	if( !cinfo->isStatic() ) totaldof += 6;
	DJ.setSize(6, totaldof);
	DJ.setAllValue(0.0);
		
	vector3 dotaxis(0,0,0);
	vector3 axis(0,0,0);
	vector3 axis0;
	vector3 dotlin;
	vector3 arm;
	for(int i=0; i<3; i++)
	{
		axis[i]=1.0;
		axis0=axis;                // sml convention (global axis)
		// lin= skew(axis0)*arm
		// dotlin = skew(axis0)*dotArm
		dotlin.cross(axis0, targetLink->v + cross(targetLink->w, targetPos-targetLink->p)-baseLink->v);
#ifdef SWAP_FORCE_AND_TORQUE
		setVector3(dotaxis, DJ, 0, i);
		setVector3(dotaxis, DJ, 3, i+3);
		setVector3(dotlin, DJ, 3, i); 
#else
		setVector3(dotaxis, DJ, 0, i+3);
		setVector3(dotaxis, DJ, 3, i);
		setVector3(dotlin, DJ, 3, i+3); 
#endif
		axis[i]=0.0;
	}
	for (int i=0, ni=path.numJoints(); i<ni; i++)
	{
		int jtype=path.joint(i)->jointType;
		ASSERT(jtype==TRL::Link::ROTATIONAL_JOINT || jtype==TRL::Link::SLIDE_JOINT);
		DJ.column(path.joint(i)->dqIndex).assign(subj.column(i));
	}
}

// output is compatible to MotionDOF class.
void DynamicsSimulator_TRL_penalty::getLinkData(int ichara, LinkDataType t, vectorn& out)
{
	VRMLloader const& l=*_characters[ichara]->skeleton;

	out.resize(l.dofInfo.numDOF());

	OpenHRP::DynamicsSimulator& s=*this;
	TRL::BodyPtr cinfo=world.body(ichara);

	vector3 p;
	double imag;
	vector3 v;
	for(int i=1; i<l.numBone(); i++)
	{
		VRMLTransform& b=(VRMLTransform&)l.bone(i);
		if(b.mJoint->jointType==HRP_JOINT::FIXED) continue;
		
		if(b.mJoint->jointType==HRP_JOINT::FREE)
		{
			TRL::Link* j=getTRLlink(cinfo,b.mJoint->jointStartId);

			ASSERT(l.dofInfo.hasTranslation(i));
			ASSERT(l.dofInfo.hasQuaternion(i));
			int sTDOF=l.dofInfo.startT(i);
			int sRDOF=l.dofInfo.startR(i);
			ASSERT(sRDOF-sTDOF==3);

			switch(t)
			{
			case OpenHRP::DynamicsSimulator::JOINT_VALUE:
				{
					p=j->p;
					quater q=toBase(j->attitude());
					imag=q.w;
					v.x=q.x;v.y=q.y;v.z=q.z;
				}
				break;
			case OpenHRP::DynamicsSimulator::JOINT_VELOCITY:
				{
					quater q=toBase(j->attitude());
					p=j->v;
					v=j->w;
					p.rotate(q.inverse());
					v.rotate(q.inverse());
					imag=0.0;
				}
				break;
			case OpenHRP::DynamicsSimulator::JOINT_ACCELERATION:
				p=j->dv;
				v=j->dw;
				imag=0.0;
				break;
			case OpenHRP::DynamicsSimulator::JOINT_TORQUE: p=j->fext;
				v=j->tauext;
				imag=0.0;
				break;
			default:
				ASSERT(0);
			}

			
			out.setVec3(sTDOF, p);
			out[sRDOF]=imag;
			out.setVec3(sRDOF+1, v);

		}
		else if (b.mJoint->jointType==HRP_JOINT::BALL){
			TRL::Link* j=getTRLlink(cinfo,b.mJoint->jointStartId);

			ASSERT(l.dofInfo.hasQuaternion(i));
			int sRDOF=l.dofInfo.startR(i);

			switch(t)
			{
			case OpenHRP::DynamicsSimulator::JOINT_VALUE:
				{
					quater q=toBase(j->attitude());
					imag=q.w;
					v.x=q.x;v.y=q.y;v.z=q.z;
				}
				break;
			case OpenHRP::DynamicsSimulator::JOINT_VELOCITY:
				v=j->w;
				imag=0.0;
				break;
			case OpenHRP::DynamicsSimulator::JOINT_ACCELERATION:
				v=j->dw;
				imag=0.0;
				break;
			case OpenHRP::DynamicsSimulator::JOINT_TORQUE:
				v=j->tauext;
				imag=0.0;
				break;
			default:
				ASSERT(0);
			}
			
			out[sRDOF]=imag;
			out.setVec3(sRDOF+1, v);
		}
		else			
		{
			int sj=b.mJoint->jointStartId;
			int sDOF=l.dofInfo.startT(i);
			int nDOF=l.dofInfo.endR(i)-sDOF;
			for(int jj=0; jj<nDOF; jj++)
			{

				switch(t)
				{
				case OpenHRP::DynamicsSimulator::JOINT_VALUE:
					out[sDOF+jj]=getTRLlink(cinfo,jj+sj)->q;
					break;
				case OpenHRP::DynamicsSimulator::JOINT_VELOCITY:
					out[sDOF+jj]=getTRLlink(cinfo,jj+sj)->dq;
					break;
				case OpenHRP::DynamicsSimulator::JOINT_ACCELERATION:
					out[sDOF+jj]=getTRLlink(cinfo,jj+sj)->ddq;
					break;
				case OpenHRP::DynamicsSimulator::JOINT_TORQUE:
					out[sDOF+jj]=getTRLlink(cinfo,jj+sj)->u;
					break;

				default:
					ASSERT(0);
				}
			}
		}

	}
}

// output is compatible to MotionDOF class.
void DynamicsSimulator_TRL_penalty::setLinkData(int ichara, LinkDataType t, vectorn const& in)
{
	VRMLloader const& l=*this->_characters[ichara]->skeleton;

	ASSERT(in.size()==l.dofInfo.numDOF());

	OpenHRP::DynamicsSimulator& s=*this;
	TRL::BodyPtr cinfo=this->world.body(ichara);

	vector3 p;
	double imag;
	vector3 v;

	for(int i=1; i<l.numBone(); i++)
	{
		VRMLTransform& b=(VRMLTransform&)l.bone(i);
		if(b.mJoint->jointType==HRP_JOINT::FIXED) continue;

		if(b.mJoint->jointType==HRP_JOINT::FREE)
		{
			TRL::Link* j=getTRLlink(cinfo,b.mJoint->jointStartId);

			ASSERT(l.dofInfo.hasTranslation(i));
			ASSERT(l.dofInfo.hasQuaternion(i));
			int sTDOF=l.dofInfo.startT(i);
			int sRDOF=l.dofInfo.startR(i);
			ASSERT(sRDOF-sTDOF==3);
			p=in.toVector3(sTDOF);
			imag=in[sRDOF];
			v=in.toVector3(sRDOF+1);

			switch(t)
			{
			case OpenHRP::DynamicsSimulator::JOINT_VALUE:
				{
					j->p=p;
					quater q(imag,v.x, v.y, v.z);
					q.normalize();
					j->setAttitude(toOpenHRP(q));
				}
				break;
			case OpenHRP::DynamicsSimulator::JOINT_VELOCITY:
				{
					::vector3 bv,bw;
					quater q=toBase(j->attitude());
					p.rotate(q);
					v.rotate(q);
					j->w=v;

					// this line is important because j->v is calculated from j->vo.
					j->vo= p - cross(j->w,j->p);
					j->v= p;

					imag=0.0;
				}
				break;
			case OpenHRP::DynamicsSimulator::JOINT_ACCELERATION:
				
				j->dw=v;

				// this line is important as j->dv is calculated form j->dvo.
				j->dvo = p - cross(j->dw, j->p) - cross(j->w, j->v);
				j->dv=p;
				
				imag=0.0;
				break;
			case OpenHRP::DynamicsSimulator::JOINT_TORQUE:
				j->fext=p;
				j->tauext=v;
				imag=0.0;
				break;
			default:
				ASSERT(0);
			}
		}
		else if(b.mJoint->jointType==HRP_JOINT::BALL)
		{
			TRL::Link* j=getTRLlink(cinfo,b.mJoint->jointStartId);

			int sRDOF=l.dofInfo.startR(i);
			imag=in[sRDOF];
			v=in.toVector3(sRDOF+1);

			switch(t)
			{
			case OpenHRP::DynamicsSimulator::JOINT_VALUE:
				{
					quater q(imag,v.x, v.y, v.z);
					q.normalize();
					j->setAttitude(toOpenHRP(q));
				}
				break;
			case OpenHRP::DynamicsSimulator::JOINT_VELOCITY:
				j->w=v;
				break;
			case OpenHRP::DynamicsSimulator::JOINT_ACCELERATION:
				j->dw=v;
				break;
			case OpenHRP::DynamicsSimulator::JOINT_TORQUE:
				//				j->tauext=v;
				break;
			default:
				ASSERT(0);
			}
		}
		else
		{
			int sj=b.mJoint->jointStartId;
			int sDOF=l.dofInfo.startT(i);
			int nDOF=l.dofInfo.endR(i)-sDOF;
			for(int jj=0; jj<nDOF; jj++)
			{

				switch(t)
				{
				case OpenHRP::DynamicsSimulator::JOINT_VALUE:
					getTRLlink(cinfo,sj+jj)->q=in[sDOF+jj];
					break;
				case OpenHRP::DynamicsSimulator::JOINT_VELOCITY:
					getTRLlink(cinfo,jj+sj)->dq=in[sDOF+jj];
					break;
				case OpenHRP::DynamicsSimulator::JOINT_ACCELERATION:
					getTRLlink(cinfo,jj+sj)->ddq=in[sDOF+jj];
					break;
				case OpenHRP::DynamicsSimulator::JOINT_TORQUE:
					getTRLlink(cinfo,jj+sj)->u=in[sDOF+jj];
					break;
				default:
					ASSERT(0);
				}
			}
		}
	}

	if(t==OpenHRP::DynamicsSimulator::JOINT_ACCELERATION)
	{
		// calc link->vo, w, R, p, sw, sv, v, cv, cw, wc, ...
		TRL::ForwardDynamicsABM* fd=world.forwardDynamics(ichara);
		fd->calcABMPhase1(); // position, velocity fk + gravity
		fd->calcAccelFK();	
		_updateCharacterPose();
	}
}

double DynamicsSimulator_TRL_penalty::currentTime()
{
	return world.currentTime();
}
void DynamicsSimulator_TRL_penalty::calcMassMatrix(int ichara, matrixn& M, vectorn & b)
{
	TRL::BodyPtr cinfo=((DynamicsSimulator_TRL_penalty*)this)->world.body(ichara);
	unsigned int nJ = cinfo->numJoints();
	int totaldof = nJ;
	if( !cinfo->isStatic() ) totaldof += 6;

	b.setSize(totaldof);
	cinfo->calcMassMatrix(M,b.column().lval(), world.getGravityAcceleration());

	/* moved to body.cpp
#ifdef SWAP_FORCE_AND_TORQUE
	matrixn temp;
	vectorn tempb;

	// swap tau and moment rows
	temp.assign(M.range(0,M.rows(),0,3));
	M.range(0,M.rows(),0,3).assign(M.range(0,M.rows(),3,6));
	M.range(0,M.rows(),3,6).assign(temp);
	temp.assign(M.range(0,3,0,M.cols()));
	M.range(0,3,0,M.cols()).assign(M.range(3,6,0,M.cols()));
	M.range(3,6,0,M.cols()).assign(temp);
	tempb.assign(b.range(0,3));
	b.range(0,3).assign(b.range(3,6));
	b.range(3,6).assign(tempb);
#endif // SWAP_FORCE_AND_TORQUE

*/
}
void DynamicsSimulator_TRL_penalty::getU(int ichara, double  out[]) const
{
	VRMLloader const& l=*_characters[ichara]->skeleton;


	TRL::BodyPtr cinfo=world.body(ichara);

	vector3 p;
	vector3 v;
	TRL::Link* j;
	for(int i=-1, ni=cinfo->numJoints(); i<ni; i++)
	{
		j=cinfo->joint(i);
		
		if(j->jointType==TRL::Link::FREE_JOINT)
		{
			setVector3( j->fext, out, 0);
			setVector3( j->tauext, out, 3);
		}
		else if(j->dqIndex!=-1)
		{
			out[j->dqIndex]=j->u;
		}
	}
}

void DynamicsSimulator_TRL_penalty::setU(int ichara, const vectorn& in)
{
	VRMLloader const& l=*_characters[ichara]->skeleton;


	TRL::BodyPtr cinfo=world.body(ichara);
	TRL::Link* j;
	for(int i=-1, ni=cinfo->numJoints(); i<ni; i++)
	{
		j=cinfo->joint(i);
		
		if(j->jointType==TRL::Link::FREE_JOINT)
		{
			j->fext=in.toVector3(0);
			j->tauext=in.toVector3(3);
		}
		else if(j->dqIndex!=-1)
		{
			j->u=in[j->dqIndex];
		}
	}
}


void DynamicsSimulator_TRL_penalty::poseToQ(vectorn const& v, vectorn& out) 
{ 
	int rdof=v.size(); out.setSize(rdof); 
	out.setVec3(0, v.toVector3(0)); 
	out[rdof-1]=v(3); 
	out.setVec3(3, v.toVector3(4)); 
	out.range(6, rdof-1)=v.range(7, rdof);
}
void DynamicsSimulator_TRL_penalty::dposeToDQ(quater const& rootOri, vectorn const& v, vectorn& out) 
{ 
	int rdof=v.size(); out.setSize(rdof-1); 
#ifdef SWAP_FORCE_AND_TORQUE
	out.setVec3(3, rootOri*v.toVector3(0)); // linvel
	out.setVec3(0, rootOri*v.toVector3(4)); // angvel
#else
	out.setVec3(0, rootOri*v.toVector3(0)); 
	out.setVec3(3, rootOri*v.toVector3(4)); 
#endif
	out.range(6, rdof-1)=v.range(7, rdof);
}
void DynamicsSimulator_TRL_penalty::torqueToU(const vectorn& v, vectorn& U)  
{ 
	int rdof=v.size(); U.setSize(rdof-1); 
	U.setVec3(0, v.toVector3(0)); 
	U.setVec3(3, v.toVector3(4)); 
	U.range(6, rdof-1).assign(v.range(7,rdof)); 
}

void DynamicsSimulator_TRL_penalty::setQ(int ichara, const double v[])
{
	VRMLloader const& l=*_characters[ichara]->skeleton;


	TRL::BodyPtr cinfo=world.body(ichara);
	TRL::Link* j;
	for(int i=-1, ni=cinfo->numJoints(); i<ni; i++)
	{
		j=cinfo->joint(i);
		
		if(j->jointType==TRL::Link::FREE_JOINT)
		{
			j->p=vector3(v[0],v[1],v[2]);
			quater q(v[dof(ichara)], v[3], v[4], v[5]);
			q.normalize();
			j->setAttitude(toOpenHRP(q));
		}
		else if(j->dqIndex!=-1)
		{
			j->q=v[j->dqIndex];
		}
	}
}
void DynamicsSimulator_TRL_penalty::getQ(int ichara, double v[]) const
{
	VRMLloader const& l=*_characters[ichara]->skeleton;


	TRL::Body* cinfo=world.body(ichara);
	TRL::Link* j;
	for(int i=-1, ni=cinfo->numJoints(); i<ni; i++)
	{
		j=cinfo->joint(i);
		
		if(j->jointType==TRL::Link::FREE_JOINT)
		{
			quater q=toBase(j->attitude());
			v[0]=j->p.x;
			v[1]=j->p.y;
			v[2]=j->p.z;
			v[dof(ichara)]=q.w;
			v[3]=q.x;
			v[4]=q.y;
			v[5]=q.z;
		}
		else if(j->dqIndex!=-1)
		{
			v[j->dqIndex]=j->q;
			//assert(j->q==j->q);
		}
	}
}
void DynamicsSimulator_TRL_penalty::_updateCharacterPose()
{
	int n = _characters.size();

	vectorn v;
	for(int i=n-1; i>=0; i--)
	{
		vectorn& _tempPose=_characters[i]->_tempPose;
		int ndof=dof(i);
		if (ndof==0) continue;
		if(rdof(i)==ndof)
			getQ(i, _tempPose);
		else
		{
			getQ(i,v);
			QToPose(v, _tempPose);
		}
		/*Posture pose;
		VRMLloader* loader=_characters[i]->skeleton;
		pose.Init(loader->numRotJoint(), loader->numTransJoint());
		_characters[i]->skeleton->dofInfo.setDOF(_tempPose, pose);
		_characters[i]->chain->setPose(pose);
		*/
		if(ndof)
			_characters[i]->chain->setPoseDOF(_tempPose);
	}
}

void DynamicsSimulator_TRL_penalty::getDQ(int ichara, double out[]) const
{
	VRMLloader const& l=*_characters[ichara]->skeleton;


	TRL::BodyPtr cinfo=world.body(ichara);
	TRL::Link* j;
	for(int i=-1, ni=cinfo->numJoints(); i<ni; i++)
	{
		j=cinfo->joint(i);

		if(j->jointType==TRL::Link::FREE_JOINT)
		{

#ifdef SWAP_FORCE_AND_TORQUE
			out[0]=j->w.x;
			out[1]=j->w.y;
			out[2]=j->w.z;
			out[3]=j->v.x;
			out[4]=j->v.y;
			out[5]=j->v.z;
#else
			out[0]=j->v.x;
			out[1]=j->v.y;
			out[2]=j->v.z;
			out[3]=j->w.x;
			out[4]=j->w.y;
			out[5]=j->w.z;
#endif
		}
		else if(j->dqIndex!=-1)
		{
			out[j->dqIndex]=j->dq;
		}
	}
}
void DynamicsSimulator_TRL_penalty::setDQ(int ichara, const double in[])
{
	VRMLloader const& l=*_characters[ichara]->skeleton;


	TRL::BodyPtr cinfo=world.body(ichara);

	TRL::Link* j;
	for(int i=-1, ni=cinfo->numJoints(); i<ni; i++)
	{
		j=cinfo->joint(i);
		
		if(j->jointType==TRL::Link::FREE_JOINT)
		{
#ifdef SWAP_FORCE_AND_TORQUE
			j->w.x=in[0];
			j->w.y=in[1];
			j->w.z=in[2];
			j->v.x=in[3];
			j->v.y=in[4];
			j->v.z=in[5];
#else
			j->v.x=in[0];
			j->v.y=in[1];
			j->v.z=in[2];
			j->w.x=in[3];
			j->w.y=in[4];
			j->w.z=in[5];
#endif

			// this line is important because j->v is calculated from j->vo.
			j->vo= j->v - cross(j->w,j->p);
		}
		else if(j->dqIndex!=-1)
		{
			j->dq=in[j->dqIndex];
		}
	}
}
void DynamicsSimulator_TRL_penalty::setDDQ(int ichara, vectorn const& in)
{
	VRMLloader const& l=*_characters[ichara]->skeleton;


	TRL::BodyPtr cinfo=world.body(ichara);

	vector3 dv;
	vector3 dw;
	TRL::Link* j;
	for(int i=-1, ni=cinfo->numJoints(); i<ni; i++)
	{
		j=cinfo->joint(i);
		
		if(j->jointType==TRL::Link::FREE_JOINT)
		{
			int sTDOF=0;
#ifdef SWAP_FORCE_AND_TORQUE
			dw=in.toVector3(sTDOF);
			dv=in.toVector3(sTDOF+3);

#else
			dv=in.toVector3(sTDOF);
			dw=in.toVector3(sTDOF+3);
#endif
			j->dw=dw;
			// this line is important as j->dv is calculated form j->dvo.
			j->dvo = dv - cross(j->dw, j->p) - cross(j->w, j->v);
			j->dv=dv;
		}
		else if(j->dqIndex!=-1)
		{
			j->ddq=in[j->dqIndex];
		}
	}
}
TRL::Link* DynamicsSimulator_TRL_penalty::getLink(int ichara, int ibone, int jj)
{
	TRL::BodyPtr cinfo=((DynamicsSimulator_TRL_penalty*)this)->world.body(ichara);
	VRMLTransform* b;
	VRMLloader& skel=skeleton(ichara);
	b=&skel.VRMLbone(ibone);
	int sj=b->mJoint->jointStartId;
	TRL::Link* j=getTRLlink(cinfo,jj+sj);
	return j;
}
int DynamicsSimulator_TRL_penalty::calcS(int ichara, int ibone, matrixn& S)
{
	TRL::BodyPtr cinfo = world.body(ichara);
	VRMLTransform* b;
	VRMLloader& skel=skeleton(ichara);
	b=&skel.VRMLbone(ibone);
	TRL::Link* targetLink=getTRLlink(cinfo,b->HRPjointIndex(b->numHRPjoints()-1));
	b=(VRMLTransform*)(b->parent());
	TRL::Link* baseLink=getTRLlink(cinfo,b->HRPjointIndex(b->numHRPjoints()-1));
	TRL::JointPath path(baseLink, targetLink);

	path.calcAngularJacobian(S);
	return targetLink->dqIndex-S.cols()+1;
}
int DynamicsSimulator_TRL_penalty::getDQindex(int ichara, int ibone, int idof)
{ 
	return getLink(ichara, ibone, idof)->dqIndex;
}

static inline double dot(double* a, Liegroup::dse3 const& b)
{
	double out=0;
	for (int i=0; i<6; i++)
		out+=a[i]*b[i];
	return out;
}

static inline void radd(::vectorn & v, Liegroup::dse3 const& vv)
{
	for (int i=0; i<6; i++)
		v(i)+=vv[i];
}
static inline Liegroup::dse3 mult(matrixn const& in, Liegroup::dse3 const& in2)
{
	Liegroup::dse3 out;
	for (int i=0; i<6; i++)
	{
		out[i]=dot(&in(i,0), in2);
	}	
	return out;
}


void DynamicsSimulator_TRL_penalty::calcMomentumDotJacobian(int ichar, matrixn& jacobian, matrixn& dotjacobian)
{
	VRMLloader const& l=*_characters[ichar]->skeleton;
	VRMLTransform* b;
	VRMLloader& skel=skeleton(ichar);
	TRL::BodyPtr cinfo=world.body(ichar);
	matrixn j,dj;
	jacobian.setSize(6,l.dofInfo.numActualDOF());
	jacobian.setAllValue(0);
	dotjacobian.setSize(6, l.dofInfo.numActualDOF());
	dotjacobian.setAllValue(0);

	::vector3 COM;
	::vector3 COMVEL;
	double totalMass;
	//COM=cinfo->calcCM();
	COM=calculateCOM(ichar, totalMass);
	COMVEL=calculateCOMvel(ichar, totalMass);
	matrixn bJ, bdotJ;
	matrixn dAd_T(6,6);
	matrixn dot_dAd_T(6,6);
	for(int i=1; i<l.numBone(); i++) {

		vector3 localpos(0,0,0);
		calcDotBodyJacobianAt(ichar,i,bJ,bdotJ,localpos);
		b=&skel.VRMLbone(i);
		TRL::Link* link=getTRLlink(cinfo,b->HRPjointIndex(b->numHRPjoints()-1));

		matrix4 bodyT(getWorldState(ichar).global(i));
		matrix4 invBodyT;
		matrix3 invR;
		invR.inverse(link->R);
		invBodyT.inverse(bodyT);
		Liegroup::se3 V(invR*link->w, invR*link->v); // body velocity
		matrix4 dotBodyT=calcDotT(bodyT, V);

		// T * invT = I
		// dotT*invT+T*dotInvT=0
		// dot(invT)=-invT*dotT*invT
		matrix4 T=invBodyT*Liegroup::toSE3(COM);
		Liegroup::dAd(dAd_T, T);
		matrix4 dotCOM;
		Liegroup::zero(dotCOM);
		dotCOM.setTranslation(COMVEL);
		matrix4 dotT= invBodyT*dotCOM - invBodyT*dotBodyT*invBodyT * Liegroup::toSE3(COM); 
		Liegroup::dot_dAd(dot_dAd_T, T, dotT);

		VRMLTransform& bone=*b;
		double mass=bone.mSegment->mass;
		Liegroup::Inertia bodyI(mass, bone.momentsOfInertia(), mass*bone.localCOM());

		for (int j=0; j<bJ.cols(); j++){
			Liegroup::se3 cj=Liegroup::to_se3(bJ.column(j));
			Liegroup::dse3 temp=bodyI*cj;
			radd(jacobian.column(j).lval(),mult( dAd_T,temp));

			radd(dotjacobian.column(j).lval(), 
					mult(dot_dAd_T,temp)+mult(dAd_T,bodyI*Liegroup::to_se3(bdotJ.column(j))));
			// dAd(T)=
			//    (     R       0 )'
			//    (   skew(T)*R R )
			// cj2= dAd(T)*I*cj
		}
	}
}


#include "../../PhysicsLib/TRL/eigenSupport.h"
enum {X,Y,Z};
#define VCOPYN(B,A,n)		(memcpy(B, A, sizeof(double)*(n)))
inline matrix3 GetRotationMatrixBaseToWorldZYX_reordered (const vector3& xyz)
{
  double x = xyz.x;
  double y = xyz.y;
  double z = xyz.z;

  matrix3 M;
  //  http://docs.leggedrobotics.com/kindr/cheatsheet_latest.pdf (Euler ZYX)
  M.setValue( cos(y)*cos(z), cos(z)*sin(x)*sin(y) - cos(x)*sin(z), sin(x)*sin(z) + cos(x)*cos(z)*sin(y),
       cos(y)*sin(z), cos(x)*cos(z) + sin(x)*sin(y)*sin(z), cos(x)*sin(y)*sin(z) - cos(z)*sin(x),
             -sin(y),                        cos(y)*sin(x),                        cos(x)*cos(y));
  return M;
}
inline matrix3 GetM (const vector3& xyz)
{
  double z = xyz.z;
  double y = xyz.y;

  // Euler ZYX rates to angular velocity
  // http://docs.leggedrobotics.com/kindr/cheatsheet_latest.pdf
  matrix3 M;

  M(0,X) =  cos(y)*cos(z);
  M(0,Y) = -sin(z);  
  M(0,Z) = 0.0;
  M(1,X) =  cos(y)*sin(z);
  M(1,Y) =  cos(z);  
  M(1,Z) =  0.0;
  M(2,X) =  -sin(y);
  M(2,Y) =  0.0;
  M(2,Z) = 1.0;          

  return M;
}
inline matrix3 GetInvM (const vector3& xyz)
{
  double x = xyz.x;
  double z = xyz.z;
  double y = xyz.y;

  // Euler ZYX rates to angular velocity
  // http://docs.leggedrobotics.com/kindr/cheatsheet_latest.pdf
  matrix3 M;

  M(2,X) =  cos(z)*sin(y)/cos(y);
  M(2,Y) = sin(y)*sin(z)/cos(y);  
  M(2,Z)=1.0;
  M(1,X) =  -sin(z);
  M(1,Y) =  cos(z);  
  M(1,Z)=0.0;
  M(0,X) =  cos(z)/cos(y);
  M(0,Y)=sin(z)/cos(y);
  M(0,Z) = 0.0;          

  return M;
}
// getEulerRate(euler pos, world angular velocity0
inline vector3 getEulerRate (const vector3& pos, const vector3 &ang) 
{
	return GetInvM(pos)*ang;
}

// (eulerpos, eulerrate)
inline vector3 getAngularVelocityInWorld (const vector3& pos, const vector3& euler_rate)
{
  return GetM(pos)*euler_rate;
}

// from YUP to ZUP
inline vector3 toZUP(vector3 const& v)
{
	return vector3(v.z, v.x, v.y);
}
inline quater toZUP(quater const& q)
{
	return quater(q.w, q.z, q.x, q.y);
}
// from ZUP to YUP
inline vector3 ZUPtoYUP(vector3 const& v)
{
	return vector3(v.y, v.z, v.x);
}
// from ZUP to YUP
inline quater ZUPtoYUP(quater const& q)
{
	return quater(q.w, q.y,  q.z, q.x);
}

// eulerZYX contains RZ, RY, RX
inline vector3 getDotEuler(double eulerZYX[], vector3 const& ang)
{
	// reorder to XYZ
	vector3 euler(eulerZYX[2], eulerZYX[1], eulerZYX[0]);
	vector3 eulerRate=getEulerRate(euler, ang);


#ifdef TEST_EULER_CONVERTER
	// test inverse mapping
	vector3 ang2=getAngularVelocityInWorld(euler, eulerRate);
	printf("ang %s %s\n", ang.output().ptr(), ang2.output().ptr());
#endif
	// reorder to ZYX
	return vector3(eulerRate.z, eulerRate.y, eulerRate.x);
}
void DynamicsSimulator_TRL_penalty::stateToEulerZYX(vectorn const& q, vectorn const& dq, vectorn& eulerState) const
{
	DynamicsSimulator_TRL_penalty const& robot=*this;

	int NDOF=robot.dof();

	eulerState.resize(NDOF*2);
	double *state=&eulerState[0];
	double* temp=&q[0];
	double* temp2=&dq[0];

	// 3~6: exp_coord, 0~3: pos
	state[0]=temp[0];
	state[1]=temp[1];
	state[2]=temp[2];
	quater qq(temp[NDOF], temp[3], temp[4], temp[5]);
	double o[3];
	qq.getRotation("ZYX", o);
	state[3]=o[0];
	state[4]=o[1];
	state[5]=o[2];

	vector3 ang(temp2[0], temp2[1], temp2[2]);
	vector3 dotEuler=getDotEuler(o, ang);

	state[NDOF]=temp2[3];
	state[NDOF+1]=temp2[4];
	state[NDOF+2]=temp2[5];
	state[NDOF+3]=dotEuler.x;
	state[NDOF+4]=dotEuler.y;
	state[NDOF+5]=dotEuler.z;

	VCOPYN(&state[6], &temp[6], NDOF-6);
	VCOPYN(&state[NDOF+6], &temp2[6], NDOF-6);
}
void DynamicsSimulator_TRL_penalty::stateToEulerYXZ(vectorn const& q, vectorn const& dq, vectorn& eulerState) const
{
	DynamicsSimulator_TRL_penalty const& robot=*this;

	int NDOF=robot.dof();

	eulerState.resize(NDOF*2);
	double *state=&eulerState[0];
	double* temp=&q[0];
	double* temp2=&dq[0];

	// 3~6: exp_coord, 0~3: pos
	state[0]=temp[0];
	state[1]=temp[1];
	state[2]=temp[2];
	quater qq(temp[NDOF], temp[3], temp[4], temp[5]);
	double o[3];
	toZUP(qq).getRotation("ZYX", o);
	state[3]=o[0];
	state[4]=o[1];
	state[5]=o[2];

#ifdef TEST_EULER_CONVERTER
	double o2[3];
	qq.getRotation("YXZ", o2);
	printf("t: %f %f %f == %f %f %f\n",o[2], o[1], o[0], o2[2], o2[1], o2[0]);
#endif
	vector3 ang(temp2[0], temp2[1], temp2[2]);
	vector3 dotEuler=getDotEuler(o, toZUP(ang));

	state[NDOF]=temp2[3];
	state[NDOF+1]=temp2[4];
	state[NDOF+2]=temp2[5];
	state[NDOF+3]=dotEuler.x;
	state[NDOF+4]=dotEuler.y;
	state[NDOF+5]=dotEuler.z;

	VCOPYN(&state[6], &temp[6], NDOF-6);
	VCOPYN(&state[NDOF+6], &temp2[6], NDOF-6);
}
void DynamicsSimulator_TRL_penalty::eulerYXZtoState(vectorn const& eulerState, vectorn& _state) const
{
	int NDOF=dof();
	_state.resize(NDOF*2+1);
	double* temp=&_state[0];
	double* state=&eulerState[0];

	temp[0]=state[0];
	temp[1]=state[1];
	temp[2]=state[2];
	quater q;
	q.setRotation("YXZ", (double*)(&state[3])); 
	temp[NDOF+0]=q.w;
	temp[3]=q.x;
	temp[4]=q.y;
	temp[5]=q.z;
	VCOPYN(&temp[6], &state[6], NDOF-6);

	// reorder to XYZ 
	vector3 euler(state[5], state[4], state[3]);
	vector3 dotEuler(state[NDOF+5], state[NDOF+4], state[NDOF+3]);
	vector3 ang=ZUPtoYUP(getAngularVelocityInWorld(euler, dotEuler));

	temp[NDOF+4]=state[NDOF];
	temp[NDOF+5]=state[NDOF+1];
	temp[NDOF+6]=state[NDOF+2];
	temp[NDOF+1]=ang.x;
	temp[NDOF+2]=ang.y;
	temp[NDOF+3]=ang.z;

	// fillDQ
	VCOPYN(&temp[NDOF+1+6], &state[NDOF+6], NDOF-6);
}
void DynamicsSimulator_TRL_penalty::inverseDynamics(vectorn const& q, vectorn const& dq, vectorn const& ddq, vectorn& u)
{
	int ichara= 0;
	TRL::BodyPtr cinfo=((DynamicsSimulator_TRL_penalty*)this)->world.body(ichara);
	unsigned int nJ = cinfo->numJoints();
	int totaldof = nJ;
	if( !cinfo->isStatic() ) totaldof += 6;
	u.setSize(totaldof);

	_contactForces.clear();
	for(int i=0; i<world.numBodies(); i++)
		world.body(i)->clearExternalForces();

	setQ(ichara, q);
	_updateCharacterPose();
	setDQ(ichara, dq); // w, v, dq -> vo
	setDDQ(ichara, ddq); // dw, dv, ddq ->  dvo

	ASSERT(false);


}

void DynamicsSimulator_TRL_penalty::eulerZYXtoState(vectorn const& eulerState, vectorn& _state) const
{
	int NDOF=dof();
	_state.resize(NDOF*2+1);
	double* temp=&_state[0];
	double* state=&eulerState[0];

	temp[0]=state[0];
	temp[1]=state[1];
	temp[2]=state[2];
	quater q;
	q.setRotation("ZYX", (double*)(&state[3])); 
	temp[NDOF+0]=q.w;
	temp[3]=q.x;
	temp[4]=q.y;
	temp[5]=q.z;
	VCOPYN(&temp[6], &state[6], NDOF-6);

	// reorder to XYZ 
	vector3 euler(state[5], state[4], state[3]);
	vector3 dotEuler(state[NDOF+5], state[NDOF+4], state[NDOF+3]);
	vector3 ang=getAngularVelocityInWorld(euler, dotEuler);

	temp[NDOF+4]=state[NDOF];
	temp[NDOF+5]=state[NDOF+1];
	temp[NDOF+6]=state[NDOF+2];
	temp[NDOF+1]=ang.x;
	temp[NDOF+2]=ang.y;
	temp[NDOF+3]=ang.z;

	// fillDQ
	VCOPYN(&temp[NDOF+1+6], &state[NDOF+6], NDOF-6);
}
