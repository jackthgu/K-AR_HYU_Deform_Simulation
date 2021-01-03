#include "stdafx.h"
#include "SoftDemoWin.h"
#include <OgreSubMesh.h>
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

///btSoftBody implementation by Nathanael Presson

#include "stdafx.h"
#include <OgrePrerequisites.h>
#include <Ogre.h>
#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"

#include "BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpa2.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btIDebugDraw.h"
//#include "BMF_Api.h"
#include "BunnyMesh.h"
#include "TorusMesh.h"
#include <stdio.h> //printf debugging
#include "LinearMath/btConvexHull.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"

#include "Ogre_ShapeDrawer.h"
#include "../../MainLib/OgreFltk/RE.h"
#include "../../MainLib/OgreFltk/FlChoice.h"
#include "../../MainLib/OgreFltk/FltkRenderer.h"


static float	gCollisionMargin = 0.05f/*0.05f*/;
#include "SoftDemo.h"

extern float eye[3];

const int maxProxies = 32766;
const int maxOverlap = 65535;



#ifdef _DEBUG
const int gNumObjects = 1;
#else
const int gNumObjects = 1;//try this in release mode: 3000. never go above 16384, unless you increate maxNumObjects  value in DemoApplication.cp
#endif

const int maxNumObjects = 32760;

#define CUBE_HALF_EXTENTS 1.5
#define EXTRA_HEIGHT -10.f

//
void SoftDemo::createStack( btCollisionShape* boxShape, float halfCubeSize, int size, float zPos )
{
	btTransform trans;
	trans.setIdentity();

	for(int i=0; i<size; i++)
	{
		// This constructs a row, from left to right
		int rowSize = size - i;
		for(int j=0; j< rowSize; j++)
		{
			btVector4 pos;
			pos.setValue(
				-rowSize * halfCubeSize + halfCubeSize + j * 2.0f * halfCubeSize,
				halfCubeSize + i * halfCubeSize * 2.0f,
				zPos);

			trans.setOrigin(pos);
			btScalar mass = 1.f;

			btRigidBody* body = 0;
			body = localCreateRigidBody(mass,trans,boxShape);

		}
	}
}

btRigidBody* createOgreRigidBody(const btRigidBody::btRigidBodyConstructionInfo& constructionInfo, Ogre::SceneNode* parent);

btRigidBody*	SoftDemo::localCreateRigidBody(float mass, const btTransform& startTransform,btCollisionShape* shape)
{
	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0,0,0);
	if (isDynamic)
		shape->calculateLocalInertia(mass,localInertia);

	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

#define USE_MOTIONSTATE 1
#ifdef USE_MOTIONSTATE
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

	btRigidBody::btRigidBodyConstructionInfo cInfo(mass,myMotionState,shape,localInertia);

	btRigidBody* body = //new btRigidBody(cInfo);
		createOgreRigidBody(cInfo, RE::ogreRootSceneNode());

#else
	btRigidBody* body = new btRigidBody(mass,0,shape,localInertia);	
	body->setWorldTransform(startTransform);
#endif//

	m_dynamicsWorld->addRigidBody(body);
	
	return body;
}

////////////////////////////////////

extern int gNumManifold;
extern int gOverlappingPairs;
extern int gTotalContactPoints;

void SoftDemo::clientMoveAndDisplay()
{
//	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 




	//float dt = getDeltaTimeMicroseconds() * 0.000001f;

	float dt=1.0/420;
	//	printf("dt = %f: ",dt);


	if (m_dynamicsWorld)
	{
//#define FIXED_STEP
#ifdef FIXED_STEP
		m_dynamicsWorld->stepSimulation(dt=1.0f/60.f,0);

#else
		//during idle mode, just run 1 simulation step maximum
		int maxSimSubSteps = 1;
		dt = 1.0/420.f;

		int numSimSteps = 0;
		numSimSteps = m_dynamicsWorld->stepSimulation(dt,maxSimSubSteps);		

#ifdef VERBOSE_TIMESTEPPING_CONSOLEOUTPUT
		if (!numSimSteps)
			printf("Interpolated transforms\n");
		else
		{
			if (numSimSteps > maxSimSubSteps)
			{
				//detect dropping frames
				printf("Dropped (%i) simulation steps out of %i\n",numSimSteps - maxSimSubSteps,numSimSteps);
			} else
			{
				printf("Simulated (%i) steps\n",numSimSteps);
			}
		}
#endif //VERBOSE_TIMESTEPPING_CONSOLEOUTPUT

#endif		


		m_softBodyWorldInfo.m_sparsesdf.GarbageCollect();

		//optional but useful: debug drawing

	
		
	}
	
	renderme(); 

	//some additional debugging info
#ifdef PRINT_CONTACT_STATISTICS
	printf("num manifolds: %i\n",gNumManifold);
	printf("num gOverlappingPairs: %i\n",gOverlappingPairs);
	printf("num gTotalContactPoints : %i\n",gTotalContactPoints );
#endif //PRINT_CONTACT_STATISTICS

	gTotalContactPoints = 0;
}



//
// Random
//

static inline btScalar	UnitRand()
{
	return(rand()/(btScalar)RAND_MAX);
}

static inline btScalar	SignedUnitRand()
{
	return(UnitRand()*2-1);
}

static inline btVector3	Vector3Rand()
{
	const btVector3	p=btVector3(SignedUnitRand(),SignedUnitRand(),SignedUnitRand());
	return(p.normalized());
}

//
// Rb rain
//
static void	Ctor_RbUpStack(SoftDemo* pdemo,int count)
{
	float				mass=10;
	btCollisionShape*	shape[]={	new btSphereShape(1.5),
		new btBoxShape(btVector3(1,1,1)),
		new btCylinderShapeX(btVector3(4,1,1))};
	static const int	nshapes=sizeof(shape)/sizeof(shape[0]);
	for(int i=0;i<count;++i)
	{
		btTransform startTransform;
		startTransform.setIdentity();
		startTransform.setOrigin(btVector3(0,1+6*i,0));
		btRigidBody*		body=pdemo->localCreateRigidBody(mass,startTransform,shape[i%nshapes]);
	}
}

//
// Big ball
//
static void	Ctor_BigBall(SoftDemo* pdemo,btScalar mass=10)
{
	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(0,13,0));
	btRigidBody*		body=pdemo->localCreateRigidBody(mass,startTransform,new btSphereShape(3));
}

//
// Big plate
//
static void	Ctor_BigPlate(SoftDemo* pdemo,btScalar mass=15)
{
	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(0,4,0.5));
	btRigidBody*		body=pdemo->localCreateRigidBody(mass,startTransform,new btBoxShape(btVector3(5,1,5)));
	body->setFriction(1);
}

//
// Linear stair
//
static void Ctor_LinearStair(SoftDemo* pdemo,const btVector3& org,const btVector3& sizes,btScalar angle,int count)
{
	btBoxShape*	shape=new btBoxShape(sizes);
	for(int i=0;i<count;++i)
	{
		btTransform startTransform;
		startTransform.setIdentity();
		startTransform.setOrigin(org+btVector3(sizes.x()*i*2,sizes.y()*i*2,0));
		btRigidBody* body=pdemo->localCreateRigidBody(0,startTransform,shape);
		body->setFriction(1);
	}
}

//
// Softbox
//
static btSoftBody* Ctor_SoftBox(SoftDemo* pdemo,const btVector3& p,const btVector3& s)
{
	const btVector3	h=s*0.5;
	const btVector3	c[]={	p+h*btVector3(-1,-1,-1),
		p+h*btVector3(+1,-1,-1),
		p+h*btVector3(-1,+1,-1),
		p+h*btVector3(+1,+1,-1),
		p+h*btVector3(-1,-1,+1),
		p+h*btVector3(+1,-1,+1),
		p+h*btVector3(-1,+1,+1),
		p+h*btVector3(+1,+1,+1)};
	btSoftBody*		psb=btSoftBodyHelpers::CreateFromConvexHull(pdemo->m_softBodyWorldInfo,c,8);
	psb->generateBendingConstraints(2,1);
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb);
	
	return(psb);
}

//
// SoftBoulder
//
static btSoftBody* Ctor_SoftBoulder(SoftDemo* pdemo,const btVector3& p,const btVector3& s,int np,int id)
{
	btAlignedObjectArray<btVector3>	pts;
	if(id) srand(id);
	for(int i=0;i<np;++i)
	{
		pts.push_back(Vector3Rand()*s+p);
	}
	btSoftBody*		psb=btSoftBodyHelpers::CreateFromConvexHull(pdemo->m_softBodyWorldInfo,&pts[0],pts.size());
	psb->generateBendingConstraints(2,1);
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb);

	return(psb);
}

//#define TRACEDEMO { pdemo->demoname=__FUNCTION__+5;printf("Launching demo: " __FUNCTION__ "\r\n"); }

//
// Basic ropes
//
static void	Init_Ropes(SoftDemo* pdemo)
{
	//TRACEDEMO
	const int n=15;
	for(int i=0;i<n;++i)
	{
		btSoftBody*	psb=btSoftBodyHelpers::CreateRope(pdemo->m_softBodyWorldInfo,	btVector3(-10,0,i*0.25),
			btVector3(10,0,i*0.25),
			16,
			1+2);
		psb->m_cfg.iterations	=	4;
		psb->m_cfg.kLST			=	0.1+(i/(btScalar)(n-1))*0.9;
		psb->setTotalMass(20);
		pdemo->getSoftDynamicsWorld()->addSoftBody(psb);
	
	}
}

//
// Rope attach
//
static void	Init_RopeAttach(SoftDemo* pdemo)
{
	//TRACEDEMO
	struct	Functors
	{
		static btSoftBody* CtorRope(SoftDemo* pdemo,const btVector3& p)
		{
			btSoftBody*	psb=btSoftBodyHelpers::CreateRope(pdemo->m_softBodyWorldInfo,p,p+btVector3(10,0,0),8,1);
			psb->setTotalMass(50);
			pdemo->getSoftDynamicsWorld()->addSoftBody(psb);
			return(psb);
		}
	};
	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(12,8,0));
	btRigidBody*		body=pdemo->localCreateRigidBody(50,startTransform,new btBoxShape(btVector3(2,6,2)));
	btSoftBody*	psb0=Functors::CtorRope(pdemo,btVector3(0,8,-1));
	btSoftBody*	psb1=Functors::CtorRope(pdemo,btVector3(0,8,+1));
	psb0->appendAnchor(psb0->getNodes().size()-1,body);
	psb1->appendAnchor(psb1->getNodes().size()-1,body);
}

//
// Cloth attach
//
static void	Init_ClothAttach(SoftDemo* pdemo)
{
	//TRACEDEMO
	const btScalar	s=4;
	const btScalar	h=6;
	const int		r=9;
	btSoftBody*		psb=btSoftBodyHelpers::CreatePatch(pdemo->m_softBodyWorldInfo,btVector3(-s,h,-s),
		btVector3(+s,h,-s),
		btVector3(-s,h,+s),
		btVector3(+s,h,+s),r,r,4+8,true);
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb);

	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(0,h,-(s+3.5)));
	btRigidBody*		body=pdemo->localCreateRigidBody(20,startTransform,new btBoxShape(btVector3(s,1,3)));
	psb->appendAnchor(0,body);
	psb->appendAnchor(r-1,body);
}

//
// Impact
//
static void	Init_Impact(SoftDemo* pdemo)
{
	//TRACEDEMO
	btSoftBody*	psb=btSoftBodyHelpers::CreateRope(pdemo->m_softBodyWorldInfo,	btVector3(0,0,0),
		btVector3(0,-1,0),
		0,
		1);
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb);
	psb->m_cfg.kCHR=0.5;
	btTransform startTransform;
	startTransform.setIdentity();
	startTransform.setOrigin(btVector3(0,20,0));
	btRigidBody*		body=pdemo->localCreateRigidBody(10,startTransform,new btBoxShape(btVector3(2,2,2)));
}

//
// Collide
//
static void	Init_Collide(SoftDemo* pdemo)
{
	//TRACEDEMO
	struct Functor
		{
		static btSoftBody* Create(SoftDemo* pdemo,const btVector3& x,const btVector3& a)
			{
			btSoftBody*	psb=btSoftBodyHelpers::CreateFromTriMesh(pdemo->m_softBodyWorldInfo,gVertices,
				&gIndices[0][0],
				NUM_TRIANGLES);
			psb->generateBendingConstraints(2,1);
			psb->m_cfg.iterations=2;
			psb->m_cfg.collisions|=btSoftBody::fCollision::VF_SS;
			psb->randomizeConstraints();
			btMatrix3x3	m;
			m.setEulerZYX(a.x(),a.y(),a.z());
			psb->transform(btTransform(m,x));
			psb->scale(btVector3(2,2,2));
			psb->setTotalMass(50,true);
			pdemo->getSoftDynamicsWorld()->addSoftBody(psb);
			return(psb);
			}
		};
	for(int i=0;i<3;++i)
		{
		Functor::Create(pdemo,btVector3(3*i,2,0),btVector3(SIMD_PI/2*(1-(i&1)),SIMD_PI/2*(i&1),0));
		}
}

//
// Collide2
//
static void	Init_Collide2(SoftDemo* pdemo)
{
	//TRACEDEMO
	struct Functor
		{
		static btSoftBody* Create(SoftDemo* pdemo,const btVector3& x,const btVector3& a)
			{
			btSoftBody*	psb=btSoftBodyHelpers::CreateFromTriMesh(pdemo->m_softBodyWorldInfo,gVerticesBunny,
				&gIndicesBunny[0][0],
				BUNNY_NUM_TRIANGLES);
			psb->generateBendingConstraints(2,0.5);
			psb->m_cfg.iterations	=	2;
			psb->m_cfg.kDF			=	0.5;
			psb->m_cfg.collisions	|=	btSoftBody::fCollision::VF_SS;
			psb->randomizeConstraints();
			btMatrix3x3	m;
			m.setEulerZYX(a.x(),a.y(),a.z());
			psb->transform(btTransform(m,x));
			psb->scale(btVector3(6,6,6));
			psb->setTotalMass(100,true);
			pdemo->getSoftDynamicsWorld()->addSoftBody(psb);
			return(psb);
			}
		};
	for(int i=0;i<3;++i)
		{
		Functor::Create(pdemo,btVector3(0,-1+5*i,0),btVector3(0,SIMD_PI/2*(i&1),0));
		}	
}

//
// Collide3
//
static void	Init_Collide3(SoftDemo* pdemo)
{
	//TRACEDEMO
		{
		const btScalar	s=8;
		btSoftBody*		psb=btSoftBodyHelpers::CreatePatch(	pdemo->m_softBodyWorldInfo,btVector3(-s,0,-s),
			btVector3(+s,0,-s),
			btVector3(-s,0,+s),
			btVector3(+s,0,+s),
			15,15,1+2+4+8,true);
		psb->m_cfg.kLST			=	0.4;
		psb->m_cfg.collisions	|=	btSoftBody::fCollision::VF_SS;
		psb->setTotalMass(150);
		pdemo->getSoftDynamicsWorld()->addSoftBody(psb);
		}
		{		
		const btScalar	s=4;
		const btVector3	o=btVector3(4,10,0);
		btSoftBody*		psb=btSoftBodyHelpers::CreatePatch(	pdemo->m_softBodyWorldInfo,
			btVector3(-s,0,-s)+o,
			btVector3(+s,0,-s)+o,
			btVector3(-s,0,+s)+o,
			btVector3(+s,0,+s)+o,
			7,7,0,true);
		psb->generateBendingConstraints(2,0.5);
		psb->m_cfg.kLST			=	0.4;
		psb->m_cfg.collisions	|=	btSoftBody::fCollision::VF_SS;
		psb->setTotalMass(150);
		pdemo->getSoftDynamicsWorld()->addSoftBody(psb);
		}
}

//
// Aerodynamic forces, 50x1g flyers
//
static void	Init_Aero(SoftDemo* pdemo)
{
	//TRACEDEMO
	const btScalar	s=2;
	const btScalar	h=10;
	const int		segments=6;
	const int		count=50;
	for(int i=0;i<count;++i)
	{
		btSoftBody*		psb=btSoftBodyHelpers::CreatePatch(pdemo->m_softBodyWorldInfo,btVector3(-s,h,-s),
			btVector3(+s,h,-s),
			btVector3(-s,h,+s),
			btVector3(+s,h,+s),
			segments,segments,
			0,true);
		psb->generateBendingConstraints(2,1);
		psb->m_cfg.kLF			=	0.004;
		psb->m_cfg.kDG			=	0.0003;
		psb->m_cfg.aeromodel	=	btSoftBody::eAeroModel::V_TwoSided;
		btTransform		trs;
		btQuaternion	rot;
		btVector3		ra=Vector3Rand()*0.1;
		btVector3		rp=Vector3Rand()*15+btVector3(0,20,80);
		rot.setEuler(SIMD_PI/8+ra.x(),-SIMD_PI/7+ra.y(),ra.z());
		trs.setIdentity();
		trs.setOrigin(rp);
		trs.setRotation(rot);
		psb->transform(trs);
		psb->setTotalMass(0.1);
		psb->addForce(btVector3(0,2,0),0);
		pdemo->getSoftDynamicsWorld()->addSoftBody(psb);

	}
}

//
// Friction
//
static void	Init_Friction(SoftDemo* pdemo)
{
	//TRACEDEMO
	const btScalar	bs=2;
	const btScalar	ts=bs+bs/4;
	for(int i=0,ni=20;i<ni;++i)
	{
		const btVector3	p(-ni*ts/2+i*ts,-10+bs,40);
		btSoftBody*		psb=Ctor_SoftBox(pdemo,p,btVector3(bs,bs,bs));
		psb->m_cfg.kDF	=	0.1 * ((i+1)/(btScalar)ni);
		psb->addVelocity(btVector3(0,0,-10));
	}
}

//
// Pressure
//
static void	Init_Pressure(SoftDemo* pdemo)
{
	//TRACEDEMO
	btSoftBody*	psb=btSoftBodyHelpers::CreateEllipsoid(pdemo->m_softBodyWorldInfo,btVector3(35,25,0),
		btVector3(1,1,1)*3,
		512);
	psb->m_cfg.kLST		=	0.1;
	psb->m_cfg.kDF		=	1;
	psb->m_cfg.kDP		=	0.001; // fun factor...
	psb->m_cfg.kPR		=	2500;
	psb->setTotalMass(30,true);
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb);

	Ctor_BigPlate(pdemo);
	Ctor_LinearStair(pdemo,btVector3(0,0,0),btVector3(2,1,5),0,10);
}

//
// Volume conservation
//
static void	Init_Volume(SoftDemo* pdemo)
{
	//TRACEDEMO
	btSoftBody*	psb=btSoftBodyHelpers::CreateEllipsoid(pdemo->m_softBodyWorldInfo,btVector3(35,25,0),
		btVector3(1,1,1)*3,
		512);
	psb->m_cfg.kLST		=	0.45;
	psb->m_cfg.kVC		=	20;
	psb->setTotalMass(50,true);
	psb->setPose(true,false);
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb);

	Ctor_BigPlate(pdemo);
	Ctor_LinearStair(pdemo,btVector3(0,0,0),btVector3(2,1,5),0,10);
}

//
// Stick+Bending+Rb's
//
static void	Init_Sticks(SoftDemo* pdemo)
{
	//TRACEDEMO
	const int		n=16;
	const int		sg=4;
	const btScalar	sz=5;
	const btScalar	hg=4;
	const btScalar	in=1/(btScalar)(n-1);
	for(int y=0;y<n;++y)
	{
		for(int x=0;x<n;++x)
		{
			const btVector3	org(-sz+sz*2*x*in,
				-10,
				-sz+sz*2*y*in);
			btSoftBody*		psb=btSoftBodyHelpers::CreateRope(	pdemo->m_softBodyWorldInfo,	org,
				org+btVector3(hg*0.001,hg,0),
				sg,
				1);
			psb->m_cfg.iterations	=	1;
			psb->m_cfg.kDP		=	0.005;
			psb->m_cfg.kCHR		=	0.1;
			for(int i=0;i<3;++i)
			{
				psb->generateBendingConstraints(2+i,1);
			}
			psb->setMass(1,0);
			psb->setTotalMass(0.01);
			pdemo->getSoftDynamicsWorld()->addSoftBody(psb);

		}
	}
	Ctor_BigBall(pdemo);
}

//
// 100kg cloth locked at corners, 10 falling 10kg rb's.
//
static void	Init_Cloth(SoftDemo* pdemo)
{
	//TRACEDEMO
	const btScalar	s=8;
	btSoftBody*		psb=btSoftBodyHelpers::CreatePatch(	pdemo->m_softBodyWorldInfo,btVector3(-s,0,-s),
		btVector3(+s,0,-s),
		btVector3(-s,0,+s),
		btVector3(+s,0,+s),
		31,31,

		//		31,31,
		1+2+4+8,true);
	psb->generateBendingConstraints(2,1);
	psb->m_cfg.kLST			=	0.4;
	psb->setTotalMass(150);
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb);

	Ctor_RbUpStack(pdemo,10);
}

//
// 100kg Stanford's bunny
//
static void	Init_Bunny(SoftDemo* pdemo)
{
	//TRACEDEMO
	btSoftBody*	psb=btSoftBodyHelpers::CreateFromTriMesh(pdemo->m_softBodyWorldInfo,gVerticesBunny,
		&gIndicesBunny[0][0],
		BUNNY_NUM_TRIANGLES);
	psb->generateBendingConstraints(2,0.5);
	psb->m_cfg.iterations	=	2;
	psb->m_cfg.kDF			=	0.5;
	psb->randomizeConstraints();
	psb->scale(btVector3(6,6,6));
	psb->setTotalMass(100,true);
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb);

}

//
// 100kg Stanford's bunny with pose matching
//
static void	Init_BunnyMatch(SoftDemo* pdemo)
{
	//TRACEDEMO
	btSoftBody*	psb=btSoftBodyHelpers::CreateFromTriMesh(pdemo->m_softBodyWorldInfo,	gVerticesBunny,
		&gIndicesBunny[0][0],
		BUNNY_NUM_TRIANGLES);
	psb->m_cfg.kDF		=	0.5;
	psb->m_cfg.kLST		=	0.1;
	psb->m_cfg.kMT		=	0.05;
	psb->randomizeConstraints();
	psb->scale(btVector3(6,6,6));
	psb->setTotalMass(100,true);
	psb->setPose(true,true);
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb);	

}

//
// 50Kg Torus
//
static void	Init_Torus(SoftDemo* pdemo)
{
	//TRACEDEMO
	btSoftBody*	psb=btSoftBodyHelpers::CreateFromTriMesh(	pdemo->m_softBodyWorldInfo,	gVertices,
		&gIndices[0][0],
		NUM_TRIANGLES);
	psb->generateBendingConstraints(2,1);
	psb->m_cfg.iterations=2;
	psb->randomizeConstraints();
	btMatrix3x3	m;
	m.setEulerZYX(SIMD_PI/2,0,0);
	psb->transform(btTransform(m,btVector3(0,4,0)));
	psb->scale(btVector3(2,2,2));
	psb->setTotalMass(50,true);
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb);

}

//
// 50Kg Torus with pose matching
//
static void	Init_TorusMatch(SoftDemo* pdemo)
{
	//TRACEDEMO
	btSoftBody*	psb=btSoftBodyHelpers::CreateFromTriMesh(pdemo->m_softBodyWorldInfo,	gVertices,
		&gIndices[0][0],
		NUM_TRIANGLES);
	psb->m_cfg.kLST=0.1;
	psb->m_cfg.kMT=0.05;
	psb->randomizeConstraints();
	btMatrix3x3	m;
	m.setEulerZYX(SIMD_PI/2,0,0);
	psb->transform(btTransform(m,btVector3(0,4,0)));
	psb->scale(btVector3(2,2,2));
	psb->setTotalMass(50,true);
	psb->setPose(true,true);
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb);


}

unsigned	current_demo=0;

void	SoftDemo::clientResetScene()
{
	/* Clean up	*/ 
	for(int i=m_dynamicsWorld->getNumCollisionObjects()-1;i>0;i--)
	{
		btCollisionObject*	obj=m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody*		body=btRigidBody::upcast(obj);
		if(body&&body->getMotionState())
		{
			delete body->getMotionState();
		}
		btSoftBody* softBody = btSoftBody::upcast(obj);
		if (softBody)
		{
			getSoftDynamicsWorld()->removeSoftBody(softBody);
		} else
		{
			m_dynamicsWorld->removeCollisionObject(obj);
		}
		delete obj;
	}
	
	m_softBodyWorldInfo.m_sparsesdf.Reset();
	/* Init		*/ 
	void (*demofncs[])(SoftDemo*)=
	{
		Init_Cloth,
		Init_Pressure,
		Init_Volume,
		Init_Ropes,
		Init_RopeAttach,
		Init_ClothAttach,
		Init_Sticks,	
		Init_Collide,
		Init_Collide2,
		Init_Collide3,
		Init_Impact,
		Init_Aero,
		Init_Friction,			
		Init_Torus,
		Init_TorusMatch,
		Init_Bunny,
		Init_BunnyMatch,
	};
	current_demo=current_demo%(sizeof(demofncs)/sizeof(demofncs[0]));


	m_softBodyWorldInfo.air_density		=	(btScalar)1.2;
	m_softBodyWorldInfo.water_density	=	0;
	m_softBodyWorldInfo.water_offset		=	0;
	m_softBodyWorldInfo.water_normal		=	btVector3(0,0,0);
	m_softBodyWorldInfo.m_gravity.setValue(0,-10,0);


	demofncs[current_demo](this);
}

void	SoftDemo::renderme()
{
	btIDebugDraw*	idraw=m_dynamicsWorld->getDebugDrawer();
	
	m_dynamicsWorld->debugDrawWorld();
	
	/* Bodies		*/ 
	btVector3	ps(0,0,0);
	int			nps=0;
	
	btSoftBodyArray&	sbs=getSoftDynamicsWorld()->getSoftBodyArray();
	for(int ib=0;ib<sbs.size();++ib)
	{
		btSoftBody*	psb=sbs[ib];
		nps+=psb->getNodes().size();
		for(int i=0;i<psb->getNodes().size();++i)
		{
			ps+=psb->getNodes()[i].m_x;
		}		
	}
	ps/=nps;
	
	/* Anm			*/ 
	//if(!isIdle())
	//m_animtime=m_clock.getTimeMilliseconds()/1000.f;
	btScalar m_animtime=0.01;
	
	/* Water level	*/ 
	static const btVector3	axis[]={btVector3(1,0,0),
		btVector3(0,1,0),
		btVector3(0,0,1)};
	if(m_softBodyWorldInfo.water_density>0)
	{
		const btVector3	c=	btVector3((btScalar)0.25,(btScalar)0.25,1);
		const btScalar	a=	(btScalar)0.5;
		const btVector3	n=	m_softBodyWorldInfo.water_normal;
		const btVector3	o=	-n*m_softBodyWorldInfo.water_offset;
		const btVector3	x=	cross(n,axis[n.minAxis()]).normalized();
		const btVector3	y=	cross(x,n).normalized();
		const btScalar	s=	25;
		idraw->drawTriangle(o-x*s-y*s,o+x*s-y*s,o+x*s+y*s,c,a);
		idraw->drawTriangle(o-x*s-y*s,o+x*s+y*s,o-x*s+y*s,c,a);
	}



	btScalar m[16];

	if (m_dynamicsWorld)
	{
		int numObjects = m_dynamicsWorld->getNumCollisionObjects();
		btVector3 wireColor(1,0,0);
		for (int i=0;i<numObjects;i++)
		{
			btCollisionObject* colObj = m_dynamicsWorld->getCollisionObjectArray()[i];
			btRigidBody* body = btRigidBody::upcast(colObj);

			if (body && body->getMotionState())
			{
				btDefaultMotionState* myMotionState = (btDefaultMotionState*)body->getMotionState();
				myMotionState->m_graphicsWorldTrans.getOpenGLMatrix(m);
			} else
			{
				colObj->getWorldTransform().getOpenGLMatrix(m);
			}

			btVector3 wireColor(1.f,1.0f,0.5f); //wants deactivation
			if (i & 1)
			{
				wireColor = btVector3(0.f,0.0f,1.f);
			}
			///color differently for active, sleeping, wantsdeactivation states
			if (colObj->getActivationState() == 1) //active
			{
				if (i & 1)
				{
					wireColor += btVector3 (1.f,0.f,0.f);
				} else
				{			
					wireColor += btVector3 (.5f,0.f,0.f);
				}
			}
			if (colObj->getActivationState() == 2) //ISLAND_SLEEPING
			{
				if (i & 1)
				{
					wireColor += btVector3 (0.f,1.f, 0.f);
				} else
				{
					wireColor += btVector3 (0.f,0.5f,0.f);
				}
			}

			m_shapeDrawer.drawOpenGL(m,body, colObj->getCollisionShape(),wireColor,0);
		}
		
	}

	((Ogre_DebugDrawer*)idraw)->flush();
}

/*
void	SoftDemo::keyboardCallback(unsigned char key, int x, int y)
{
	switch(key)
	{
	case	']':	++current_demo;clientResetScene();break;
	case	'[':	--current_demo;clientResetScene();break;
	case	',':	m_raycast=!m_raycast;break;
	case	';':	m_autocam=!m_autocam;break;
	default:		SofeDemo::keyboardCallback(key,x,y);
	}
}
*/

void	SoftDemo::initPhysics()
{
	
	m_collisionShapes.push_back(new btBoxShape (btVector3(200,CUBE_HALF_EXTENTS,200)));

	m_collisionShapes.push_back(new btCylinderShape (btVector3(CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS,CUBE_HALF_EXTENTS)));


	m_dispatcher=0;

	///register some softbody collision algorithms on top of the default btDefaultCollisionConfiguration
	m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();


	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
	m_softBodyWorldInfo.m_dispatcher = m_dispatcher;

	////////////////////////////
	///Register softbody versus softbody collision algorithm


	///Register softbody versus rigidbody collision algorithm


	////////////////////////////

	btVector3 worldAabbMin(-1000,-1000,-1000);
	btVector3 worldAabbMax(1000,1000,1000);

	m_broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);

	m_softBodyWorldInfo.m_broadphase = m_broadphase;

	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();

	m_solver = solver;

	btDiscreteDynamicsWorld* world = new btSoftRigidDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	m_dynamicsWorld = world;


	m_dynamicsWorld->getDispatchInfo().m_enableSPU = true;
	m_dynamicsWorld->setGravity(btVector3(0,-10,0));
	m_softBodyWorldInfo.m_gravity.setValue(0,-10,0);


	m_dynamicsWorld->setDebugDrawer(new Ogre_DebugDrawer());


	btTransform tr;
	tr.setIdentity();
	tr.setOrigin(btVector3(0,-12,0));



	btRigidBody* body = localCreateRigidBody(0.f,tr,m_collisionShapes[0]);


	//	clientResetScene();

	m_softBodyWorldInfo.m_sparsesdf.Initialize();
	clientResetScene();
}






void	SoftDemo::exitPhysics()
{

	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	int i;
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		m_collisionShapes[j] = 0;
		delete shape;
	}

	//delete dynamics world
	delete m_dynamicsWorld;

	//delete solver
	delete m_solver;

	//delete broadphase
	delete m_broadphase;

	//delete dispatcher
	delete m_dispatcher;



	delete m_collisionConfiguration;


}



/*
OgreRigidObject::OgreRigidObject(const char* name, vector3 pos, quater quat)
	:	mBody(0),
		mShape(0),
		mState(0),
		mTriMesh(0)
{
	mNode=RE::ogreRootSceneNode()->createChildSceneNode(name, ToOgre(pos), ToOgre(quat));
}


// -------------------------------------------------------- //
OgreRigidObject::~OgreRigidObject()
{
	if (mShape)
	{
		delete mShape;
		mShape=0;
	}
	if (mState)
	{
		delete mState;
		mState=0;
	}
}

// -------------------------------------------------------- //
void OgreRigidObject::createBody(btDynamicsWorld *world)
{
	const float DynamicBodyMass=1.0;
	const float DynamicBodyRestitution = 0.6f;
	const float DynamicBodyFriction    = 0.6f;

	mShape = new btBoxShape(btVector3(1,1,1));
	mState = new OgreObjectState(this);

	btVector3 localInertiaTensor = btVector3(0,0,0);
	mShape->calculateLocalInertia(DynamicBodyMass, localInertiaTensor);

	mBody = new btRigidBody(DynamicBodyMass, mState, mShape, localInertiaTensor);
	mBody->setRestitution(DynamicBodyRestitution);
	mBody->setFriction(DynamicBodyFriction);

	world->addRigidBody(mBody);
}

// -------------------------------------------------------- //
void OgreRigidObject::createMeshCollider(Ogre::Mesh *ptr, btDynamicsWorld *world)
{
	const float      StaticBodyRestitution  = 0.1f;
	const float      StaticBodyFriction     = 0.8f;

	mState = new OgreObjectState(this);
	mTriMesh = new btTriangleMesh();

	unsigned short subCount = ptr->getNumSubMeshes();
	for (unsigned short i=0; i<subCount; i++)
	{

		// ripped from OgreMesh.cpp
		Ogre::SubMesh *pSubMesh = ptr->getSubMesh(i);

		Ogre::uint16	*pVIndices16 = NULL;
		Ogre::uint32	*pVIndices32 = NULL;

		Ogre::IndexData *indexData = pSubMesh->indexData;
		Ogre::HardwareIndexBufferSharedPtr buffIndex = indexData->indexBuffer;

		bool use32bit = false;
		if (buffIndex->getType() == Ogre::HardwareIndexBuffer::IT_32BIT)
		{
		    pVIndices32 = static_cast<Ogre::uint32*>(
				buffIndex->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
			use32bit = true;
		}
		else
		{
		    pVIndices16 = static_cast<Ogre::uint16*>(
				buffIndex->lock(Ogre::HardwareBuffer::HBL_READ_ONLY));
		}

		Ogre::VertexData *usedVertexData;
		usedVertexData = pSubMesh->vertexData;

		Ogre::VertexDeclaration *vDecl = usedVertexData->vertexDeclaration;
		Ogre::VertexBufferBinding *vBind = usedVertexData->vertexBufferBinding;

		const Ogre::VertexElement *elemVPos = vDecl->findElementBySemantic(Ogre::VES_POSITION);
		Ogre::HardwareVertexBufferSharedPtr srcBuf = vBind->getBuffer(elemVPos->getSource());

		unsigned char *pSrcBase = static_cast<unsigned char*>(
                srcBuf->lock(Ogre::HardwareBuffer::HBL_NORMAL));

		size_t numFaces = indexData->indexCount / 3;

		btVector3 vertexPos[3];
		Ogre::uint32 vertInd[3];
		float	*pVPos;

		for (size_t n = 0; n < numFaces; ++n)
		{

			int i;
			for (i = 0; i < 3; ++i)
			{
				// get indexes of vertices that form a polygon in the position buffer
				if (use32bit)
				{
					vertInd[i] = *pVIndices32++;
				}
				else
				{
					vertInd[i] = *pVIndices16++;
				}
			
				// get the vertices positions from the position buffer
				unsigned char* vBase = pSrcBase + (srcBuf->getVertexSize() * vertInd[i]);
				elemVPos->baseVertexPointerToElement(vBase, &pVPos);
				vertexPos[i][0] = pVPos[0];
				vertexPos[i][1] = pVPos[1];
				vertexPos[i][2] = pVPos[2];
			}
			mTriMesh->addTriangle(vertexPos[0], vertexPos[1], vertexPos[2]);
		}
	}

	mShape = new btBvhTriangleMeshShape(mTriMesh, false);
	mBody = new btRigidBody(0.0, mState, mShape);
	mBody->setRestitution(StaticBodyRestitution);
	mBody->setFriction(StaticBodyFriction);
	world->addRigidBody(mBody);
}



// -------------------------------------------------------- //
OgreObjectState::OgreObjectState(OgreRigidObject *parent)
	:	mObject(parent)
{
}



// -------------------------------------------------------- //
OgreObjectState::~OgreObjectState()
{
}


// -------------------------------------------------------- //
void OgreObjectState::getWorldTransform(btTransform& worldTrans ) const
{
	if (mObject)
	{
		Ogre::Vector3 pos = mObject->mNode->getWorldPosition();
		Ogre::Quaternion quat = mObject->mNode->getWorldOrientation();

		worldTrans.setOrigin(btVector3(pos.x, pos.y, pos.z));
		worldTrans.setRotation(btQuaternion(quat.x, quat.y, quat.z, quat.w));
	}
}


// -------------------------------------------------------- //
void OgreObjectState::setWorldTransform(const btTransform& worldTrans)
{
	if (mObject)
	{
		btVector3 pos = worldTrans.getOrigin();
		btQuaternion quat = worldTrans.getRotation();

		mObject->mNode->setPosition(pos[0],pos[1], pos[2]);
		mObject->mNode->setOrientation(quat.getW(),quat.getX(), quat.getY(), quat.getZ());
	}
}
*/

//////////////////////////////////////////////////
// taesoo kwon

/*#include "../MainLib/WrapperLua/BaselibLUA.h"
#include "../MainLib/WrapperLua/MainlibLUA.h"
#include <luabind/luabind.hpp>
#include <luabind/operator.hpp>
*/

#include "luna_flexiblebody.h"
void Register_baselib(lua_State*L);
void Register_mainlib(lua_State*L);
void Register_physicsbind(lua_State*L);
void Register_classificationLib_bind(lua_State*L);
void Register_flexiblebody(lua_State*L);

SoftDemoWin::SoftDemoWin(int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer)
: FlLayout(0,0,w,h),
m_motionPanel(mp),
mRenderer(renderer),
mDemo(NULL)
{
	create("Button", "Start", "Start");

	create("Choice", "choose demo", "choose demo");
	menu(0)->size(17);
	menu(0)->item(0, "Cloth");
	menu(0)->item(1, "Pressure");
	menu(0)->item(2, "Volume");
	menu(0)->item(3, "Ropes");
	menu(0)->item(4, "RopeAttach");
	menu(0)->item(5, "ClothAttach");
	menu(0)->item(6, "Sticks");	
	menu(0)->item(7, "Collide");
	menu(0)->item(8, "Collide2");
	menu(0)->item(9, "Collide3");
	menu(0)->item(10, "Impact");
	menu(0)->item(11, "Aero");
	menu(0)->item(12, "Friction");	
	menu(0)->item(13, "Torus");
	menu(0)->item(14, "TorusMatch");
	menu(0)->item(15, "Bunny");
	menu(0)->item(16, "BunnyMatch");
	menu(0)->value(0);

	updateLayout();
	renderer.ogreRenderer().addFrameMoveObject(this);

}

SoftDemoWin ::~SoftDemoWin (void)
{
}

void SoftDemoWin::onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData)
{
	if(w.mId=="Start")
	{
		delete mDemo;
		mDemo=new SoftDemo();
		mDemo->initPhysics();

		TString default_script="../src/lua/flexiblebody.lua";


		LUAwrapper* L=new LUAwrapper();

		// export Mainlib classes and functions for use in LUA script.
		//addMainlibModule(L->L);

		//L->dofile(default_script);
		Register_baselib(L->L);
		Register_mainlib(L->L);
		Register_physicsbind(L->L);
		Register_classificationLib_bind(L->L);
		Register_flexiblebody(L->L);
		//L->setRef<SimulatorImplicit>("world", win.mSimulator);
		//L->setRef<FlexibleBodyImplicitSimpleWin>("win", win);

		L->dofile(default_script);

		delete L;

	}
	else if(w.mId=="choose demo")
	{
		current_demo=w.menu()->value();
	}
}

// FrameMoveObject
int SoftDemoWin::FrameMove(float fElapsedTime)
{
	if(mDemo) 
		mDemo->clientMoveAndDisplay();
	return 1;
}


FlLayout* createSoftDemoWin(int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer)
{
	return new SoftDemoWin(x,y,w,h,mp,renderer);
}
