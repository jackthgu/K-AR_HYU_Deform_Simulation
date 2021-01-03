#include "stdafx.h"
#include "FlexibleBodyWin.h"
//#include "../MainLib/Ogre/PLDPrimCustumSkin.h"
#include "bulletTools.h"

namespace FlexibleBody
{
	double gravity=-10;
	double mass=50;
	double scale_factor=0.1;
	double air_density=1.2;
	int bendingConstraints1=2;
	double bendingConstraints2=0.5;
	int iterations=2;
	double simulationFrameRate;
	double kDF=0.5;
	double kST=1.0;
}
const int maxProxies = 32766;
const int maxOverlap = 65535;
const int maxNumObjects = 32760;

#define EXTRA_HEIGHT -10.f

#include "../../MainLib/WrapperLua/luna_baselib.h"
#include "../../MainLib/WrapperLua/luna_mainlib.h"
#include "FELearning.h"
#include "FlexibleBodyImplicitSimpleWin.h"
#include "FlexibleBodyImplicitWin.h"
#include "FlexibleBodyWin.h"
#include "luna_flexiblebody.h"
void Register_baselib(lua_State*L);
void Register_mainlib(lua_State*L);
void Register_physicsbind(lua_State*L);
void Register_classificationLib_bind(lua_State*L);
void Register_flexiblebody(lua_State*L);

void FlexibleBodyWin ::initWorld()
{
	m_dispatcher=0;

	///register some softbody collision algorithms on top of the default btDefaultCollisionConfiguration
	m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
	m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
	m_softBodyWorldInfo.m_dispatcher = m_dispatcher;

	btVector3 worldAabbMin(-1000,-1000,-1000);
	btVector3 worldAabbMax(1000,1000,1000);

	m_broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);

	m_softBodyWorldInfo.m_broadphase = m_broadphase;

	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();

	m_solver = solver;

	btDiscreteDynamicsWorld* world = new btSoftRigidDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	m_dynamicsWorld = world;


	m_dynamicsWorld->getDispatchInfo().m_enableSPU = true;
	
	m_debugDrawer=new Ogre_DebugDrawer();
	m_dynamicsWorld->setDebugDrawer(m_debugDrawer);

}

void FlexibleBodyWin ::createFloor(double floorHeight, vector3 const& size)
{
	btTransform tr;
	tr.setIdentity();
	

	// 바닥판.
	m_collisionShapes.push_back(new btBoxShape (ToBullet(size)));
	btRigidBody* body = localCreateRigidBody(0.f,tr,m_collisionShapes[0]);
}


void FlexibleBodyWin::createTerrain(const char* filename, vector3 const& pos, int sizeX, int sizeY, m_real width, m_real height, m_real heightMax)
{
	
	btTransform tr;
	tr.setIdentity();
	tr.setOrigin(ToBullet(pos));

	// 바닥판.
	m_collisionShapes.push_back(createTerrainCollisionShape(filename, sizeX, sizeY, width, height, heightMax));
	btRigidBody* body = localCreateRigidBody(0.f,tr,m_collisionShapes[0]);
}


btRigidBody*	FlexibleBodyWin::localCreateRigidBody(float mass, const btTransform& startTransform,btCollisionShape* shape)
{
	btRigidBody* body=btLocalCreateRigidBody(mass, startTransform, shape, RE::ogreRootSceneNode());
	m_dynamicsWorld->addRigidBody(body);	
	return body;
}


//
btSoftBody*		_CreateFromTriMesh(btSoftBody::btSoftBodyWorldInfo& worldInfo, OBJloader::Mesh const& mesh, OBJloader::EdgeConnectivity const& edges)
{
	double scale_factor=FlexibleBody::scale_factor;
	btAlignedObjectArray<btVector3>	vtx;
	
	int nv=mesh.numVertex();
	vtx.resize(nv);
	for(int i=0;i<nv; i++)
	{
		vtx[i]=ToBullet(mesh.getVertex(i)*scale_factor);
	}
	btSoftBody*		psb=new btSoftBody(&worldInfo,vtx.size(),&vtx[0],0);
	
	OBJloader::EdgeConnectivity::edgeT e;
   TUGL_for_all_edges(e, edges.mMeshGraph)
   {
	   psb->appendLink(e.v1().index(), e.v2().index(), FlexibleBody::kST, btSoftBody::eLType::Structural);
	}

	for(int i=0; i<mesh.numFace(); i++)
	{	
		psb->appendFace(mesh.getFace(i).vi(0), mesh.getFace(i).vi(1), mesh.getFace(i).vi(2));
	}
	psb->randomizeConstraints();
	return(psb);
}

static void			UpdateConstantsTaesoo2(btSoftBody* psb, OBJloader::Mesh* mesh)
{
	/* Links		*/ 
	for(int i=0,ni=psb->getLinks().size();i<ni;++i)
	{
		btSoftBody::Link&	l=psb->getLinks()[i];
		l.m_rl	=	mesh->getVertex(l.m_n[0]->m_index).distance(mesh->getVertex(l.m_n[1]->m_index))*FlexibleBody::scale_factor;
		l.m_c0	=	l.m_n[0]->m_im+l.m_n[1]->m_im;
		l.m_c1	=	l.m_rl*l.m_rl;
	}

}

////////////////////////////////////



// FrameMoveObject
int FlexibleBodyWin::FrameMove(float fElapsedTime)
{
	if(!m_dynamicsWorld ) return 0;
	//float dt = getDeltaTimeMicroseconds() * 0.000001f;

	if (m_dynamicsWorld)
	{

		if(mMesh)
		{
			mMeshAnimation->setPose(mMotion.pose(m_motionPanel.motionWin()->getCurrFrame()));
			mMeshAnimation->retrieveAnimatedMesh(*mMesh);

			btSoftBody*	psb	= getSoftDynamicsWorld() ->getSoftBodyArray()[0];

			//psb->updateConstants.

			UpdateConstantsTaesoo2(psb, mMesh);
		}

//#define FIXED_STEP
#ifdef FIXED_STEP
		m_dynamicsWorld->stepSimulation(dt=1.0f/60.f,0);

#else
		// 120일때는 이상하게 떨리고, 420정도로 커지면 다시 안정적이 된다. 신기한건 simulationFrameRate가 커지면 훨씬 더 stiff해진다는거.
		float renderingFrameRate=30;
		int maxSimSubSteps = 100;

		m_dynamicsWorld->stepSimulation(1.0/renderingFrameRate,maxSimSubSteps, 1.0/FlexibleBody::simulationFrameRate);		
		m_softBodyWorldInfo.m_sparsesdf.GarbageCollect();

#endif
	
		
	}
	
	renderme(); 


	return 1;
}

void	FlexibleBodyWin::cleanupScene()
{
	/* Clean up	*/ 
	//remove the rigid and soft bodies from the dynamics world and delete them
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
}






void	FlexibleBodyWin::exitPhysics()
{
	if(!m_dynamicsWorld) return;

	//cleanup in the reverse order of creation/initialization

	cleanupScene();

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		m_collisionShapes[j] = 0;
		delete shape;
	}

	m_collisionShapes.resize(0);

	//delete dynamics world
	delete m_dynamicsWorld;
	m_dynamicsWorld =NULL;
	delete m_solver;
	delete m_broadphase;
	delete m_dispatcher;

	delete m_debugDrawer;

	delete m_collisionConfiguration;
}



void	FlexibleBodyWin::renderme()
{
	btIDebugDraw*	idraw=m_dynamicsWorld->getDebugDrawer();
	
	//m_dynamicsWorld->debugDrawWorld();
	
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


FlexibleBodyWin::FlexibleBodyWin(int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer)
: FlLayout(0,0,w,h),
m_motionPanel(mp),
mRenderer(renderer),
m_dynamicsWorld(NULL),
mMesh(NULL)
//,mSkin(NULL)
{
	create("Button", "Start", "Start");

	updateLayout();
	renderer.ogreRenderer().addFrameMoveObject(this);

}

FlexibleBodyWin ::~FlexibleBodyWin (void)
{
	exitPhysics();

	//RE::remove(mSkin);
}





void FlexibleBodyWin::onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData)
{
	if(w.mId=="Start")
	{
		if(m_dynamicsWorld)
		{
			exitPhysics();

		}
		else
		{
			mMotion.Init(RE::motionLoader("trc/iguana_from248.bvh"));
			mMotion.SetIdentifier("iguana");		
			
			//mSkin=(PLDPrimCustumSkin*)RE::createCustumSkin(mMotion);

			mMesh=new OBJloader::Mesh();
			mMeshAnimation=new SkinnedMeshLoader("iguana_physics.mesh");
			mMeshAnimation->mMesh.mergeDuplicateVertices();
			mMeshAnimation->mMesh.mesh.calculateVertexNormal();
			mMeshAnimation->mMesh.reorderVertices();	// to match iguana_bound.obj

			mMeshAnimation->sortBones(mMotion.skeleton());
			
			*mMesh=mMeshAnimation->mMesh.mesh;

			/*
			mMeshAnimation.create(mMotion, "iguana_physics.mesh", *mMesh, 30);
*/
			/*
			RE::renderer().mRoot->renderOneFrame();
			
			mMeshAnimation.update(*mMesh, 0);*/
			//mMesh->firstInit();

			//RE::ogreRootSceneNode()->attachObject(mMesh);

			//m_motionPanel.motionWin()->addSkin(mSkin);
		}

		TString default_script="../src/lua/flexiblebody.lua";

		LUAwrapper* L=new LUAwrapper();

		// export Mainlib classes and functions for use in LUA script.
		Register_baselib(L->L);
		Register_mainlib(L->L);
		Register_physicsbind(L->L);
		Register_classificationLib_bind(L->L);
		Register_flexiblebody(L->L);

		// export member functions for use in LUA script.
		// to understand the following codes, please refer to "luabind" manual.
		L->set<FlexibleBodyWin>("win", this);

		initWorld();

		L->dofile(default_script);

		FlexibleBody::gravity=L->getDouble("gravity");
		FlexibleBody::mass=L->getDouble("mass");
		FlexibleBody::scale_factor=L->getDouble("scale_factor");
		FlexibleBody::air_density=L->getDouble("air_density");
		FlexibleBody::bendingConstraints1=L->getInt("bendingConstraints1");
		FlexibleBody::bendingConstraints2=L->getDouble("bendingConstraints2");;
		FlexibleBody::iterations=L->getInt("iterations");
		FlexibleBody::kDF=L->getDouble("kDF");
		FlexibleBody::simulationFrameRate=L->getDouble("simulationFrameRate");
		delete L;


		m_dynamicsWorld->setGravity(btVector3(0,FlexibleBody::gravity,0));
		

		
		// softbody
		m_softBodyWorldInfo.m_sparsesdf.Initialize();
		m_softBodyWorldInfo.air_density		=	(btScalar)FlexibleBody::air_density;
		m_softBodyWorldInfo.water_density	=	0;
		m_softBodyWorldInfo.water_offset	=	0;
		m_softBodyWorldInfo.water_normal	=	btVector3(0,0,0);
		m_softBodyWorldInfo.m_gravity.setValue(0,FlexibleBody::gravity,0);

	//#define _USE_BUNNY
	#ifdef _USE_BUNNY
		btSoftBody*	psb=btSoftBodyHelpers::CreateFromTriMesh(m_softBodyWorldInfo,			gVerticesBunny,
			&gIndicesBunny[0][0],
			BUNNY_NUM_TRIANGLES);
	#else
		OBJloader::EdgeConnectivity e(*mMesh);
		btSoftBody* psb=_CreateFromTriMesh(m_softBodyWorldInfo, *mMesh, e);
	#endif

		psb->generateBendingConstraints(FlexibleBody::bendingConstraints1,FlexibleBody::bendingConstraints2);
		psb->m_cfg.iterations	=	FlexibleBody::iterations;
		psb->m_cfg.kDF			=	FlexibleBody::kDF;
		psb->randomizeConstraints();
		//psb->scale(btVector3(6,6,6));
		psb->setTotalMass(FlexibleBody::mass,true);

		getSoftDynamicsWorld()->addSoftBody(psb);

	}
}


FlLayout* createFlexibleBodyWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer)
{
	return new FlexibleBodyWin(x,y,w,h,mp, renderer);
}

