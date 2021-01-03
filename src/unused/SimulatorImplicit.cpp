
#include "stdafx.h"
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>
#include "softbody2/Physics.h"
#include "SimulatorImplicit.h"
#include "../../MainLib/OgreFltk/RE.h"
#include "../../MainLib/OgreFltk/pldprimskin.h"

const int maxProxies = 32766;
const int maxOverlap = 65535;
const int maxNumObjects = 32760;


//
btSoftBody2*		SimulatorImplicit::_CreateFromTriMesh(btSoftBody2::btSoftBody2WorldInfo& worldInfo, OBJloader::Mesh const& mesh, OBJloader::EdgeConnectivity const& edges, double stiffness)
{
	btAlignedObjectArray<btVector3>	vtx;
	
	int nv=mesh.numVertex();
	vtx.resize(nv);
	for(int i=0;i<nv; i++)
	{
		vtx[i]=ToBullet(mesh.getVertex(i));
	}
	btSoftBody2*		psb=new btSoftBody2(&worldInfo, vtx.size(), &vtx[0], 0, _springDamp, _integrateMethod);
	
	OBJloader::EdgeConnectivity::edgeT e;
   TUGL_for_all_edges(e, edges.mMeshGraph)
   {
		psb->appendLink(e.v1().index(), e.v2().index(), stiffness, btSoftBody2::eLType::Structural);
	}

	for(int i=0; i<mesh.numFace(); i++)
	{	
		psb->appendFace(mesh.getFace(i).vi(0), mesh.getFace(i).vi(1), mesh.getFace(i).vi(2));
	}
	
	psb->generateBendingConstraints(_bendingConstraints1,_bendingStiffness);
	
	const bool randomize=false;	// set false for debugging.
	if(randomize)
		psb->randomizeConstraints();
	return(psb);
}

void SimulatorImplicit::extractLinkLength(OBJloader::Mesh const& targetMesh, vectorn& length)
{
	btSoftBody2*	psb	= getSoftDynamicsWorld() ->getSoftBodyArray()[0];

	int ni=psb->getLinks().size();
	length.setSize(ni);

	/* Links		*/ 
	for(int i=0;i<ni;++i)
	{
		btSoftBody2::Link&	l=psb->getLinks()[i];
		length[i]	=	targetMesh.getVertex(l.m_n[0]->m_index).distance(targetMesh.getVertex(l.m_n[1]->m_index));
	}
}

void SimulatorImplicit::setRestLength(vectorn const& length)
{
	btSoftBody2*	psb	= getSoftDynamicsWorld() ->getSoftBodyArray()[0];

	/* Links		*/ 
	for(int i=0,ni=psb->getLinks().size();i<ni;++i)
	{
		btSoftBody2::Link&	l=psb->getLinks()[i];
		l.m_pSpring->m_RestDistance	=	length[i];
		l.m_c0	=	l.m_n[0]->im()+l.m_n[1]->im();
		l.m_c1	=	l.m_pSpring->m_RestDistance*l.m_pSpring->m_RestDistance;
		l.m_pSpring->m_MaxDistance=l.m_pSpring->m_RestDistance*1.1;
	}
}
void SimulatorImplicit::addExternalForce(intvectorn const& indices, vector3N const& forces)
{
	btSoftBody2*	psb	= getSoftDynamicsWorld() ->getSoftBodyArray()[0];
	psb->addExternalForce(indices, forces);
}

void SimulatorImplicit::changeSoftBodyRestLength(OBJloader::Mesh const& targetMesh)
{

	btSoftBody2*	psb	= getSoftDynamicsWorld() ->getSoftBodyArray()[0];

	/* Links		*/ 
	for(int i=0,ni=psb->getLinks().size();i<ni;++i)
	{
		btSoftBody2::Link&	l=psb->getLinks()[i];
		l.m_pSpring->m_RestDistance	=	targetMesh.getVertex(l.m_n[0]->m_index).distance(targetMesh.getVertex(l.m_n[1]->m_index));
		l.m_c0	=	l.m_n[0]->im()+l.m_n[1]->im();
		l.m_c1	=	l.m_pSpring->m_RestDistance*l.m_pSpring->m_RestDistance;

		l.m_pSpring->m_MaxDistance=l.m_pSpring->m_RestDistance*1.1;
	}
}



void SimulatorImplicit::createFloor(double floorHeight, vector3 const& size)
{
	btTransform tr;
	tr.setIdentity();
	tr.setOrigin(btVector3(0,floorHeight,0));

	tr.setOrigin(btVector3(0,floorHeight,0));
	tr.setRotation(ToBullet(quater(TO_RADIAN(10), vector3(0,0,1))));

	// 바닥판.
	m_collisionShapes.push_back(new btBoxShape (ToBullet(size)));
	btRigidBody* body = localCreateRigidBody(0.f,tr,m_collisionShapes[0]);
}


void SimulatorImplicit::createTerrain(const char* filename, vector3 const& pos, int sizeX, int sizeY, m_real width, m_real height, m_real heightMax)
{
	btTransform tr;
	tr.setIdentity();
	tr.setOrigin(ToBullet(pos));

	// 바닥판.
	m_collisionShapes.push_back(createTerrainCollisionShape(filename, sizeX, sizeY, width, height, heightMax));
	btRigidBody* body = localCreateRigidBody(0.f,tr,m_collisionShapes[0]);
	mTerrain=body;
}

btRigidBody*	SimulatorImplicit::localCreateRigidBody(float mass, const btTransform& startTransform,btCollisionShape* shape)
{
	btRigidBody* body=btLocalCreateRigidBody(mass, startTransform, shape, mNode.get());
	m_dynamicsWorld->addRigidBody(body);	
	return body;
}





void	SimulatorImplicit::renderme()
{
	btIDebugDraw*	idraw=m_dynamicsWorld->getDebugDrawer();
	
	if(_debugDrawWorld)
		m_dynamicsWorld->debugDrawWorld();
	
	/* Bodies		*/ 
	
	btSoftBody2Array&	sbs=getSoftDynamicsWorld()->getSoftBodyArray();
	ASSERT(sbs.size()==1);
	btSoftBody2*	psb=sbs[0];
	for(int i=0;i<psb->getNodes().size();++i)
		mSimulatedMesh.getVertex(i)=ToBase(psb->getNodes()[i].x());
	mSimulatedMesh.calculateVertexNormal();
	mSimulatedOgreMesh->updatePositionsAndNormals();
	mNode->_updateBounds();
	//mNode->showBoundingBox(true);
	//mSimulatedOgreMesh->updatePositions();
	//mSimulatedOgreMesh->mTargetMesh->buildEdgeList();

	btScalar m[16];

	if (m_dynamicsWorld)
	{
		if(_drawContactForce)
		{
			btSoftBody2*	psb	= getSoftDynamicsWorld() ->getSoftBodyArray()[0];
			for(int i=0; i<psb->m_pSystem->m_iParticles; i++)
			{
				btVector3 contactForce;
				//contactForce.setX(psb->m_pSystem->m_vContactForce[i].x);
				//contactForce.setY(psb->m_pSystem->m_vContactForce[i].y);
				//contactForce.setZ(psb->m_pSystem->m_vContactForce[i].z);

				//printf("%f %f %f\n", contactForce.x() , contactForce.y(), contactForce.z());
				if(contactForce.length()>0.1)
				{	
					btVector3 from=psb->getNodes()[i].x();
					btVector3 to=from+contactForce*10;
					
					idraw->drawLine(from,to,btVector3(1,1,1));
				}
			}
		}

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
	m_softBodyWorldInfo.m_sparsesdf.GarbageCollect();
}



void	SimulatorImplicit::cleanupScene()
{
	/* Clean up	*/ 
	//remove the rigid and soft bodies from the dynamics world and delete them
	for(int i=m_dynamicsWorld->getNumCollisionObjects()-1;i>=0;i--)
	{
		btCollisionObject*	obj=m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody*		body=btRigidBody::upcast(obj);
		if(body&&body->getMotionState())
		{
			delete body->getMotionState();
		}
		btSoftBody2* softBody = btSoftBody2::upcast(obj);
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






void	SimulatorImplicit::exitWorld()
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
	mTerrain=NULL;
	delete m_solver;

	m_softBodyWorldInfo.cleanup();

	delete m_debugDrawer;

	delete m_collisionConfiguration;
}



SimulatorImplicit::SimulatorImplicit()
:m_dynamicsWorld(NULL),
mTerrain(NULL),
_drawContactForce(true),
_debugDrawWorld(false),
_drawSimulatedMesh(true)
{
}


void SimulatorImplicit::startWorld()
{
	if(!mNode)
	{
		mNode.reset(RE::createSceneNode(RE::generateUniqueName()), (void(*)(Ogre::SceneNode* node))RE::removeEntity);			
	}

	if(m_dynamicsWorld)
	{
		exitWorld();
	}

	///register some softbody collision algorithms on top of the default btDefaultCollisionConfiguration
	m_collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
	m_softBodyWorldInfo.m_dispatcher = new	btCollisionDispatcher(m_collisionConfiguration);
	

	btVector3 worldAabbMin(-1000,-1000,-1000);
	btVector3 worldAabbMax(1000,1000,1000);

	m_softBodyWorldInfo.m_broadphase = new btAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);

	

	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver();

	m_solver = solver;

	btDiscreteDynamicsWorld* world = new btSoftRigidDynamicsWorld2(
		m_softBodyWorldInfo.m_dispatcher ,
		m_softBodyWorldInfo.m_broadphase ,m_solver,m_collisionConfiguration);
	m_dynamicsWorld = world;


	m_dynamicsWorld->getDispatchInfo().m_enableSPU = true;
	m_debugDrawer=new Ogre_DebugDrawer();
	m_dynamicsWorld->setDebugDrawer(m_debugDrawer);

}

void SimulatorImplicit::setGravity(vector3 const& gravity)
{
	m_dynamicsWorld->setGravity(ToBullet(gravity));
		
	// softbody
	m_softBodyWorldInfo.m_sparsesdf.Initialize();
	m_softBodyWorldInfo.m_gravity=ToBullet(gravity);
}

void SimulatorImplicit::createSoftBody(OBJloader::Mesh const& targetMesh)
{
	mSimulatedMesh=targetMesh;
	//OBJloader::GetMeshAnimation::_create(mSkin, mSimulatedMesh,0);
	mSimulatedMesh.calculateVertexNormal();
	OBJloader::MeshToEntity::Option option;
	option.buildEdgeList=true;
	option.useTexCoord=false;
	mSimulatedOgreMesh=new OBJloader::MeshToEntity(mSimulatedMesh, "simulatedMesh", option);
	mNode->attachObject(mSimulatedOgreMesh->createEntity("simulatedMeshEntity", "white"));
	//mNode->attachObject(mSimulatedOgreMesh->createEntity("simulatedMeshEntity", "lightgrey_transparent"));

	mSimulatedOgreMesh->getLastCreatedEntity()->setVisible(_drawSimulatedMesh);
//#define _USE_BUNNY
#ifdef _USE_BUNNY
	btSoftBody2*	psb=btSoftBody2Helpers::CreateFromTriMesh(m_softBodyWorldInfo,			gVerticesBunny,
		&gIndicesBunny[0][0],
		BUNNY_NUM_TRIANGLES);
#else
	OBJloader::EdgeConnectivity edges(targetMesh);

	btSoftBody2* psb=_CreateFromTriMesh(m_softBodyWorldInfo, targetMesh, edges, _stiffness);
#endif

	
	//psb->scale(btVector3(6,6,6));
	//psb->transform(btTransform(btQuaternion(0,0,0,1), btVector3(0,80,0)));
	psb->setTotalMass(_mass,true);

	psb->m_pSystem->SetupMatrices();
	
	Physics_GravityForce *pGravity = new Physics_GravityForce( ToBase( psb->m_worldInfo->m_gravity));
	
	psb->m_pSystem->AddForce(*pGravity);

	//psb->createSystem();
	//psb->m_pSystem->m_cfg.mu=FlexibleBodyImplicit::kDF;

	for(int i=0; i<psb->m_pSystem->m_cfg.m_dynamicFrictionCoef.size(); i++)
	{
		psb->m_pSystem->m_cfg.m_dynamicFrictionCoef[i]=_kDF;
	}
	psb->m_pSystem->m_cfg.DFthresholdVel=_kDFvelThr;


	getSoftDynamicsWorld()->addSoftBody(psb);
}


void SimulatorImplicit::adjustSoftBodyParam(const char* _adjustWhat, const char* selectionFileName, double value)
{

	bitvectorn selectedVertices, selectedEdges;
	BinaryFile bf(false, selectionFileName);
	bf.unpack(selectedVertices);
	bf.unpack(selectedEdges);
	bf.close();
	adjustSoftBodyVertexParam( _adjustWhat,  selectedVertices, value);
}

void SimulatorImplicit::adjustSoftBodyEdgeParam(const char* _adjustWhat, boolN const& selectedEdges, double value)
{
	TString adjustWhat(_adjustWhat);
	btSoftBody2* psb=getSoftBody();
}

void SimulatorImplicit::adjustSoftBodyVertexParam(const char* _adjustWhat, boolN const& selectedVertices, double value)
{
	TString adjustWhat(_adjustWhat);
	btSoftBody2* psb=getSoftBody();
	if(adjustWhat=="dynamicFrictionCoef")
	{		
		for(int i=0; i<psb->m_pSystem->m_cfg.m_dynamicFrictionCoef.size(); i++)
		{
			if(selectedVertices[i])
				psb->m_pSystem->m_cfg.m_dynamicFrictionCoef[i]=value;
		}		
	}
	else if(adjustWhat=="stiffness")
	{
		for(int i=0,ni=psb->getLinks().size();i<ni;++i)
		{
			btSoftBody2::Link&	l=psb->getLinks()[i];
			int index1=l.m_n[0]->m_index;
			int index2=l.m_n[1]->m_index;
			if(selectedVertices[index1] && selectedVertices[index2])	
			{
				l.m_pSpring->m_kSpring=value;
			}
		}
	}
}

void SimulatorImplicit::setParameter(const char* _what, double value)
{
	TString what(_what);

	if(what=="mass")
		_mass=value;
	else if(what=="stiffness")
		_stiffness=value;
	else if(what=="springDamp")
		_springDamp=value;
	else if(what=="kDF")
		_kDF=value;
	else if(what=="kDFvelThr")
		_kDFvelThr=value;
	else if(what=="bendingConstraints1")
		_bendingConstraints1=value;
	else if(what=="bendingStiffness")
		_bendingStiffness=value;
	else if(what=="integrateMethod")
		_integrateMethod=value;
	else if(what=="debugDrawWorld")
		_debugDrawWorld=value!=0.0;
	else if(what=="drawContactForce")
		_drawContactForce=value!=0.0;
	else if(what=="drawSimulatedMesh")
		_drawSimulatedMesh=value!=0.0;
	else 
	{
		btSoftBody2* psb=getSoftBody();
		ASSERT(psb->m_pSystem);
		if(what=="penaltyStiffness")
		{
			psb->m_pSystem->m_cfg.penaltyStiffness=value;
		}
		else if(what=="penaltyDampness")
		{
			psb->m_pSystem->m_cfg.penaltyDampness=value;
		}
		else if(what=="usePenaltyMethod")
		{
			if(value==1.0)
				psb->m_pSystem->m_cfg.contactMethod=Physics_ParticleSystem  ::Config::PENALTY_METHOD;
		}
		else if(what=="contactMethod")
		{
			psb->m_pSystem->m_cfg.contactMethod=(int)value;
		}
		else if(what=="penaltyMuScale")
		{
			psb->m_pSystem->m_cfg.penaltyMuScale=value;
		}
		else
			Msg::error("unknown parameter %s", _what);
	}
}
