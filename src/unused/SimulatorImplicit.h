#pragma once

#include "../../BaseLib/math/Metric.h"
//#include "../../BaseLib/motion/TransitionCost.h"
#include "../../BaseLib/motion/FullbodyIK.h"
#include "../../BaseLib/utility/scoped_ptr.h"
#include "../../BaseLib/math/Operator.h"
#include "../../MainLib/OgreFltk/Mesh.h"
#include "../../MainLib/OgreFltk/MotionPanel.h"
#include "Ogre_ShapeDrawer.h"
#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld2.h"
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
#include "BulletSoftBody/btSoftBody2Helpers.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "BulletSoftBody/btSoftBody2.h"
//#include "../MainLib/Ogre/PLDPrimCustumSkin.h"
#include "bulletTools.h"

class SimulatorImplicit
{
public:

	// softbody parameters
	double _mass;
	double _stiffness;
	double _springDamp;
	double _kDF;
	double _kDFvelThr;
	int _bendingConstraints1;
	double _bendingStiffness;
	int _integrateMethod;
	bool _drawContactForce;
	bool _debugDrawWorld;
	bool _drawSimulatedMesh;
	
	//this is the most important class
	btDynamicsWorld*		m_dynamicsWorld;

	btSoftBody2::btSoftBody2WorldInfo	m_softBodyWorldInfo;



	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>		m_collisionShapes;
	btRigidBody* mTerrain;
	btConstraintSolver*	m_solver;

	btDefaultCollisionConfiguration* m_collisionConfiguration;

	Ogre_ShapeDrawer m_shapeDrawer;

	//////////////////////////////////////////////////////////////
	// My data
	//////////////////////////////////////////////////////////////
	Ogre_DebugDrawer* m_debugDrawer;
	OBJloader::Mesh mSimulatedMesh;
	OBJloader::MeshToEntity* mSimulatedOgreMesh;
	
	boost::shared_ptr<Ogre::SceneNode> mNode;

	const btSoftRigidDynamicsWorld2*	getSoftDynamicsWorld() const
	{
		return (btSoftRigidDynamicsWorld2*) m_dynamicsWorld;
	}

	btSoftRigidDynamicsWorld2*	getSoftDynamicsWorld()
	{
		return (btSoftRigidDynamicsWorld2*) m_dynamicsWorld;
	}

	btRigidBody*	localCreateRigidBody(float mass, const btTransform& startTransform,btCollisionShape* shape);

	SimulatorImplicit();

	void start();
	void startWorld();
	void exitWorld();
	bool isWorldValid()	{ return m_dynamicsWorld;}
	
	void setGravity(vector3 const& gravity);

	
	void createSoftBody(OBJloader::Mesh const& targetMesh);
	btSoftBody2* getSoftBody()	{return getSoftDynamicsWorld()->getSoftBodyArray()[0];}

	void setParameter(const char* _what, double value);
	// adjustWhat=="dynamicFrictionCoef" or "stiffness"	
	void adjustSoftBodyParam(const char* _adjustWhat, const char* selectionFileName, double value);
	void adjustSoftBodyVertexParam(const char* _adjustWhat, boolN const& selectedVertices, double value);
	void adjustSoftBodyEdgeParam(const char* _adjustWhat, boolN const& selectedEdges, double value);

	void extractLinkLength(OBJloader::Mesh const& targetMesh, vectorn& length);
	void setRestLength(vectorn const& length);
	void changeSoftBodyRestLength(OBJloader::Mesh const& targetMesh);

	void addExternalForce(intvectorn const& indices, vector3N const& forces);
	void createFloor(double floorHeight, vector3 const& size);
	void createTerrain(const char* filename, vector3 const& pos, int sizeX, int sizeY, m_real width, m_real height, m_real heightMax);

	void renderme();

	void addSimulatorModule(void* lua_state);
private:
	void cleanupScene();	
	btSoftBody2*		_CreateFromTriMesh(btSoftBody2::btSoftBody2WorldInfo& worldInfo, OBJloader::Mesh const& mesh, OBJloader::EdgeConnectivity const& edges, double stiffness);

};
