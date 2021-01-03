#ifndef FLEXZIBODYWIN_H
#define FLEXZIBODYWIN_H
#pragma once

class MotionPanel;
class FltkRenderer;

#include <OgreBone.h>
#include <OgreNode.h>
#include "MainLib/OgreFltk/Line3D.h"
#include "MainLib/OgreFltk/pldprimskin.h"
#include "MainLib/OgreFltk/OgreMotionLoader.h"
#include "MainLib/OgreFltk/FlLayout.h"
#include "MainLib/OgreFltk/FltkRenderer.h"
#include "MainLib/OgreFltk/timesensor.h"
#include "MainLib/OgreFltk/FltkMotionWindow.h"
#include "Mesh_old.h"
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
#include "MainLib/OgreFltk/MotionPanel.h"
#include "Ogre_ShapeDrawer.h"
FlLayout* createFlexibleBodyWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer);
namespace FlexibleBody
{
	extern double gravity;
	extern double mass;
	extern double scale_factor;
	extern double air_density;
	extern int bendingConstraints1;
	extern double bendingConstraints2;
	extern int iterations;
	extern double simulationFrameRate;
	extern double kDF;
	extern double kST;
}



///CcdPhysicsDemo shows basic stacking using Bullet physics, and allows toggle of Ccd (using key '1')
class FlexibleBodyWin : public FlLayout, public FrameMoveObject
{
private:

	//////////////////////////////////////////////////////////////
	// Bullet data
	//////////////////////////////////////////////////////////////

	//this is the most important class
	btDynamicsWorld*		m_dynamicsWorld;

	btSoftBody::btSoftBodyWorldInfo	m_softBodyWorldInfo;
	// btSoftBodyWorldInfo::
	btBroadphaseInterface*	m_broadphase;
	btCollisionDispatcher*	m_dispatcher;

	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>		m_collisionShapes;

	btConstraintSolver*	m_solver;

	btDefaultCollisionConfiguration* m_collisionConfiguration;

	Ogre_ShapeDrawer m_shapeDrawer;

	//////////////////////////////////////////////////////////////
	// My data
	//////////////////////////////////////////////////////////////
	Ogre_DebugDrawer* m_debugDrawer;
	Motion mMotion;
	//PLDPrimCustumSkin* mSkin;
	OBJloader::Mesh* mMesh;
	SkinnedMeshLoader* mMeshAnimation;

	void	exitPhysics();

	btRigidBody*	localCreateRigidBody(float mass, const btTransform& 
startTransform,btCollisionShape* shape);

public:

	
	FlexibleBodyWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer);
	~FlexibleBodyWin (void);


	MotionPanel& m_motionPanel;
	FltkRenderer& mRenderer;

	// layout callback
	virtual void onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData);

	// FrameMoveObject
	virtual int FrameMove(float fElapsedTime);

	const btSoftRigidDynamicsWorld*	getSoftDynamicsWorld() const
	{
		return (btSoftRigidDynamicsWorld*) m_dynamicsWorld;
	}

	btSoftRigidDynamicsWorld*	getSoftDynamicsWorld()
	{
		return (btSoftRigidDynamicsWorld*) m_dynamicsWorld;
	}

	void	cleanupScene();
	void	renderme();
	void	keyboardCallback(unsigned char key, int x, int y);

	// LUA interface functions.
	void initWorld();
	void createFloor(double floorHeight, vector3 const& size);
	void createTerrain(const char* filename, vector3 const& pos, int sizeX, int sizeY, m_real width, m_real height, m_real heightMax);

	virtual void show()	{	mRenderer.ogreRenderer().fixedTimeStep(true); FlLayout::show();}
	virtual void hide()	{	mRenderer.ogreRenderer().fixedTimeStep(false); FlLayout::hide();}
};
#endif
