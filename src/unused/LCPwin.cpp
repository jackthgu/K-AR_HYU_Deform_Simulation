
#include "stdafx.h"
#include <OgrePrerequisites.h>
#include <Ogre.h>
#include "../../BaseLib/utility/scoped_ptr.h"
#include "LCPwin.h"
#include "../../MainLib/OgreFltk/OgreMotionLoader.h"
#include "../../BaseLib/motion/version.h"
#include "../../MainLib/OgreFltk/FltkMotionWindow.h"

#include "../../PhysicsLib/ScriptBaseWin.h"
#include "../../MainLib/OgreFltk/FlLayout.h"
#include "../../MainLib/WrapperLua/luna_baselib.h"
#include "../../MainLib/WrapperLua/luna_mainlib.h"
#include "../../MainLib/OgreFltk/VRMLloader.h"
#include "../../MainLib/OgreFltk/VRMLloaderView.h"
class btRigidBody;
#include "luna_flexiblebody.h"
void Register_baselib(lua_State*L);
void Register_mainlib(lua_State*L);
void Register_physicsbind(lua_State*L);
void Register_classificationLib_bind(lua_State*L);
void Register_flexiblebody(lua_State*L);

void FlexibleBodyImplicitSimpleLCPWin::show()	
{
	m_renderer->ogreRenderer().fixedTimeStep(true); 
	ScriptBaseWin::show();
}
void FlexibleBodyImplicitSimpleLCPWin::hide()	
{	
	m_renderer->ogreRenderer().fixedTimeStep(false); 
	m_renderer->ogreRenderer().setCaptureFPS(30);
	ScriptBaseWin::hide();
}


// FrameMoveObject


FlexibleBodyImplicitSimpleLCPWin::FlexibleBodyImplicitSimpleLCPWin(int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer)
: ScriptBaseWin(0,0,w,h, mp, renderer, 
		"testLCP.lua",
		"../src/lua/test/"),
mMeshAnimation(NULL),
mMotion(NULL)
{
}

void FlexibleBodyImplicitSimpleLCPWin ::loadIguanaMotion()
{
	if(!mMotion)
	{
		std::vector<Motion*> motions;
		m_motionPanel->loader()->startManualLoad(1, motions);
		mMotion=motions[0];
		mMotion->InitSkeleton(RE::motionLoader("iguana.skl"));
		mMotion->skeleton().readJointIndex("../Resource/motion/trc/iguana_from248.ee");
		RE::motion::concatFromFile(*mMotion, "trc/iguana_motion_set.mot");
		mMotion->SetIdentifier("iguana");

		m_motionPanel->loader()->endManualLoad(motions);
	}


	//mSkin=(PLDPrimCustumSkin*)RE::createCustumSkin(mMotion);
	m_motionPanel->motionWin()->addSkin(RE::createEmptyAnimationObject(mMotion->numFrames(), mMotion->frameTime()));
}
FlexibleBodyImplicitSimpleLCPWin ::~FlexibleBodyImplicitSimpleLCPWin (void)
{

	//RE::remove(mSkin);
}
void FlexibleBodyImplicitSimpleLCPWin ::initLuaEnvironment()
{
	ScriptBaseWin::initLuaEnvironment();
	Register_classificationLib_bind(L);
	Register_flexiblebody(L);
	lunaStack ls(L);
	ls.set<TRL::DynamicsSimulator_TRL_massSpring>("world", &mSimulator);
	ls.set<FlexibleBodyImplicitSimpleLCPWin>("win", this);
}

void FlexibleBodyImplicitSimpleLCPWin::startWorld(double timestep)
{
	if(!mSimulator.hasTerrain())
	{
		if(!mMotion)
		{
			std::vector<Motion*> motions;
			m_motionPanel->loader()->startManualLoad(1, motions);
			mMotion=motions[0];
			mMotion->InitSkeleton(RE::motionLoader("iguana.skl"));
			mMotion->skeleton().readJointIndex("../Resource/motion/trc/iguana_from248.ee");
			RE::motion::concatFromFile(*mMotion, "trc/iguana_motion_set.mot");
			mMotion->SetIdentifier("iguana");

			m_motionPanel->loader()->endManualLoad(motions);
		}


		//mSkin=(PLDPrimCustumSkin*)RE::createCustumSkin(mMotion);
		m_motionPanel->motionWin()->addSkin(RE::createEmptyAnimationObject(mMotion->numFrames(), mMotion->frameTime()));

		// 두배 느린동작을 따라가기.
		//m_motionPanel.motionWin()->addSkin(RE::createEmptyAnimationObject(mMotion->numFrames(), mMotion->frameTime()*2));
		//m_motionPanel.motionWin()->addSkin(mSkin);
	}

	mSimulator.init(timestep, OpenHRP::DynamicsSimulator::EULER);

}


FlLayout* createFlexibleBodyImplicitSimpleLCPWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer)
{
	return new FlexibleBodyImplicitSimpleLCPWin(x,y,w,h,mp, renderer);
}

void FlexibleBodyImplicitSimpleLCPWin::createTargetMesh(int iframe, double scale_factor, vector3 const& initialPosition)
{
	ASSERT(scale_factor=1.0);

	//mMeshAnimation.setScaleFactor(scale_factor);
	delete mMeshAnimation;
	mMeshAnimation=new SkinnedMeshLoader("iguana_physics.mesh");
	mMeshAnimation->mMesh.mergeDuplicateVertices();
	mMeshAnimation->mMesh.mesh.calculateVertexNormal();
	mMeshAnimation->mMesh.reorderVertices();	// to match iguana_bound.obj
	mMeshAnimation->sortBones(mMotion->skeleton());
	Posture bindpose;
	loadPose(bindpose, "../mesh/iguana_bind.pose");
	mMeshAnimation->setPose(bindpose);
	mMeshAnimation->setCurPoseAsBindPose();
	mMeshAnimation->mMesh.mesh.loadMesh("../mesh/iguana_bound.tri",false);	// 내가 인덱싱만 바꾼 메쉬를 읽는다. -  volume mesh 와 호환되도록 바꾸었다.
	mMeshAnimation->mMesh.mesh.resizeBuffer(OBJloader::Buffer::NORMAL, mMeshAnimation->mMesh.mesh.numVertex());
	mMeshAnimation->mMesh.mesh.copyIndex(OBJloader::Buffer::VERTEX, OBJloader::Buffer::NORMAL);

	//mMeshAnimation.create(*mMotion, "iguana_physics.mesh", *mTargetMesh, iframe);
	//mTargetMesh->loadMesh("../Resource/mesh/iguana.tri");	// 내가 인덱싱만 바꾼 메쉬를 읽는다. -  volume mesh 와 호환되도록 바꾸었다.
	mTargetMesh.copyFrom(mMeshAnimation->mMesh.mesh);


	for(int i=0; i<mTargetMesh.numVertex(); i++)
		mTargetMesh.getVertex(i)+=initialPosition;
		
/*	mTargetVolumeMesh.loadSurfaceMesh("../Resource/mesh/iguana_high.tri");
	mTargetVolumeMesh.loadVolumeMesh("../Resource/mesh/iguana.tet");
	matrix4 s;

	vector3 center=mTargetVolumeMesh.surf_embedded._mesh.calcMeshCenter();
	s.identity();
	s.leftMultTranslation(center*-1);
	s.leftMultScaling(0.8,0.8, 1.0);
	s.leftMultTranslation(center+vector3(0,0.7,0));
	mTargetVolumeMesh.surf_embedded._mesh.transform(s);
	mTargetVolumeMesh.syncSurface();
*/

/*	matrix4 m;
	m.setRotationX(TO_RADIAN(90));
#ifdef MVMC_TEST
	iguanaMesh.transform(m);
	deformer->transfer();
#endif*/

	
	/*
	Ogre::SceneNode* pnode;
	pnode=RE::createSceneNode("____");
	mIguanaHighMesh.calculateVertexNormal();
	pnode->attachObject(OBJloader::createMeshEntity(mIguanaHighMesh, RE::generateUniqueName(), "lambert6", o));
	pnode->translate(0,30,0);
*/
	/*
	Ogre::SceneNode* pnodet;
	pnodet=RE::createSceneNode("____t");
	
	pnodet->attachObject(OBJloader::createMeshEntity(iguanaMesh, RE::generateUniqueName(), "white"));
	pnodet->translate(0,60,0);*/

}


