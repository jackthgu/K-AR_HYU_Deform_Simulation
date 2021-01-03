
#include "stdafx.h"
#include <OgrePrerequisites.h>
#include <Ogre.h>
#include "TRCwin.h"
#include "../../BaseLib/utility/scoped_ptr.h"
#include "FlexibleBodyImplicitSimpleWin.h"
#include "../../MainLib/OgreFltk/OgreMotionLoader.h"
#include "../../BaseLib/motion/version.h"
#include "../../MainLib/OgreFltk/FltkMotionWindow.h"

namespace FlexibleBodyImplicitSimple
{	
	double simulationFrameRate;
	int skipFrame;
	int iskipFrame;
}

static float renderingFrameRate=240;//240;
static int skipFrame=20;// to adjust speed. default: 1
static int iskipFrame=0;//do not change this.

void FlexibleBodyImplicitSimpleWin::show()	
{
	mRenderer.ogreRenderer().fixedTimeStep(true); 
	mRenderer.ogreRenderer().setCaptureFPS(renderingFrameRate);
	FlLayout::show();
}
void FlexibleBodyImplicitSimpleWin::hide()	
{	
	mRenderer.ogreRenderer().fixedTimeStep(false); 
	mRenderer.ogreRenderer().setCaptureFPS(30);
	FlLayout::hide();
}


// FrameMoveObject
int FlexibleBodyImplicitSimpleWin::FrameMove(float fElapsedTime)
{
	if(!mSimulator.isWorldValid()) return 0;
	//float dt = getDeltaTimeMicroseconds() * 0.000001f;

	if (mSimulator.isWorldValid())
	{

		int nSubStep=ROUND(FlexibleBodyImplicitSimple::simulationFrameRate/renderingFrameRate);
		double step=1.0/FlexibleBodyImplicitSimple::simulationFrameRate;

		if(mTargetMesh.numVertex())
		{
			if (iskipFrame==0)
			{
				int curFrame=m_motionPanel.motionWin()->getCurrFrame();
				mMeshAnimation->setPose(mMotion->pose(curFrame));
				mMeshAnimation->retrieveAnimatedMesh(mTargetMesh);

				for(int i=0; i<nSubStep; i++)
				{
					//void Physics_ParticleSystem::Update( float fTime )
					mSimulator.m_dynamicsWorld->stepSimulation(step,1, step);
				}
			}
			iskipFrame=(iskipFrame+1)%skipFrame;
		}
	}
	
	mSimulator.renderme(); 


	//RE::renderer().mRoot->renderOneFrame();
	//RE::renderer().mRoot->renderOneFrame();
	//RE::renderer().mRoot->renderOneFrame();
	return 1;
}



void FlexibleBodyImplicitSimpleWin_runLUAscript(FlexibleBodyImplicitSimpleWin& win);

FlexibleBodyImplicitSimpleWin::FlexibleBodyImplicitSimpleWin(int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer)
: FlLayout(0,0,w,h),
m_motionPanel(mp),
mRenderer(renderer),
mMeshAnimation(NULL),
mMotion(NULL)
{
	create("Button", "Start", "Start");
	button(0)->shortcut(FL_ALT+'s');	

	updateLayout();
	renderer.ogreRenderer().addFrameMoveObject(this);

}

FlexibleBodyImplicitSimpleWin ::~FlexibleBodyImplicitSimpleWin (void)
{
	mSimulator.exitWorld();

	//RE::remove(mSkin);
}

void FlexibleBodyImplicitSimpleWin::onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData)
{
	if(w.mId=="Start")
	{
		if(!mSimulator.m_dynamicsWorld)
		{
			if(!mMotion)
			{
				std::vector<Motion*> motions;
				m_motionPanel.loader()->startManualLoad(1, motions);
				mMotion=motions[0];
				mMotion->InitSkeleton(RE::motionLoader("iguana.skl"));
				mMotion->skeleton().readJointIndex("../Resource/motion/trc/iguana_from248.ee");
				RE::motion::concatFromFile(*mMotion, "trc/iguana_motion_set.mot");
				mMotion->SetIdentifier("iguana");

				m_motionPanel.loader()->endManualLoad(motions);
			}
			
			
			//mSkin=(PLDPrimCustumSkin*)RE::createCustumSkin(mMotion);
			m_motionPanel.motionWin()->addSkin(RE::createEmptyAnimationObject(mMotion->numFrames(), mMotion->frameTime()));

			// 두배 느린동작을 따라가기.
			//m_motionPanel.motionWin()->addSkin(RE::createEmptyAnimationObject(mMotion->numFrames(), mMotion->frameTime()*2));
			//m_motionPanel.motionWin()->addSkin(mSkin);
		}

		mSimulator.startWorld();

		FlexibleBodyImplicitSimpleWin_runLUAscript(*this);
	}
}


FlLayout* createFlexibleBodyImplicitSimpleWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer)
{
	return new FlexibleBodyImplicitSimpleWin(x,y,w,h,mp, renderer);
}

void FlexibleBodyImplicitSimpleWin::createTargetMesh(int iframe, double scale_factor, vector3 const& initialPosition)
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

#include "../../MainLib/OgreFltk/FlLayout.h"
#include "../../MainLib/WrapperLua/luna_baselib.h"
#include "../../MainLib/WrapperLua/luna_mainlib.h"
#include "../../MainLib/OgreFltk/VRMLloader.h"
#include "../../MainLib/OgreFltk/VRMLloaderView.h"
#include "luna_flexiblebody.h"
void Register_baselib(lua_State*L);
void Register_mainlib(lua_State*L);
void Register_physicsbind(lua_State*L);
void Register_classificationLib_bind(lua_State*L);
void Register_flexiblebody(lua_State*L);

void FlexibleBodyImplicitSimpleWin_runLUAscript(FlexibleBodyImplicitSimpleWin& win)
{
	TString default_script="../src/lua/flexiblebodyimplicitsimple.lua";

	LUAwrapper* L=new LUAwrapper();

	// export Mainlib classes and functions for use in LUA script.
		Register_baselib(L->L);
		Register_mainlib(L->L);
		Register_physicsbind(L->L);
		Register_classificationLib_bind(L->L);
		Register_flexiblebody(L->L);
		L->setRef<SimulatorImplicit>("world", win.mSimulator);
		L->setRef<FlexibleBodyImplicitSimpleWin>("win", win);

		L->dofile(default_script);

		FlexibleBodyImplicitSimple::simulationFrameRate=L->getDouble("simulationFrameRate");
		delete L;
}
