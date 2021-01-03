
#include "stdafx.h"
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>
#include "MainLib/OgreFltk/OgreMotionLoader.h"
#include <OgreRoot.h>

#include "FELearning.h"

namespace FELearning
{
	bool showTargetMesh=true;
	bool matchPose;
	bool performIK;
	m_real scale_factor;
}



////////////////////////////////////

// FrameMoveObject
int FELearningWin::FrameMove(float fElapsedTime)
{
	if(!mSimulator.isWorldValid()) return 0;
	//float dt = getDeltaTimeMicroseconds() * 0.000001f;

	return 1;
}




//void FELearningWin_runLUAscript_init(FELearningWin& win);
void FELearningWin_runLUAscript_optimize(FELearningWin& win);

FELearningWin::FELearningWin(int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer)
: FlLayout(0,0,w,h),
m_motionPanel(mp),
mRenderer(renderer),
mTargetMesh(NULL)
//mSkin(NULL)
{
	create("Button", "Start", "Start");
	button(0)->shortcut(FL_ALT+'s');	

	mMotion=NULL;
	updateLayout();
	renderer.ogreRenderer().addFrameMoveObject(this);

}

FELearningWin ::~FELearningWin (void)
{
	mSimulator.exitWorld();

	//RE::remove(mSkin);
}

void FELearningWin::onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData)
{
	if(w.mId=="Start")
	{

		if(!mSimulator.m_dynamicsWorld)
		{
			//mMotion->Init(RE::motionLoader("TRC/iguana_from248.bvh"));
			//mMotion->SetIdentifier("iguana");

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
			
			
			mTargetMesh=new OBJloader::Mesh();
			//if(FELearning::showTargetMesh)
				//RE::ogreRootSceneNode()->attachObject(mTargetMesh);
			
			//m_motionPanel.motionWin()->addSkin(RE::createEmptyAnimationObject(mMotion->numFrames(), mMotion->frameTime()));
			// 두배 느린동작을 따라가기.
			//m_motionPanel.motionWin()->addSkin(RE::createEmptyAnimationObject(mMotion->numFrames(), mMotion->frameTime()*2));
			//m_motionPanel.motionWin()->addSkin(mSkin);
		}

		mOffset.setSize(0);

		//FELearningWin_runLUAscript_init(*this);
		
		
		FELearningWin_runLUAscript_optimize(*this);

		
	}
}

void FELearningWin::createTargetMesh(int iframe, double scale_factor)
{
	ASSERT(scale_factor=1.0);
	//mMeshAnimation.setScaleFactor(scale_factor);
	mMeshAnimation=new SkinnedMeshLoader("iguana_physics.mesh");
	mMeshAnimation->mMesh.mergeDuplicateVertices();
	mMeshAnimation->mMesh.mesh.calculateVertexNormal();
	mMeshAnimation->mMesh.reorderVertices();	// to match iguana_bound.obj
	mMeshAnimation->sortBones(mMotion->skeleton());

	*mTargetMesh=mMeshAnimation->mMesh.mesh;
	//mTargetMesh->firstInit();
}

void FELearningWin::updateTargetMesh(double frame)
{
	static Posture pose;
	mMotion->samplePose(pose, frame);
	pose.m_aTranslations[0].x=0;
	pose.m_aTranslations[0].z=0;

	mMeshAnimation->setPose(pose);
	mMeshAnimation->retrieveAnimatedMesh(*mTargetMesh);
	// retrieve example mesh to mTargetMesh.
	//mMeshAnimation.update(*mTargetMesh, pose);
}

void FELearningWin::measureError(OBJloader::Mesh const& mesh1, OBJloader::Mesh const& mesh2, vectorn & offset)
{
	double error=0;

	btSoftBody2* psb=mSimulator.getSoftBody();
	
	int ni=psb->getLinks().size();
	offset.setSize(ni);

	for(int i=0;i<ni;++i)
	{
		int i1=psb->getLinks()[i].m_n[0]->m_index;
		int i2=psb->getLinks()[i].m_n[1]->m_index;

		double l1=mesh1.getVertex(i1).distance(mesh1.getVertex(i2));
		double l2=mesh2.getVertex(i1).distance(mesh2.getVertex(i2));

		offset(i)=l1-l2;
	}
}


void FELearningWin::stepSimulation(double step)
{
	vectorn length;
	mSimulator.extractLinkLength(*mTargetMesh, length);

	if(mOffset.size())
		length+=mOffset;
	else
	{
		mOffset.setSize(length.size());
		mOffset.setAllValue(0);
	}

	mSimulator.setRestLength(length);
	mSimulator.m_dynamicsWorld->stepSimulation(step,1, step);
}

void FELearningWin::setOffset(vectorn const& offset)
{
	mOffset=offset;
}

void FELearningWin::renderScene()
{
	mSimulator.renderme(); 
	//RE::renderer().
	RE::renderer().mRoot->renderOneFrame();
}

FlLayout* createFELearningWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer)
{
	return new FELearningWin(x,y,w,h,mp, renderer);
}

