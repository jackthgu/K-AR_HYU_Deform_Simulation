#include "stdafx.h"
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>
#include "TRCwin.h"
#include "IguanaLCPwin.h"
#include "../../MainLib/OgreFltk/OgreMotionLoader.h"
#include "../../BaseLib/motion/version.h"
#include "../../MainLib/OgreFltk/FltkMotionWindow.h"
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

const int maxProxies = 32766;
const int maxOverlap = 65535;
const int maxNumObjects = 32760;

static double prevDrawn=0;
static double prevExactTime=0;

namespace IguanaLCP
{
static bool showHighMesh=true;
}



void IguanaLCPWin::performIguanaIK(Posture& pose, vector3N const& con)
{
	static scoped_ptr<MotionUtil::FullbodyIK> ik;

	if(!ik.get())
	{
		MotionLoader& skel=mMotion->skeleton();
		ik.reset(
				MotionUtil::createFullbodyIk_LimbIK(skel,ee));
		//MotionUtil::createFullbodyIk_MultiTarget(skel, ee));
		//MotionUtil::createFullbodyIk(skel, ee));
	}
	ik->IKsolve(pose, con);
}

double IguanaLCPWin::getTerrainHeight(vector3 const& pos)
{
	return mSimulator.getTerrain()->getTerrainHeight(pos);
}




//#include "BulletSoftBody/Softbody2/Physics.h"


void IguanaLCPWin::updateTargetMesh(Motion& mot, m_real prevTime, m_real time)
{	
	Posture prevPose, pose;
	mot.samplePose(prevPose, prevTime);
	mot.samplePose(pose, time);
	// retrieve example mesh to mTargetMesh.

	mMeshAnimation->setPose(prevPose);
	mMeshAnimation->retrieveAnimatedMesh(mPrevTargetMesh);
	mMeshAnimation->setPose(pose);
	mMeshAnimation->retrieveAnimatedMesh(mTargetMesh);
	//mMeshAnimation.update(mPrevTargetMesh, prevPose);
	//mMeshAnimation.update(*mTargetMesh, pose);
	
	if(matchPose)
	{
		// 타겟메쉬의 이전 프레임과 simulatedMesh의 마지막 프레임을 맞추어준다.

		//PointCloudMetric metric;
		KovarMetric metric;

		OBJloader::Mesh& mSimulatedMesh=mSimulator.getSoftBody()->mSimulatedMesh;

		int nv=mSimulatedMesh.numVertex();
		vectorn simulated(nv*3), captured(nv*3);
		for(int i=0; i<nv; i++)
			simulated.setVec3(i*3, mSimulatedMesh.getVertex(i));

		for(int i=0; i<nv; i++)
			captured.setVec3(i*3, mPrevTargetMesh.getVertex(i));
		
		metric.CalcDistance(simulated, captured);

		for(int i=0; i<nv; i++)
			mPrevTargetMesh.getVertex(i)=metric.m_transformedB.row3(i);

		
		transf mLastRootTransformation;
		mLastRootTransformation=prevPose.rootTransformation();
		mLastRootTransformation.leftMult(metric.m_transfB);

		m_real length=mLastRootTransformation.rotation.length();

		static int iiii=0;
		if(mLastRootTransformation.rotation.x!=mLastRootTransformation.rotation.x)
			printf("error");
		iiii++;
		if(performIK)
		{
			transf& rf=mLastRootTransformation;

			pose.setRootTransformation(pose.rootTransformation()*metric.m_transfB);

			MotionLoader& skel=mMotion->skeleton();
			

			skel.setPose(pose);//mMotion->pose(curFrame));

			int numEE=ee.size();
			vectorn desired_height(numEE);
			vectorn curHeight(numEE);
			vectorn floorHeight(numEE);
			
			for(int i=0; i<numEE; i++)
			{
				desired_height[i]=ee[i].bone->getTranslation().y;
			}
			
			// 4다리의 현재 동작에서의 높이 구하기
			skel.setPose(pose);

			vector3N con(numEE);


			if(!mSimulator.hasTerrain())
			{
				// btBoxShape등이 바닥으로 쓰인경우.
				// 아직 구현안됨.
			}
			else
			{
				for(int i=0; i<numEE; i++)
				{
					con[i]=ee[i].bone->getTranslation();
					floorHeight(i)=mSimulator.getTerrain()->getTerrainHeight(con[i]);

					curHeight[i]=con[i].y-floorHeight(i);
				}

				// 차이만큼 IK해주기.
				static MotionUtil::FullbodyIK* ik;
				
				if(!ik)
				{
					ik=					MotionUtil::createFullbodyIk_LimbIK(skel,ee);
					//MotionUtil::createFullbodyIk_MultiTarget(skel, ee));
					//MotionUtil::createFullbodyIk(skel, ee);
				}


				vectorn delta(numEE);

				//★★
				for(int i=0; i<numEE; i++)
					delta[i]=desired_height[i]-curHeight[i];

				// zero mean
				delta.each1(s1::RSUB, delta.avg());

				
				for(int i=0; i<numEE; i++)
					con[i].y+=delta[i];
				
				ik->IKsolve(pose, con);

				const bool trace_skin=false;
				if(trace_skin)
				{
					static PLDPrimSkin* mTestSkin=NULL;

					if(!mTestSkin)
						mTestSkin=RE::createSkin(skel);

					mTestSkin->SetPose(pose, skel);
				}
			}
			// update targetMesh to reveal IK result.
			mMeshAnimation->setPose(pose);
			mMeshAnimation->retrieveAnimatedMesh(mTargetMesh);
			//mMeshAnimation.update(*mTargetMesh, pose);
		}
	}

	mSimulator.getSoftBody()->changeSoftBodyRestLength(mTargetMesh);//★★
}

void IguanaLCPWin::updateTargetMeshTRCwin(Motion& mot, m_real prevTime, m_real time)
{	
	Posture prevPose, pose;
	mot.samplePose(prevPose, prevTime);
	mot.samplePose(pose, time);
	// retrieve example mesh to mTargetMesh.

	mMeshAnimation->setPose(prevPose);
	mMeshAnimation->retrieveAnimatedMesh(mPrevTargetMesh);
	mMeshAnimation->setPose(pose);
	mMeshAnimation->retrieveAnimatedMesh(mTargetMesh);

	//mMeshAnimation.update(mPrevTargetMesh, prevPose);
	//mMeshAnimation.update(*mTargetMesh, pose);
	
	if(matchPose)
	{
		// 타겟메쉬의 이전 프레임과 simulatedMesh의 마지막 프레임을 맞추어준다.

		//PointCloudMetric metric;
		KovarMetric metric;
		OBJloader::Mesh& mSimulatedMesh=mSimulator.getSoftBody()->mSimulatedMesh;

		int nv=mSimulatedMesh.numVertex();
		vectorn simulated(nv*3), captured(nv*3);
		for(int i=0; i<nv; i++)
			simulated.setVec3(i*3, mSimulatedMesh.getVertex(i));

		for(int i=0; i<nv; i++)
			captured.setVec3(i*3, mPrevTargetMesh.getVertex(i));
		
		metric.CalcDistance(simulated, captured);

		for(int i=0; i<nv; i++)
			mPrevTargetMesh.getVertex(i)=metric.m_transformedB.row3(i);

		mTRCwin->transformSynthesized(metric.m_transfB);

		if(performIK)
		{
			//transf& rf=mLastRootTransformation;

			pose.setRootTransformation(pose.rootTransformation()*metric.m_transfB);

			MotionLoader& skel=mMotion->skeleton();
			
			int numEE=ee.size();
			skel.setPose(pose);//mMotion->pose(curFrame));

			vectorn desired_height(numEE);
			vectorn curHeight(numEE);
			vectorn floorHeight(numEE);
			
			for(int i=0; i<numEE; i++)
			{
				desired_height[i]=ee[i].bone->getTranslation().y;
			}
			
			// 4다리의 현재 동작에서의 높이 구하기
			skel.setPose(pose);

			vector3N con(numEE);


			if(!mSimulator.hasTerrain())
			{
				// btBoxShape등이 바닥으로 쓰인경우.
				// 아직 구현안됨.
			}
			else
			{
				// 4다리의 예재 동작에서의 높이 구하기

				for(int i=0; i<numEE; i++)
				{
					con[i]=ee[i].bone->getTranslation();
					floorHeight(i)=getTerrainHeight(con[i]);
					curHeight[i]=con[i].y-floorHeight(i);
				}

				// 차이만큼 IK해주기.
				vectorn delta(numEE);

				for(int i=0; i<numEE; i++)
					delta[i]=desired_height[i]-curHeight[i];

				// zero mean
				delta.each1(s1::RSUB, delta.avg());

				
				for(int i=0; i<numEE; i++)
					con[i].y+=delta[i];

				performIguanaIK(pose, con);

				const bool trace_skin=false;
				if(trace_skin)
				{
					static PLDPrimSkin* mTestSkin=NULL;

					if(!mTestSkin)
						mTestSkin=RE::createSkin(skel);

					mTestSkin->SetPose(pose, skel);
				}
			}
			// update targetMesh to reveal IK result.
			mMeshAnimation->setPose(pose);
			mMeshAnimation->retrieveAnimatedMesh(mTargetMesh);
			//mMeshAnimation.update(*mTargetMesh, pose);
		}
	}

	mSimulator.getSoftBody()->changeSoftBodyRestLength(mTargetMesh);
}
////////////////////////////////////


void IguanaLCPWin::show()	
{
	m_renderer->ogreRenderer().fixedTimeStep(true); 
	m_renderer->ogreRenderer().setCaptureFPS(30);
	m_renderer->setHandler(this);
	FlLayout::show();
}
void IguanaLCPWin::hide()	
{	
	m_renderer->ogreRenderer().fixedTimeStep(false); 
	m_renderer->ogreRenderer().setCaptureFPS(30);
	FlLayout::hide();
}



int IguanaLCPWin::_frameMove(float fElapsedTime)
{
	//float dt = getDeltaTimeMicroseconds() * 0.000001f;

	{

		int nSubStep=ROUND(simulationFrameRate/renderingFrameRate);
		double step=1.0/simulationFrameRate;

		if(mTargetMesh.numVertex())
		{
			if(mTRCwin && mTRCwin->synthesizing)
			{
				mTRCwin->singleFrame();
				for(int i=0; i<nSubStep; i++)
				{
					double exactTime=sop::clampMap(i,0,nSubStep, prevDrawn, mTRCwin->mLastDrawn);

				
					printf("%f\n", exactTime);
					if(mTRCwin->mTerrain)
						updateTargetMeshTRCwin(*(mTRCwin->m_pTerrainMotion), prevExactTime, exactTime);
					else
						updateTargetMeshTRCwin(*(mTRCwin->m_pMotion), prevExactTime, exactTime);
					
					mSimulator.stepSimulation();

					prevExactTime=exactTime;
				}

				prevDrawn=mTRCwin->mLastDrawn;
			}
			else
			{
				static int prevFrame=0;
				int curFrame=m_motionPanel->motionWin()->getCurrFrame();

				for(int i=0; i<nSubStep; i++)
				{
					double frame=sop::clampMap(i, 0, nSubStep, prevFrame, curFrame);
					updateTargetMesh(*mMotion, frame, frame);
					mSimulator.stepSimulation();
				}

				prevFrame=curFrame;
			}
		}
		
	}
	
	mSimulator.renderme(); 
	if(IguanaLCP::showHighMesh) { updateHighMesh(); }

	return 1;
}



void IguanaLCPWin::_ctor()
{
}

IguanaLCPWin::IguanaLCPWin(int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer)
: ScriptBaseWin(0,0,w,h, mp, renderer, 
		"IguanaLCP.lua",
		"../src/lua/"),
mMeshAnimation(NULL),
//mSkin(NULL)
mNode(NULL)
{
	callCallbackFunction(findWidget("load"));


	int wi=widgetIndex("scriptfn");
	removeWidgets(wi+1);

	create("Button", "Start", "Start");
	button(0)->shortcut(FL_ALT+'s');	
	matchPose=true;
	performIK=true;

	mTRCwin=new Iguana::TRCwin(x,y,w,h,mp, renderer);
	mTRCwin->_autoConnect=false;
	embedLayout(mTRCwin, "IguanaWin", "IguanaWin");
	
	mMotion=NULL;
	updateLayout();
}

void IguanaLCPWin ::initLuaEnvironment()
{
	ScriptBaseWin::initLuaEnvironment();
	Register_classificationLib_bind(L);
	Register_flexiblebody(L);
	lunaStack ls(L);

	ls.set<TRL::DynamicsSimulator_TRL_massSpring>("world", &mSimulator);
	ls.set<IguanaLCPWin>("win", this);


}
IguanaLCPWin ::~IguanaLCPWin (void)
{

	create("Button", "X", "X", 9);

	//RE::remove(mSkin);
	if(mNode) RE::removeEntity(mNode);
}

void IguanaLCPWin::trcWin_setDrawSphere(bool mode)
{
	mTRCwin->_drawSphere=mode;
}
void IguanaLCPWin::start()
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
		//mSkin=(PLDPrimCustumSkin*)RE::createCustumSkin(mMotion);
		m_motionPanel->motionWin()->addSkin(RE::createEmptyAnimationObject(mMotion->numFrames(), mMotion->frameTime()));
		// 두배 느린동작을 따라가기.
		//m_motionPanel->motionWin()->addSkin(RE::createEmptyAnimationObject(mMotion->numFrames(), mMotion->frameTime()*2));
	}

	//m_motionPanel.motionWin()->addSkin(mSkin);
	prevDrawn=0;
	prevExactTime=0;

	double step=1.0/simulationFrameRate;
	mSimulator.init(step, OpenHRP::DynamicsSimulator::EULER);

		//FlexibleImplicitWin_runLUAscript(*this);
		
	mPrevTargetMesh.copyFrom(mTargetMesh);
}

bool IguanaLCPWin::isTRCwin_synthesizing() { return (mTRCwin && mTRCwin->synthesizing);}
void IguanaLCPWin::trcWin_singleFrame() { mTRCwin->singleFrame();}

FlLayout* createIguanaLCPWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer)
{
	return new IguanaLCPWin(x,y,w,h,mp, renderer);
}


	void IguanaLCPWin ::setSkeletonVisible(bool bVisible)
	{
		if(mTRCwin->m_pMotion)
			mTRCwin->m_pSkin->SetVisible(bVisible);
		if(mTRCwin->m_pTerrainMotion)
			mTRCwin->m_pTerrainSkin->SetVisible(bVisible);

	}
void IguanaLCPWin::setOption(const char* title, bool value)
{
	TString t(title);
	if(t=="performIK")
	{
		performIK=value;
	}
}

void IguanaLCPWin::createTargetMesh(int iframe, double scale_factor, vector3 const& initialPosition)
{
	ASSERT(scale_factor==1.0);

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

	if(IguanaLCP::showHighMesh) { showHighMesh(); }


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

	const int numEE=4;
	ee.resize(numEE);

	MotionLoader& skel=mMotion->skeleton();
	ee[0].bone=skel.getBoneByVoca(MotionLoader::LEFTANKLE).child();
	ee[1].bone=skel.getBoneByVoca(MotionLoader::RIGHTANKLE).child();
	ee[2].bone=skel.getBoneByVoca(MotionLoader::LEFTWRIST).child();
	ee[3].bone=skel.getBoneByVoca(MotionLoader::RIGHTWRIST).child();
}

void IguanaLCPWin::showHighMesh()
{
		//SkinnedMeshLoader* pl=new SkinnedMeshLoader ("iguana_pskinned.mesh", true);
		//mIguanaHighMesh=pl->mMesh.mesh;
		//delete pl;
		//mIguanaHighMesh.loadMesh("../Resource/mesh/iguana_high.tri",false);
		mIguanaHighMesh.loadObj("../mesh/iguana_high2.obj");

		TString dfn;
		const bool useMVMdeformer=false;

		

		if(useMVMdeformer)
		{
			dfn="../mesh/iguana_high_mvd.def";
			mDeformer.reset(createMeanValueMeshDeformer());
		}
		else
		{
			dfn="../mesh/iguana_high_hmd.def";
			mDeformer.reset(createHarmonicMeshDeformer());
		}

		if(!mDeformer->loadCorrespondence(dfn,mTargetMesh,mIguanaHighMesh))
		{
			printf("calculating correspondence\n");
			mDeformer->calculateCorrespondence(mTargetMesh, mIguanaHighMesh);
			printf("saving correspondence\n");
			mDeformer->saveCorrespondence(dfn);
		}
		OBJloader::MeshToEntity::Option o;
		o.useTexCoord=true;
		o.useNormal=true;
//#ifdef _DEBUG
		o.buildEdgeList=false;
//#else
//		o.buildEdgeList=true;
//#endif
		mIguanaHighMeshViewer.reset(new OBJloader::MeshToEntity(mIguanaHighMesh,RE::generateUniqueName(), o));
		mIguanaHighMeshViewer->createEntity(RE::generateUniqueName(), "lambert6");

		if (!mNode)
			mNode=RE::createSceneNode(RE::generateUniqueName());
		mIguanaHighMeshViewer->getLastCreatedEntity()->setCastShadows(false);
		mNode->attachObject(mIguanaHighMeshViewer->getLastCreatedEntity());
}
void IguanaLCPWin::updateHighMesh(double scale_factor)
{
		mDeformer->transfer(mSimulator.getSoftBody()->mSimulatedMesh, scale_factor);
		mIguanaHighMesh.calculateVertexNormal();
		mIguanaHighMeshViewer->updatePositionsAndNormals();
}
	int IguanaLCPWin::trcWin_lastDrawn() { return mTRCwin->mLastDrawn;}
	Motion& IguanaLCPWin::trcWin_terrainMotion() { return *mTRCwin->m_pTerrainMotion;}
	Motion& IguanaLCPWin::trcWin_motion() { return *mTRCwin->m_pMotion;}
	void IguanaLCPWin::stepSimulation(double step) { mSimulator.stepSimulation();}
	bool IguanaLCPWin::trcWin_hasTerrain(){return mTRCwin->mTerrain;}
	int IguanaLCPWin::trcWin_handleRendererEvent(const char* evs, int button, int x, int y)
{
	int ev;
	TString _evs=evs;
	if (_evs=="PUSH")
		ev=FL_PUSH;
	else if (_evs=="MOVE")
		ev=FL_MOVE;
	else if(_evs=="DRAG")
		ev=FL_DRAG;
	else if(_evs=="RELEASE")
		ev=FL_RELEASE;
	else if(_evs=="KEYUP")
		ev=FL_KEYUP;
	else if(_evs=="KEYDOWN")
		ev=FL_KEYDOWN;

	return mTRCwin->_handleRendererEvent(ev, button, x, y); 

}

void IguanaLCPWin::trcWin_transformSynthesized(matrix4 const& tf)
{
	mTRCwin->transformSynthesized(tf);
}

#include "SimulatorImplicit.h"
void IguanaLCPWin ::setTRCwinTerrain(const char* filename, vector3 const& pos, int sizeX, int sizeY, m_real width, m_real height, m_real heightMax)
{
	btTransform tr;
	tr.setIdentity();
	tr.setOrigin(ToBullet(pos));

	// 바닥판.
	btCollisionShape* col=createTerrainCollisionShape(filename, sizeX, sizeY, width, height, heightMax);
	btDefaultMotionState* myMotionState = new btDefaultMotionState(tr);
	btVector3 localInertia(0,0,0);
	btRigidBody::btRigidBodyConstructionInfo cInfo(0,myMotionState,col,localInertia);
	btRigidBody* body = new btRigidBody(cInfo);
	mTRCwin->mTerrain=body;
}
