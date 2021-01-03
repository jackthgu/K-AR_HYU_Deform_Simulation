#include "stdafx.h"
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>
#include "TRCwin.h"
#include "FlexibleBodyImplicitWin.h"
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


void FlexibleBodyImplicitWin ::setTRCwinTerrain()
{
	mTRCwin->mTerrain=mSimulator.mTerrain;
}


void FlexibleBodyImplicitWin::performIguanaIK(Posture& pose, vector3N const& con)
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

double FlexibleBodyImplicitWin::getTerrainHeight(vector3 const& pos)
{
	btRigidBody* mTerrain=mSimulator.mTerrain;
	btCollisionShape* shp=mTerrain->getCollisionShape();
	// terrain
	btTerrainTriangleMeshShape*		csh=static_cast<btTerrainTriangleMeshShape*>(shp);
	const btTransform&	wtr=mTerrain->getInterpolationWorldTransform();

	btVector3 localX=wtr.invXform(ToBullet(pos));

	vector3 normal;
	localX.setY(csh->mTerrain->height(vector2(localX.x(), localX.z()), normal));
	vector3 floor=ToBase(wtr(localX));
	return floor.y;
}




//#include "BulletSoftBody/Softbody2/Physics.h"


void FlexibleBodyImplicitWin::updateTargetMesh(Motion& mot, m_real prevTime, m_real time)
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

		OBJloader::Mesh& mSimulatedMesh=mSimulator.mSimulatedMesh;

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


			if(!mSimulator.mTerrain)
			{
				// btBoxShape등이 바닥으로 쓰인경우.
				// 아직 구현안됨.
			}
			else
			{

				btRigidBody* mTerrain=mSimulator.mTerrain;
				btCollisionShape* shp=mTerrain->getCollisionShape();
				// terrain
				btTerrainTriangleMeshShape*		csh=static_cast<btTerrainTriangleMeshShape*>(shp);
				const btTransform&	wtr=mTerrain->getInterpolationWorldTransform();

				for(int i=0; i<numEE; i++)
				{
					con[i]=ee[i].bone->getTranslation();

					btVector3 localX=wtr.invXform(ToBullet(con[i]));

					vector3 normal;
					localX.setY(csh->mTerrain->height(vector2(localX.x(), localX.z()), normal));
					vector3 floor=ToBase(wtr(localX));

					floorHeight(i)=floor.y;

					curHeight[i]=con[i].y-floor.y;
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

	mSimulator.changeSoftBodyRestLength(mTargetMesh);//★★
}

void FlexibleBodyImplicitWin::updateTargetMeshTRCwin(Motion& mot, m_real prevTime, m_real time)
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
		OBJloader::Mesh& mSimulatedMesh=mSimulator.mSimulatedMesh;

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


			if(!mSimulator.mTerrain)
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

	mSimulator.changeSoftBodyRestLength(mTargetMesh);
}
////////////////////////////////////


void FlexibleBodyImplicitWin::show()	
{
	m_renderer->ogreRenderer().fixedTimeStep(true); 
	m_renderer->ogreRenderer().setCaptureFPS(30);
	m_renderer->setHandler(this);
	FlLayout::show();
}
void FlexibleBodyImplicitWin::hide()	
{	
	m_renderer->ogreRenderer().fixedTimeStep(false); 
	m_renderer->ogreRenderer().setCaptureFPS(30);
	FlLayout::hide();
}


int FlexibleBodyImplicitWin::_frameMove(float fElapsedTime)
{
	if(!mSimulator.isWorldValid()) return 0;
	//float dt = getDeltaTimeMicroseconds() * 0.000001f;

	if (mSimulator.isWorldValid())
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
					
					// see btSoftBody2::predictMotion(btScalar dt)
					mSimulator.m_dynamicsWorld->stepSimulation(step,1, step);

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
					mSimulator.m_dynamicsWorld->stepSimulation(step,1, step);
				}

				prevFrame=curFrame;
			}
		}
		
	}
	
	mSimulator.renderme(); 
	//if(FlexibleBodyImplicit::showHighMesh) { updateHighMesh(); }

	return 1;
}



void FlexibleBodyImplicitWin::_ctor()
{
}

FlexibleBodyImplicitWin::FlexibleBodyImplicitWin(int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer)
: ScriptBaseWin(0,0,w,h, mp, renderer, 
		"flexiblebodyimplicit.lua",
		"../src/lua/"),
mMeshAnimation(NULL)
//mSkin(NULL)
{
	callCallbackFunction(findWidget("load"));


	create("Button", "Start", "Start");
	button(0)->shortcut(FL_ALT+'s');	
	create("Button", "setDest", "setDest");
	create("Button", "setView", "setView");
	create("Button", "setView2", "setView2");
	create("Button", "saveView", "saveView");
	matchPose=true;
	performIK=true;

	mTRCwin=new Iguana::TRCwin(x,y,w,h,mp, renderer);
	mTRCwin->_autoConnect=false;
	embedLayout(mTRCwin, "IguanaWin", "IguanaWin");
	
	mMotion=NULL;
	updateLayout();
}

void FlexibleBodyImplicitWin ::initLuaEnvironment()
{
	ScriptBaseWin::initLuaEnvironment();
	Register_classificationLib_bind(L);
	Register_flexiblebody(L);
	lunaStack ls(L);

	ls.set<FlLayout>("this", this);
	ls.set<SimulatorImplicit>("world", &mSimulator);
	ls.set<FlexibleBodyImplicitWin>("win", this);


}
FlexibleBodyImplicitWin ::~FlexibleBodyImplicitWin (void)
{

	create("Button", "X", "X", 9);
	mSimulator.exitWorld();

	//RE::remove(mSkin);
}

void FlexibleBodyImplicitWin::trcWin_setDrawSphere(bool mode)
{
	mTRCwin->_drawSphere=mode;
}
void FlexibleBodyImplicitWin::start()
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

	mSimulator.startWorld();

		//FlexibleImplicitWin_runLUAscript(*this);
		
	mPrevTargetMesh.copyFrom(mTargetMesh);
}

bool FlexibleBodyImplicitWin::isTRCwin_synthesizing() { return (mTRCwin && mTRCwin->synthesizing);}
void FlexibleBodyImplicitWin::trcWin_singleFrame() { mTRCwin->singleFrame();}

FlLayout* createFlexibleBodyImplicitWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer)
{
	return new FlexibleBodyImplicitWin(x,y,w,h,mp, renderer);
}


	void FlexibleBodyImplicitWin ::setSkeletonVisible(bool bVisible)
	{
		if(mTRCwin->m_pMotion)
			mTRCwin->m_pSkin->SetVisible(bVisible);
		if(mTRCwin->m_pTerrainMotion)
			mTRCwin->m_pTerrainSkin->SetVisible(bVisible);

	}
void FlexibleBodyImplicitWin::setOption(const char* title, bool value)
{
	TString t(title);
	if(t=="performIK")
	{
		performIK=value;
	}
}

void FlexibleBodyImplicitWin::createTargetMesh(int iframe, double scale_factor, vector3 const& initialPosition)
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

	//if(FlexibleBodyImplicit::showHighMesh) { showHightMesh() }


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

void FlexibleBodyImplicitWin::showHighMesh()
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
			mDeformer->calculateCorrespondence(mTargetMesh, mIguanaHighMesh);
			mDeformer->saveCorrespondence(dfn);
		}
		OBJloader::MeshToEntity::Option o;
		o.useTexCoord=true;
		o.useNormal=true;
#ifdef _DEBUG
		o.buildEdgeList=false;
#else
		o.buildEdgeList=true;
#endif
		mIguanaHighMeshViewer.reset(new OBJloader::MeshToEntity(mIguanaHighMesh,RE::generateUniqueName(), o));
		mIguanaHighMeshViewer->createEntity(RE::generateUniqueName(), "lambert6");

		mSimulator.mNode->attachObject(mIguanaHighMeshViewer->getLastCreatedEntity());
}
void FlexibleBodyImplicitWin::updateHighMesh()
{
		mDeformer->transfer(mSimulator.mSimulatedMesh,1.0);
		mIguanaHighMesh.calculateVertexNormal();
		mIguanaHighMeshViewer->updatePositionsAndNormals();
}
	int FlexibleBodyImplicitWin::trcWin_setDestination(vector3 dest){ return mTRCwin->setDestination(dest);}
	int FlexibleBodyImplicitWin::trcWin_lastDrawn() { return mTRCwin->mLastDrawn;}
	Motion& FlexibleBodyImplicitWin::trcWin_terrainMotion() { return *mTRCwin->m_pTerrainMotion;}
	Motion& FlexibleBodyImplicitWin::trcWin_motion() { return *mTRCwin->m_pMotion;}
	void FlexibleBodyImplicitWin::stepSimulation(double step) { mSimulator.m_dynamicsWorld->stepSimulation(step,1, step);}
	bool FlexibleBodyImplicitWin::trcWin_hasTerrain(){return mTRCwin->mTerrain;}
	int FlexibleBodyImplicitWin::trcWin_handleRendererEvent(const char* evs, int button, int x, int y)
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

void FlexibleBodyImplicitWin::trcWin_transformSynthesized(matrix4 const& tf)
{
	mTRCwin->transformSynthesized(tf);
}
