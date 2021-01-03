#include "stdafx.h"
#include "MainLib/OgreFltk/FlLayout.h"
#include "TRCwin.h"
#include "IguanaIKSolver.h"
//#include "../mainlibpython/mainlibpython.h"
#include "MainLib/OgreFltk/MotionPanel.h"
#include "MainLib/OgreFltk/objectList.h"
#include "BaseLib/math/intervals.h"
#include "BaseLib/math/Metric.h"
#include "BaseLib/motion/MotionUtil.h"
#include "BaseLib/motion/MotionLoader.h"
#include "BaseLib/motion/FullbodyIK.h"
#include "BaseLib/utility/scoped_ptr.h"
#include "BaseLib/math/conversion.h"
#include "BaseLib/motion/FullbodyIK.h"
#include "Stitch.h"
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>
#include "btBulletDynamicsCommon.h"
#include "bulletTools.h"

#define CURVE_ANGLE_THRESH		10
#define RUN_CURVE_ANGLE_THRESH	60
#define RUN_DIST_THRESH			150
#define STOP_DIST_THRESH		50
const int MAX_CONSECUTIVE_RUN=1;

#define MOTION_EDIT

//enum IGUANA_STATE {}

using namespace Iguana;

static void setAllValue(std::vector<TGL::node_struct*>& inout, TGL::node_struct* val)
{
	for (std::vector<TGL::node_struct*>::iterator i=inout.begin(); i!=inout.end(); ++i)
		*i=val;
}
static void setAllValue(std::vector<TGL::node_struct*>& inout,int start, int end, TGL::node_struct* val)
{
	for(int i=start; i<end; i++)
		inout[i]=val;
}
MotionGraph ::MotionGraph (const Motion& mot, bitvectorn const& segmentation)
:mSrcMotion(mot)
{
	mSrcMotion.CalcInterFrameDifference();
	intIntervals grp;
	grp.runLengthEncodeCut(segmentation);

	// add motion segments 
	for(int seg=0; seg<grp.numInterval(); seg++)
	{
		int startTime=grp.start(seg);
		int endTime=grp.end(seg);

		int minLength=mot.NumFrames(0.15) ;
		// 너무 짧거나.. 불연속적인 세그먼트가 있거나(concat된 동작) 하지 않은경우만 그래프에
		// 추가한다. 
		if(endTime-startTime> minLength && mot.IsValid(startTime, endTime)
			&& mot.IsContinuous(startTime) /*&& mot.IsContinuous(endTime)*/
			&& startTime!=0)
		{
			nodeS node=mGraph.newNode();
			node.data().m_nFirstFrame=startTime;
			node.data().m_nLastFrame=endTime;
			cout << "start time: " << startTime << endl;
		}
	}

	mSegmentByFrame.resize(mSrcMotion.numFrames());
	setAllValue(mSegmentByFrame, NULL);

	nodeS v;
	TGL_for_all_node(v, mGraph)
	{
		setAllValue(mSegmentByFrame, v.data().m_nFirstFrame, v.data().m_nLastFrame+1,v._ptr);
	}



	//mGraph.newEdge(findSegment(37), findSegment(2158));
	//mGraph.newEdge(findSegment(2158), findSegment(37));

	/*mGraph.newEdge(findSegment(37), findSegment(269));
	mGraph.newEdge(findSegment(37), findSegment(37));
	mGraph.newEdge(findSegment(37), findSegment(1037));
	mGraph.newEdge(findSegment(37), findSegment(2254));

	mGraph.newEdge(findSegment(269), findSegment(269));
	mGraph.newEdge(findSegment(269), findSegment(37));
	mGraph.newEdge(findSegment(269), findSegment(1037));
	mGraph.newEdge(findSegment(269), findSegment(2254));

	mGraph.newEdge(findSegment(1037), findSegment(269));
	mGraph.newEdge(findSegment(1037), findSegment(37));
	mGraph.newEdge(findSegment(1037), findSegment(1037));
	mGraph.newEdge(findSegment(1037), findSegment(2254));

	mGraph.newEdge(findSegment(2254), findSegment(269));
	mGraph.newEdge(findSegment(2254), findSegment(37));
	mGraph.newEdge(findSegment(2254), findSegment(1037));
	mGraph.newEdge(findSegment(2254), findSegment(2254));*/

	


	TGL::node_array<TString> name;
	mGraph.connect(name);

	TGL_for_all_node(v, mGraph)
		name[v].format("%d:%d", v.index(), v->m_nFirstFrame);

	DRAW(mGraph, name, "iguana");
	
	//mGraph.newEdge(findSegment(269), findSegment(800));
	//mGraph.newEdge(findSegment(269), findSegment(37));
	//mGraph.newEdge(mGraph.findNode(800), mGraph.findNode(800));

	/*TGL_for_all_node(v, mGraph){
		nodeS v2;
		TGL_for_all_node(v2, mGraph)	{
			// 그냥 full 그래프를 만들어 버렸음. 원래는 transition이 가능한 에지만 넣어야함.
			mGraph.newEdge(v, v2);
		}
	}*/
}

nodeS MotionGraph::findSegment(int startTime)
{
	nodeS s;
	s._ptr=(TGL::node_struct*)mSegmentByFrame[startTime];
	return s;
}

TRCwin::TRCwin(int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer)
:FlLayout(x,y,w,h),m_motionPanel(mp),mRenderer(renderer)
{
	_drawSphere=true;
	setUniformGuidelines(2);
	create("Light_Button", "Synthesis", "Synthesis");
	create("Check_Button", "Direct Control", "Direct Control");
	create("Check_Button", "Cons", "Cons");
	create("Check_Button", "Stitch", "Stitch");
	checkButton(0)->value(1);

	create("Check_Button", "Use terrain", "Use terrain");

	synthesizing=false;
	updateLayout();

	renderer.ogreRenderer().addFrameMoveObject(this);

	mGraph=NULL;
	mTimer=NULL;
	isTrack = false;
	isAttack = false;
	isJump = false;
	isXrun = false;
	isDirect = false;
	directState = STOP;
	mTerrain=NULL;
}

TRCwin::~TRCwin(void)
{
	delete mGraph;
	delete mTimer;
}

void TRCwin::show()
{
	FlLayout::show();
	if(_autoConnect)
		mRenderer.setHandler(this);
}

void TRCwin::hide()
{
	FlLayout::hide();
}

void TRCwin::setAutoConnect(bool b)
{
	_autoConnect=b;
}

btRigidBody* createTerrain(const char* filename, vector3 const& pos, int sizeX, int sizeY, m_real width, m_real height, m_real heightMax)
{
	btTransform tr;
	tr.setIdentity();
	tr.setOrigin(ToBullet(pos));

	// 바닥판.
	btRigidBody* body = btLocalCreateRigidBody(0.f,tr,
		createTerrainCollisionShape(filename, sizeX, sizeY, width, height, heightMax), RE::ogreRootSceneNode());

	return body;
}

void TRCwin::onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData)
{
	if(w.mId=="Synthesis")
	{
		if(w.checkButton()->value())
		{
			if(findCheckButton("Use terrain")->value())
			{
				if(!mTerrain)
				{
					RE::createChildSceneNode(RE::ogreRootSceneNode(), "BackgroundNode");

					mTerrain=createTerrain("../Resource/crowdEditingScripts/terrain/heightmap1_256_256_2bytes.raw", vector3(-200,-50, -200), 256, 256, 800, 800, 50);
				}
			}
	
			currFootIsR=false;
			synthesizing=true;
			Motion& srcMot=m_motionPanel.currMotion();

			bitvectorn segmentation;
			segmentation.resize(srcMot.numFrames());
			segmentation.clearAll();

			//for(int i=0; i<segmentation.size(); i+=40)
			//	segmentation.setAt(i);	// 임의로 40프레임마다 잘라주었음. -잘 자르도록 고칠것.
#define USE_IGUANA_MIN			
#ifdef USE_IGUANA_MIN
			segmentation.load(srcMot.numFrames(), "../src/lua/iguana_min.bit");

			mGraph=new Iguana::MotionGraph(srcMot, segmentation);


			segmentR.setSize(NUM_TYPE);
			segmentR.setAllValue(-1);
			segmentL.setSize(NUM_TYPE);
			segmentL.setAllValue(-1);

			// 오른발에서 시작해서 오른발으로 끝나는 세그먼트들 (R-L-R)
			segmentR[STOP]=mGraph->findSegment(4008).index();
			segmentR[STOPPING]=mGraph->findSegment(3858).index();
			segmentR[RUN_BEFORE]=mGraph->findSegment(3558).index();		//뛰기전
			segmentR[RUN]=mGraph->findSegment(3625).index();		//뛰기
			segmentR[RUN_AFTER]=mGraph->findSegment(3697).index();		//뛰기후
			segmentR[JUMP]=mGraph->findSegment(3317).index();		//점프
			segmentR[ATTACK]=mGraph->findSegment(800).index();		//공격
			segmentR[XRUN]=mGraph->findSegment(1045).index();		//전력질주


			// R-L
			segmentR[LEFT]=mGraph->findSegment(2158).index();		// 좌회전
			segmentR[RIGHT]=mGraph->findSegment(2755).index();		// 우회전
			segmentR[WALK]=mGraph->findSegment(1205).index();		// 걷기
			

			// L-R
			segmentL[LEFT]=mGraph->findSegment(2158).index()+1;		// 좌회전
			segmentL[RIGHT]=mGraph->findSegment(2755).index()+1;		// 우회전
			segmentL[WALK]=mGraph->findSegment(1205).index()+1;		// 걷기
#else
			segmentation.load(srcMot.numFrames(), "../src/lua/iguana.bit");

			mGraph=new Iguana::MotionGraph(srcMot, segmentation);


			segmentR.setSize(NUM_TYPE);
			segmentR.setAllValue(-1);
			segmentL.setSize(NUM_TYPE);
			segmentL.setAllValue(-1);

			// 오른발에서 시작해서 오른발으로 끝나는 세그먼트들 (R-L-R)
			segmentR[STOP]=mGraph->findSegment(4008).index();
			segmentR[STOPPING]=mGraph->findSegment(3858).index();
			segmentR[RUN_BEFORE]=mGraph->findSegment(3558).index();		//뛰기전
			segmentR[RUN]=mGraph->findSegment(3652).index();		//뛰기
			segmentR[RUN_AFTER]=mGraph->findSegment(3681).index();		//뛰기후
			segmentR[JUMP]=mGraph->findSegment(3317).index();		//점프
			segmentR[ATTACK]=mGraph->findSegment(800).index();		//공격
			segmentR[XRUN]=mGraph->findSegment(1037).index();		//전력질주


			// R-L
			segmentR[LEFT]=mGraph->findSegment(2158).index();		// 좌회전
			segmentR[RIGHT]=mGraph->findSegment(2755).index();		// 우회전
			segmentR[WALK]=mGraph->findSegment(1205).index();		// 걷기
			
			// L-R 없음.
#endif
			typeName.resize(NUM_TYPE);
			typeName[STOP]="STOP";
			typeName[STOPPING]="STOPPING";
			typeName[LEFT]="LEFT";
			typeName[RIGHT]="RIGHT";		
			typeName[WALK]="WALK";		
			typeName[RUN_BEFORE]="RUN_BEFORE";
			typeName[RUN]="RUN";		
			typeName[RUN_AFTER]="RUN_AFTER";	
			typeName[JUMP]="JUMP";		
			typeName[ATTACK]="ATTACK";		
			typeName[XRUN]="XRUN";		


			mTime.setSize(0);	
			mNode.clear();
			mTime.push_back(0);
			curState=STOP;
			mNode.push_back(mGraph->mGraph.findNode(segmentR(curState)));
			m_pMotion=new Motion();
			if(mTerrain)
				m_pTerrainMotion=new Motion();
			else 
				m_pTerrainMotion=NULL;

			oneStep();	// 한 세그먼트를 생성한다.

			mTimer=new FrameSensor();
			
			//m_pSkin=RE::createSkin(m_pMotion->skeleton(),RE::PLDPRIM_CYLINDER);
			//m_pSkin=RE::createOgreSkin(m_pMotion->skeleton());
			m_pSkin=RE::createOgreSkin(srcMot);
			m_pSkin->setDrawCallback(this);
			mTimer->connect(m_pMotion, m_pSkin, true);
			
			if(mTerrain)
			{
				m_pTerrainSkin=RE::createSkin(m_pMotion->skeleton(), RE::PLDPRIM_CYLINDER);
				mTimer->connect(m_pTerrainMotion, m_pTerrainSkin, true);
			}
			
			//RE::renderer().addFrameMoveObject(mTimer); // manually done in singleFrame()
			mTimer->triggerSet(this, playEnd()-1.0);
			mLastDrawn=0;
		}
		else
		{
			synthesizing=false;
			m_pMotion->exportMOT("result.mot");
			mTime.setSize(0);
			mNode.clear();
			delete m_pMotion;
			RE::remove(m_pSkin);

			if(mTerrain)
			{
				delete m_pTerrainMotion;
				RE::remove(m_pTerrainSkin);
			}
			delete mGraph;
			mGraph=NULL;
			//RE::renderer().removeFrameMoveObject(mTimer);
			delete mTimer;
			mTimer=NULL;
		}
	}

	if(w.mId=="Direct Control")
	{
		if(/* ev==FL_RELEASE && */(findCheckButton("Direct Control")->value()==1) ){
			isDirect = true;
			cout << "true";
		}
		else if( /*ev==FL_RELEASE &&*/ (findCheckButton("Direct Control")->value()==0)){
			isDirect = false;
			cout << "false";
		}
	}
}


// PLDPrimSkin::DrawCallback
void TRCwin::draw(const Posture& posture, const MotionLoader& skeleton)
{
	mLastDrawn=mTimer->curFrame();

	RE::output("timer", "%f", mLastDrawn);
}
/*
void TRCwin::draw(const Motion& mot, int iframe)
{
	mLastDrawn=iframe;
}*/

// FrameMoveObject
int TRCwin::FrameMove(float fElapsedTime)
{	
	if(mTerrain)
	{
		btScalar m[16];

		btRigidBody* body=mTerrain;

		if (body && body->getMotionState())
		{
			btDefaultMotionState* myMotionState = (btDefaultMotionState*)body->getMotionState();
			myMotionState->m_graphicsWorldTrans.getOpenGLMatrix(m);
		} else
		{
			body->getWorldTransform().getOpenGLMatrix(m);
		}
		
		btVector3 wireColor(1.f,1.0f,1.0f); 
		m_shapeDrawer.drawOpenGL(m,body, body->getCollisionShape(),wireColor,0);
	}
	return 1;
}

void TRCwin::singleFrame()
{
	mTimer->FrameMove(1.0/30.0);
}
// FrameSensor::Trigger 
void TRCwin::Triggered(FrameSensor* pTimer)
{
	oneStep();
	mTimer->triggerSet(this, playEnd()-1.0);
}

#include <Fl/Fl.H>
// FltkRenderer::Handler
int	TRCwin::handleRendererEvent(int ev) 
{
	if(!visible()) return 0;
	switch(ev)
	{
	case FL_PUSH:
	case FL_MOVE: 
	case FL_DRAG: 
	case FL_RELEASE:
		{
			int x=Fl::event_x();
			int y=Fl::event_y();
			int button=Fl::event_button();
			bool bButton1=Fl::event_state()&FL_BUTTON1;
			bool bButton2=Fl::event_state()&FL_BUTTON2;
			bool bButton3=Fl::event_state()&FL_BUTTON3;
			RE::output("mouseIguana", "%d %d %d %d", ev, x, y, button);
			_handleRendererEvent(ev, button, x,y);
		}
		break;	
		case FL_KEYUP:
		{
			int key;
			key=Fl::event_key();
			_handleRendererEvent(ev, key, -1,-1);
			//printf("key up %c\n", key);
		}
		break;
	}
	return 0;
}

nodeS TRCwin::getState(int state)
{
	if(state==JUMP)
		isJump=false;
	else if(state==ATTACK)
		isAttack=false;
	else if(state==XRUN)
		isXrun=false;

	int out;
	int prevState=curState;
	if(currFootIsR || segmentL(curState)==-1)
	{
		// 오른발로 끝난 경우.
		curState=state;
		out=segmentR(curState); 
		if(segmentL(curState)!=-1)
			currFootIsR=false;	// 왼발도 한번 더 디뎌야함
	}
	else
	{
		ASSERT(!currFootIsR);

		// 다음 타입에 왼발로 시작하는 세그먼트가 있으면 그거 사용
		if(segmentL(state)!=-1)
			curState=state;
		
		out=segmentL(curState);
		currFootIsR=true;
	}

	static int consecutiveRun=0;
	if(prevState==curState && curState==RUN)
	{
		consecutiveRun++;
		if(consecutiveRun>MAX_CONSECUTIVE_RUN)
		{
			curState=RUN_AFTER;
			out=segmentR(curState);
			currFootIsR=true;			
		}
	}
	else
		consecutiveRun=0;

	

	if(curState!=STOP)
		cout << typeName[curState].ptr() <<endl;

	ASSERT(out!=-1);
	return mGraph->mGraph.findNode(out);
}

nodeS TRCwin::directControl()
{
	if(m_pMotion->numFrames()==0)
		return getState(STOP);
	else{
		if(isJump == true)
			return getState(JUMP);
		else if(isAttack == true)
			return getState(ATTACK);
		else if(isXrun == true)
			return getState(XRUN);
		return getState(directState);
	}
}

nodeS TRCwin::indirectControl()
{
	if(m_pMotion->numFrames()==0 || isTrack == false)
		return getState(STOP);
	else{
		vector3 moDir ;
		moDir.rotate(m_pMotion->pose(playEnd()).m_aRotations[0], vector3(0,0,1));
		moDir.y=0;

		vector3 objDir = mousePos - m_pMotion->pose(playEnd()).m_aTranslations[0];
		objDir.y=0;
 
		quater q;
		q.setAxisRotation(vector3(0,1,0), moDir, objDir);
		q.align(quater(1,0,0,0));

		dist = objDir.length();
		angle = q.rotationAngleAboutAxis(vector3(0,1,0))*180.0/M_PI;

		m_real FIX = 6.0;	// 각 모션클립의 마지막 루트 오리엔테이션이 나타내는 방향과 실제 앞방향의 오차 각도를 수정하는 값

		if(curState==RUN_BEFORE)
			return getState(RUN);
		else if(isJump == true)
			return getState(JUMP);
		else if(isAttack == true)
			return getState(ATTACK);
		else if(isXrun == true)
			return getState(XRUN);
		else if(dist < STOP_DIST_THRESH){
			if(curState == STOPPING || curState == STOP)
				return getState(STOP);
			else
				return getState(STOPPING);
		}
		else if(angle+FIX > CURVE_ANGLE_THRESH){
			if(curState == RUN){
				if(angle+FIX > RUN_CURVE_ANGLE_THRESH || dist < RUN_DIST_THRESH)
					return getState(RUN_AFTER);
				else
					return getState(RUN);
			}
			else
				return getState(LEFT);
		}
		else if(angle+FIX < -CURVE_ANGLE_THRESH){
			if(curState == RUN){
				if(angle+FIX < -RUN_CURVE_ANGLE_THRESH || dist < RUN_DIST_THRESH)
					return getState(RUN_AFTER);
				else
					return getState(RUN);
			}
			else
				return getState(RIGHT);
		}
		else if(dist > RUN_DIST_THRESH){
			if(curState == WALK)
				return getState(RUN_BEFORE);  // running allowed
				//return getState(WALK); // no running allowed (for the straight-walking comparison)
			else if(curState == RUN || curState == RUN_BEFORE)
				return getState(RUN);
			else
				return getState(WALK);
		}
		else{
			if(curState == RUN)
				return getState(RUN_AFTER);
			else
				return getState(WALK);
		}
	}
}

float TRCwin::getAngle()
{
	//cout << angle << endl;
	float ret=0.0f;

	switch(curState)
	{
	case LEFT:
		if(angle > 30)
			ret = 0.3f;
		break;
	case RIGHT:
		if(angle < -30)
		ret = -0.3f;
		break;
	case RUN:
		ret = 0.1f;
	}

	return ret;
}


void TRCwin::oneStep()
{
	// 새로운 node결정.	
	nodeS prevNode=mNode.back();
	nodeS newNode;

	// 간단하게 random transition으로 구현하였음..
	//newNode=prevNode.outEdge(ran%prevNode.outdeg()).target();

	//if(m_pMotion->numFrames()==0 || isTrack == false)
	//	newNode = mGraph->findSegment(3558);
	//else
	//	newNode = mGraph->findSegment(3652);

	if(isDirect == true)
		newNode = directControl();
	else
		newNode = indirectControl();

	/*  실제로는 루프를 돌면서 유저 입력에 가장 알맞은 노드를 골라야함
	nodeS v;
	newNode=prevNode.outEdge(0).target();
	TGL_forall_adj_nodes(v, prevNode)
		if(v is better than the previous best node)
			newNode=v; */
	
	//스티칭 시작
	if(m_pMotion->numFrames()==0)
	{
		m_pMotion->Init(mGraph->mSrcMotion, newNode.data().m_nFirstFrame, newNode.data().m_nLastFrame+1);
		if(mTerrain) m_pTerrainMotion->Init(*m_pMotion);
	}
	else{
		int firstSafe=playEnd();
		int lastSafe=m_pMotion->length()+(m_pMotion->length()-playEnd());

		if(findCheckButton("Stitch")->value())
		{
			//MotionUtil::stitchGlobalC2(firstSafe, lastSafe, *m_pMotion, mGraph->mSrcMotion, newNode.data().m_nFirstFrame, newNode.data().m_nLastFrame);

#ifdef MOTION_EDIT
			float ang = getAngle();
			Motion temp(mGraph->mSrcMotion, newNode.data().m_nFirstFrame, newNode.data().m_nLastFrame+1);
			for(int i=0; i<temp.numFrames(); i++)
				temp.pose(i).m_dq.leftMult(quater(TO_RADIAN(ang), vector3(0,1,0)));
		
			// edit temp
			MotionUtil::stitchGlobal(firstSafe, lastSafe, *m_pMotion, temp, 0, temp.numFrames()-1);
#else
			MotionUtil::stitchGlobal(firstSafe, lastSafe, *m_pMotion, mGraph->mSrcMotion, newNode.data().m_nFirstFrame, newNode.data().m_nLastFrame);
#endif

			if(mTerrain)
			{
				m_pTerrainMotion->Resize(m_pMotion->numFrames());

				for(int i=firstSafe; i<m_pMotion->numFrames(); i++)
					m_pTerrainMotion->pose(i)=m_pMotion->pose(i);
			}
		}
		else
			MotionUtil::nostitch(*m_pMotion, mGraph->mSrcMotion, newNode.data().m_nFirstFrame, newNode.data().m_nLastFrame);

		mTime.push_back(m_pMotion->length());
		int nextPlayEnd=playEnd();
		mTime.popBack();

		if(mTerrain)
		{
			
			MotionLoader& skel=m_pMotion->skeleton();

			for(int i=firstSafe; i<m_pMotion->numFrames(); i++)
				m_pTerrainMotion->pose(i)=m_pMotion->pose(i);

			// 30 frame에 한번씩은 정확히 높이를 만족시키고 그 외에는 minHeight이상만 되면 okay.
			//MotionUtil::retargetExact(sparse_position_constraints, max_kernel_size)(firstSafe, lastSafe, *m_pMotion);
			//MotionUtil::retargetHeightThr(height_constraints, kernel_size)(firstSafe, lastSafe, *m_pMotion);

			/*
			static scoped_ptr<IguanaIKSolver> iik;
			
			if(!iik.get())
			{
				iik.reset(new IguanaIKSolver(skel, mTerrain));
			}

			if(1)
			{
			   //	per-frame ik
				for(int i=firstSafe; i<m_pMotion->numFrames(); i++)
				{
					vector3N noParam;
					((MotionUtil::FullbodyIK*)iik.get())->IKsolve(m_pTerrainMotion->pose(i), noParam);				
				}
			}
			else if (0){
				// offline retarget 
				static scoped_ptr<MotionUtil::MotionRetarget> retarget;

				if(!retarget.get())
					retarget.reset(new MotionUtil::MotionRetarget(*iik));

				intvectorn constrainedFrames;
				//constrainedFrames.colon(firstSafe+30, m_pMotion->numFrames(), 30);
				constrainedFrames.colon(firstSafe, m_pMotion->numFrames(), 1);
				matrixn noParam(constrainedFrames.size(), 3);
				retarget->retarget(*m_pTerrainMotion, constrainedFrames, noParam, firstSafe, m_pMotion->numFrames()+			30);
			}
			else {
				// online retarget
				static scoped_ptr<MotionUtil::MotionRetargetOnline> retarget;

				if(!retarget.get())
					retarget.reset(new MotionUtil::MotionRetargetOnline(*m_pTerrainMotion, *iik));

				intvectorn constrainedFrames;
				constrainedFrames.colon(firstSafe+30, m_pMotion->numFrames(), 30);
				//constrainedFrames.colon(firstSafe, m_pMotion->numFrames(), 1);
				matrixn noParam(constrainedFrames.size(), 3);

				retarget->retarget(firstSafe, nextPlayEnd, constrainedFrames, noParam);


				if(mTime.size()>4) footRetarget(*m_pTerrainMotion, firstSafe, nextPlayEnd);
				RE::output("triggered", "%d:%d:%s", firstSafe, nextPlayEnd, constrainedFrames.output().ptr());
			}
			*/
		}
		else footRetarget(*m_pMotion, firstSafe, nextPlayEnd);
	}
	mNode.push_back(newNode);
	mTime.push_back(m_pMotion->length());
}

void TRCwin::footRetarget(Motion& targetMotion, int firstSafe, int nextPlayEnd)
{
	MotionLoader& skel=targetMotion.skeleton();

	static intvectorn conIndex;
	static std::vector<boost::shared_ptr<MotionUtil::FullbodyIK> > ikEachLimb;
	

	if(m_retargetEachLimb.size()==0)
	{
		std::vector<MotionUtil::Effector> effectors;
		effectors.resize(1);

		conIndex.resize(4);
		conIndex[0]=CONSTRAINT_LEFT_TOE;
		conIndex[1]=CONSTRAINT_RIGHT_TOE;
		conIndex[2]=CONSTRAINT_LEFT_FINGERTIP;
		conIndex[3]=CONSTRAINT_RIGHT_FINGERTIP;

		ikEachLimb.resize(conIndex.size());
		m_retargetEachLimb.resize(conIndex.size());
		
		for(int limb=0; limb<conIndex.size(); limb++)
		{
			effectors[0].bone=&dep_GetBoneFromCon(skel, conIndex[limb]);
			ikEachLimb[limb].reset(
				MotionUtil::createFullbodyIk_LimbIK(skel, effectors));
			m_retargetEachLimb[limb].reset(
			new MotionUtil::MotionRetargetOnline(targetMotion, *ikEachLimb[limb]));
		}
	}
	intvectorn constrainedFrames;
	vector3N constraints;

	for(int limb=0; limb<conIndex.size(); limb++)
	{
		MotionUtil::calcConstraints(targetMotion.range(firstSafe, nextPlayEnd), conIndex[limb], constrainedFrames, constraints);
		constrainedFrames+=firstSafe;

		m_retargetEachLimb[limb]->retarget(firstSafe, nextPlayEnd, constrainedFrames, matView(constraints));
	}

}

FlLayout* createIguanaWin(int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer)
{
	return new TRCwin(x,y,w,h,mp, renderer);
}

void TRCwin::transformSynthesized(matrix4 const& transf)
{
	MotionUtil::rigidTransform(*m_pMotion, transf);
	if(m_pTerrainMotion)
		MotionUtil::rigidTransform(*m_pTerrainMotion, transf);

	for(int limb=0; limb<m_retargetEachLimb.size(); limb++)
		m_retargetEachLimb[limb]->notifyTransformation(transf);
}


void TRCwin::setRootTransformation(m_real time, transf const& desiredRoot)
{
	Posture p;
	m_pMotion->samplePose(p, time);
	transf currRoot=p.rootTransformation();

	transf diff;
	diff.difference(currRoot, desiredRoot);

	ASSERT(diff.rotation.x==diff.rotation.x);
	MotionUtil::rigidTransform(*m_pMotion, diff);
}
//setting desired destination pos(temporary)
int TRCwin::setDestination(vector3 dest)
{
//	RE::output("mouseIguana", "%d %d %d %d", ev, x, y, button);
	isTrack = true;
	vector3 pos=dest;
	mousePos = pos;
	if(_drawSphere){
		Ogre::SceneNode* pNode=RE::createEntity(/*RE::generateUniqueName()*/"ppp", "sphere1010.mesh");
		RE::setEntityColor(pNode, RE::RED);
		//RE::removeEntity(pNode);
		pNode->resetToInitialState();
		pNode->scale(5.0,5.0,5.0);

		pNode->translate(ToOgre(pos));
		//RE::moveEntity(pNode, quater(1,0,0,0), pos);
	}

	if(mTerrain)
	{
		if(_drawSphere){
			Ogre::SceneNode* pNode=RE::createEntity(/*RE::generateUniqueName()*/"ppp_terrain", "sphere1010.mesh");

			//RE::removeEntity(pNode);
			pNode->resetToInitialState();
			pNode->scale(5.0,5.0,5.0);

			pos.y=getTerrainHeight(mTerrain, ToBullet(pos));
			pNode->translate(ToOgre(pos));
		}

	}
	return 1;		
}
int TRCwin::_handleRendererEvent(int ev, int button, int x, int y)
{
	switch(ev)
	{
	case FL_PUSH:
	case FL_MOVE: 
	case FL_DRAG: 
	case FL_RELEASE:
		{
			RE::output("mouseIguana", "%d %d %d %d", ev, x, y, button);
			if(ev==FL_RELEASE){
				isTrack = true;
				vector3 pos=mRenderer.screenToWorldXZPlane(x, y);
				printf("mouse pos x y z : %f %f %f \n",pos.x,pos.y,pos.z);
				mousePos = pos;
				if(_drawSphere){
					Ogre::SceneNode* pNode=RE::createEntity(/*RE::generateUniqueName()*/"ppp", "sphere1010.mesh");
					RE::setEntityColor(pNode, RE::RED);
					//RE::removeEntity(pNode);
					pNode->resetToInitialState();
					pNode->scale(5.0,5.0,5.0);

					pNode->translate(ToOgre(pos));
					//RE::moveEntity(pNode, quater(1,0,0,0), pos);
				}

				if(mTerrain)
				{
					if(_drawSphere){
						Ogre::SceneNode* pNode=RE::createEntity(/*RE::generateUniqueName()*/"ppp_terrain", "sphere1010.mesh");

						//RE::removeEntity(pNode);
						pNode->resetToInitialState();
						pNode->scale(5.0,5.0,5.0);

						pos.y=getTerrainHeight(mTerrain, ToBullet(pos));
						pNode->translate(ToOgre(pos));
					}

				}
				return 1;			
			}
		}
		break;
	case FL_KEYUP:
		{
			int key=button;
			if(key=='z'){
				isJump = true;
				return 1;
			}
			else if(key=='q'){
				isAttack = true;
				return 1;
			}
			else if(key=='e'){
				isXrun = true;
				return 1;
			}
			else if(key=='w'){
				directState = WALK;
				return 1;
			}
			else if(key=='a'){
				directState = LEFT;
				return 1;
			}
			else if(key=='x'){
				directState = STOP;
				return 1;
			}
			else if(key=='d'){
				directState = RIGHT;
				return 1;
			}
			else if(key=='2'){
				directState = RUN;
				return 1;
			}
		}
	}
	return 0;
}
