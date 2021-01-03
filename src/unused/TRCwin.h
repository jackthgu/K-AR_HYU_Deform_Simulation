#ifndef TRC_WIN_H_
#define TRC_WIN_H_
#pragma once

class MotionPanel;
class btRigidBody;
#include "BaseLib/utility/TGL.h"
#include "Ogre_ShapeDrawer.h"
#include <boost/shared_ptr.hpp>
#include "retarget.h"
#include "BaseLib/math/optimize.h"
#include "BaseLib/motion/Motion.h"
#include "MainLib/OgreFltk/FlLayout.h"
#include "MainLib/OgreFltk/FltkRenderer.h"
#include "MainLib/OgreFltk/pldprimskin.h"
#include "MainLib/OgreFltk/timesensor.h"
#include "BaseLib/motion/FullbodyIK.h"

namespace Iguana
{

	struct Segment
	{
		Segment():m_nFirstFrame(-1), m_nLastFrame(-1){}
		Segment(int first, int last): m_nFirstFrame(first), m_nLastFrame(last){}
		~Segment(){}
		int m_nFirstFrame;
		int m_nLastFrame;
		int length() const	{ return m_nLastFrame-m_nFirstFrame;}
	};

	typedef TGL::node<Segment> nodeS;

	enum {STOP, STOPPING, LEFT, RIGHT, WALK, RUN_BEFORE, RUN, RUN_AFTER, JUMP, ATTACK, XRUN, NUM_TYPE};

	class MotionGraph 
	{        
		std::vector<TGL::node_struct*> mSegmentByFrame;
	public:	
		MotionGraph (const Motion& mot, bitvectorn const& segmentation);

		Motion mSrcMotion;

		TGL::graph<Segment> mGraph;

		// returns segment node (which is a node in mSegmentGraph. firstFrame<=time<lastFrame)
		nodeS findSegment(int time);

		
	};

	class TRCwin: public FlLayout, public FltkRenderer::Handler, public PLDPrimSkin::DrawCallback, public FrameMoveObject, public FrameSensor::Trigger 
	{
	public:
		bool _drawSphere;
		bool _autoConnect;
		TRCwin(int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer);
		~TRCwin(void);

		MotionPanel& m_motionPanel;
		FltkRenderer& mRenderer;

		std::vector<boost::shared_ptr<MotionUtil::MotionRetargetOnline> > m_retargetEachLimb;
		btRigidBody* mTerrain;
		Iguana::MotionGraph* mGraph;
		Ogre_ShapeDrawer m_shapeDrawer;

		vector3 mousePos;
		m_real dist;
		m_real angle;

		bool currFootIsR;
		

		TStrings typeName;
		// size: NUM_TYPE
		intvectorn segmentR;	// 해당 type의 오른 뒷발로 시작하는 세그먼트.
		intvectorn segmentL;	// 해당 type의 왼 뒷발로 시작하는 세그먼트.

		
		bool isTrack;
		bool isAttack;
		bool isJump;
		bool isXrun;
		bool isDirect;
		bool synthesizing;
		int curState;
		int directState;

		void singleFrame();
		// layout callback
		virtual void onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData);

		// PLDPrimSkin::DrawCallback
		//virtual void draw(const Motion& mot, int iframe);
		virtual void draw(const Posture& posture, const MotionLoader& skeleton);

		// FrameMoveObject
		virtual int FrameMove(float fElapsedTime);

		// FrameSensor::Trigger 
		virtual void Triggered(FrameSensor* pTimer);

		// FltkRenderer::Handler
		void setAutoConnect(bool );
		virtual int	handleRendererEvent(int ev) ;
		int setDestination(vector3 dest);
		int	_handleRendererEvent(int ev, int button, int x, int y) ;
		void transformSynthesized(matrix4 const& transf);

		void show();
		void hide();
		nodeS directControl();
		nodeS indirectControl();
		nodeS getState(int state);
		float getAngle();

		intvectorn mTime;
		std::vector<Iguana::nodeS> mNode;
		Motion* m_pMotion;	//!< output motion (synthesize result)
		Motion* m_pTerrainMotion;
		PLDPrimSkin* m_pSkin;
		PLDPrimSkin* m_pTerrainSkin;

		FrameSensor* mTimer;

		// i==0 일경우 마지막 타임스텝, i==-1인경우 그 바로 이전 타임스텝..
		int length(int i=0) const	{ return (mTime.size()+i>0)?mTime[mTime.size()-1+i]:0;}
		int playEnd(int i=0) const 	
		{	// online 생성.
			return MAX(length(i-1)+2, length(i)-20);
		}

		void oneStep();	// 한 세그먼트를 생성해서 m_pMotion에 이어붙인다.
		
		float mLastDrawn;
		void setRootTransformation(m_real time, transf const& rf);
		void footRetarget(Motion& targetMotion, int firstSafe, int nextPlayEnd);
	};
}

FlLayout* createIguanaWin(int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer);
btRigidBody* createTerrain(const char* filename, vector3 const& pos, int sizeX, int sizeY, m_real width, m_real height, m_real heightMax);
#endif
