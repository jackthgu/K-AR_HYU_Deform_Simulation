#pragma once

class MotionPanel;
class FltkRenderer;
class FlLayout;
#include "../../MainLib/OgreFltk/FlLayout.h"
#include "../../MainLib/OgreFltk/AnimationObject.h"
#include "../../MainLib/OgreFltk/FltkRenderer.h"
#include "SimulatorImplicit.h"

namespace FELearning
{
	extern double simulationFrameRate;
	extern bool showTargetMesh;
	extern bool matchPose;
	extern bool performIK;
	extern m_real scale_factor;
}


FlLayout* createFELearningWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer);

///CcdPhysicsDemo shows basic stacking using Bullet physics, and allows toggle of Ccd (using key '1')
class FELearningWin : public FlLayout, public FrameMoveObject
{
public:	// public only to luabind
// private:

	//////////////////////////////////////////////////////////////
	// Bullet data
	//////////////////////////////////////////////////////////////

	SimulatorImplicit mSimulator;

	//////////////////////////////////////////////////////////////
	// My data
	//////////////////////////////////////////////////////////////
	Motion* mMotion;
	OBJloader::Mesh* mTargetMesh;
	SkinnedMeshLoader* mMeshAnimation;
	vectorn mOffset;
public:
	FELearningWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer);
	~FELearningWin (void);

	//void updateTargetMesh(Motion& mot, m_real prevTime, m_real time);

	MotionPanel& m_motionPanel;
	FltkRenderer& mRenderer;

	// layout callback
	virtual void onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData);

	// FrameMoveObject
	virtual int FrameMove(float fElapsedTime);

	
	void	keyboardCallback(unsigned char key, int x, int y);

	// LUA bind
	void createTargetMesh(int iframe, double scale_factor);
	void updateTargetMesh(double frame);
	void stepSimulation(double step);
	void renderScene();
	void setOffset(vectorn const& offset);
	void measureError(OBJloader::Mesh const& mesh1, OBJloader::Mesh const& mesh2, vectorn & offset);

	virtual void show()	{	mRenderer.ogreRenderer().fixedTimeStep(true); FlLayout::show();}
	virtual void hide()	{	mRenderer.ogreRenderer().fixedTimeStep(false); FlLayout::hide();}

};
