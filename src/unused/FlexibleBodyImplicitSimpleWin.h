#pragma once

class MotionPanel;
class FltkRenderer;
#include "SimulatorImplicit.h"

#include "../../MainLib/OgreFltk/OgreMotionLoader.h"
//#include "../MainLib/OgreFltk/AnimationObject.h"
#include "ElasticBody/DeformableBody.h"
#include "ElasticBody/MeanValueWeights.h"

namespace FlexibleBodyImplicitSimple
{
	extern double simulationFrameRate;
}

FlLayout* createFlexibleBodyImplicitSimpleWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer);

class FlexibleBodyImplicitSimpleWin : public FlLayout, public FrameMoveObject
{
public:	// public only to luabind

	//////////////////////////////////////////////////////////////
	// Bullet data
	//////////////////////////////////////////////////////////////

	SimulatorImplicit mSimulator;

	Motion* mMotion;
	OBJloader::Mesh mTargetMesh;
	SkinnedMeshLoader* mMeshAnimation;

public:
	FlexibleBodyImplicitSimpleWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer);
	~FlexibleBodyImplicitSimpleWin (void);

	void createTargetMesh(int iframe, double scale_factor, vector3 const& initialPosition);

	MotionPanel& m_motionPanel;
	FltkRenderer& mRenderer;

	// layout callback
	virtual void onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData);

	// FrameMoveObject
	virtual int FrameMove(float fElapsedTime);

	// LUA interface functions.
	virtual void show();
	virtual void hide();
};
