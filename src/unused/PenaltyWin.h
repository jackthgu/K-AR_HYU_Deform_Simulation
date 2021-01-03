#pragma once

class MotionPanel;
class FltkRenderer;
#include "SimulatorImplicit.h"

#include "../../MainLib/OgreFltk/OgreMotionLoader.h"
//#include "../MainLib/OgreFltk/AnimationObject.h"
#include "ElasticBody/DeformableBody.h"
#include "ElasticBody/MeanValueWeights.h"
#include "../../PhysicsLib/ScriptBaseWin.h"

namespace FlexibleBodyImplicitSimple
{
	extern double simulationFrameRate;
}

FlLayout* createFlexibleBodyImplicitSimplePenaltyWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer);

class FlexibleBodyImplicitSimplePenaltyWin : public ScriptBaseWin
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
	FlexibleBodyImplicitSimplePenaltyWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer);
	~FlexibleBodyImplicitSimplePenaltyWin (void);

	virtual void initLuaEnvironment();
	void loadIguanaMotion();
	void firstInit();
	void startWorld();

	void createTargetMesh(int iframe, double scale_factor, vector3 const& initialPosition);

	virtual void show();
	virtual void hide();
};
