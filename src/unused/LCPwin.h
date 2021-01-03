#pragma once

class MotionPanel;
class FltkRenderer;
#include "TRL/DynamicsSimulator_TRL_massSpring.h"

#include "../../MainLib/OgreFltk/OgreMotionLoader.h"
//#include "../MainLib/OgreFltk/AnimationObject.h"
#include "ElasticBody/DeformableBody.h"
#include "ElasticBody/MeanValueWeights.h"
#include "../../PhysicsLib/ScriptBaseWin.h"


FlLayout* createFlexibleBodyImplicitSimpleLCPWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer);

class FlexibleBodyImplicitSimpleLCPWin : public ScriptBaseWin
{
public:	// public only to luabind

	//////////////////////////////////////////////////////////////
	// Bullet data
	//////////////////////////////////////////////////////////////

	TRL::DynamicsSimulator_TRL_massSpring mSimulator;

	Motion* mMotion;
	OBJloader::Mesh mTargetMesh;
	SkinnedMeshLoader* mMeshAnimation;

public:
	FlexibleBodyImplicitSimpleLCPWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer);
	~FlexibleBodyImplicitSimpleLCPWin (void);

	virtual void initLuaEnvironment();
	void loadIguanaMotion();
	void firstInit();
	void startWorld(double timestep);

	void createTargetMesh(int iframe, double scale_factor, vector3 const& initialPosition);

	virtual void show();
	virtual void hide();
};
