#pragma once

class MotionPanel;
class FltkRenderer;
#include "SimulatorImplicit.h"

#include "../../MainLib/OgreFltk/OgreMotionLoader.h"
//#include "../MainLib/OgreFltk/AnimationObject.h"
#include "ElasticBody/DeformableBody.h"
#include "ElasticBody/MeanValueWeights.h"
#include "../../PhysicsLib/ScriptBaseWin.h"

namespace Iguana
{
	class TRCwin;
}



FlLayout* createFlexibleBodyImplicitWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer);



///CcdPhysicsDemo shows basic stacking using Bullet physics, and allows toggle of Ccd (using key '1')
class FlexibleBodyImplicitWin : public ScriptBaseWin
{
public:	// public only to luabind
	double renderingFrameRate;//240; 30;
	double simulationFrameRate;
	bool matchPose;
	bool performIK;	
	// for lua wrapping.
	int trcWin_setDestination(vector3 dest);
	bool isTRCwin_synthesizing();
	void trcWin_singleFrame();
	int trcWin_lastDrawn() ;
	Motion& trcWin_terrainMotion() ;
	Motion& trcWin_motion() ;
	bool trcWin_hasTerrain();
	int trcWin_handleRendererEvent(const char* evs, int button, int x, int y);
	void trcWin_transformSynthesized(matrix4 const& tf);
	void trcWin_setDrawSphere(bool mode);
	void stepSimulation(double step) ;

	void performIguanaIK(Posture& pose, vector3N const& con);
// private:

	void start();
	Iguana::TRCwin* mTRCwin;

	//////////////////////////////////////////////////////////////
	// Bullet data
	//////////////////////////////////////////////////////////////

	SimulatorImplicit mSimulator;

	Motion* mMotion;
	//PLDPrimCustumSkin* mSkin;
	OBJloader::Mesh mTargetMesh;
	OBJloader::Mesh mPrevTargetMesh;
	//OBJloader::GetMeshAnimation mMeshAnimation;
	SkinnedMeshLoader* mMeshAnimation;

	OBJloader::Mesh mIguanaHighMesh;
	scoped_ptr<OBJloader::MeshToEntity> mIguanaHighMeshViewer;
	scoped_ptr<MeshDeformer> mDeformer;

	//DeformableBody mTargetVolumeMesh;


public:
	void _ctor();
	
	FlexibleBodyImplicitWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer);
	~FlexibleBodyImplicitWin (void);

	void createTargetMesh(int iframe, double scale_factor, vector3 const& initialPosition);
	void updateTargetMesh(Motion& mot, m_real prevTime, m_real time);
	void updateTargetMeshTRCwin(Motion& mot, m_real prevTime, m_real time);

	double getTerrainHeight(vector3 const& pos);
	virtual void initLuaEnvironment();

	// LUA interface functions.
	
	
	void setTRCwinTerrain();
	void setSkeletonVisible(bool bVisible);
	void setOption(const char* title, bool value);
	virtual void show();
	virtual void hide();

	int _frameMove(float fElapsedTime);
	void showHighMesh();
	void updateHighMesh();
	std::vector<MotionUtil::Effector> ee;
};
