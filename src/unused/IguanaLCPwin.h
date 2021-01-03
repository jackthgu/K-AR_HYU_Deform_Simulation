#pragma once

class MotionPanel;
class FltkRenderer;
//#include "SimulatorImplicit.h"
#include "TRL/DynamicsSimulator_TRL_massSpring.h"

#include "../../MainLib/OgreFltk/OgreMotionLoader.h"
//#include "../MainLib/OgreFltk/AnimationObject.h"
#include "ElasticBody/DeformableBody.h"
#include "ElasticBody/MeanValueWeights.h"
#include "../../PhysicsLib/ScriptBaseWin.h"

namespace Iguana
{
	class TRCwin;
}



FlLayout* createIguanaLCPWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer);

// bullet 디펜던시는 제거 가능한 상태임. TRCwin에만 살짝 남아있는데, 쉽게 제거 가능.
class IguanaLCPWin : public ScriptBaseWin
{
public:	// public only to luabind
	double renderingFrameRate;//240; 30;
	double simulationFrameRate;
	bool matchPose;
	bool performIK;	
	// for lua wrapping.
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


	//SimulatorImplicit mSimulator;
	TRL::DynamicsSimulator_TRL_massSpring mSimulator;

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
	
	IguanaLCPWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer);
	~IguanaLCPWin (void);

	void createTargetMesh(int iframe, double scale_factor, vector3 const& initialPosition);
	void updateTargetMesh(Motion& mot, m_real prevTime, m_real time);
	void updateTargetMeshTRCwin(Motion& mot, m_real prevTime, m_real time);

	double getTerrainHeight(vector3 const& pos);
	virtual void initLuaEnvironment();

	// LUA interface functions.
	
	
	void setTRCwinTerrain(const char* filename, vector3 const& pos, int sizeX, int sizeY, m_real width, m_real height, m_real heightMax);
	void setSkeletonVisible(bool bVisible);
	void setOption(const char* title, bool value);
	virtual void show();
	virtual void hide();

	int _frameMove(float fElapsedTime);
	void showHighMesh();
	void updateHighMesh(double scale_factor=0.01);
	std::vector<MotionUtil::Effector> ee;
	Ogre::SceneNode* mNode;
};
