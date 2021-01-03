#include "stdafx.h"
#include "SurfaceControl.h"
#include <BaseLib/motion/VRMLloader.h>

SurfaceControl::~SurfaceControl( )
{
	delete mSkinningInfo;
}

#include <MainLib/WrapperLua/LUAwrapper.h>
#include <MainLib/OgreFltk/FlLayout.h>
#include <MainLib/WrapperLua/luna_baselib.h>
#include <MainLib/WrapperLua/luna_mainlib.h>
void Register_baselib(lua_State*L);
void Register_mainlib(lua_State*L);
	SurfaceControl::SurfaceControl(const char* fbxFile, double _entityScale_per_skinScale, const vector3& initialRootPos):
	mOrigLoader(fbxFile, false, false),
	entityScale_per_skinScale(_entityScale_per_skinScale),
	mSkinningInfo(NULL)
{

#if OGRE_VERSION_MINOR>=12

	// create mOrigLoader 
	mOrigLoader.printHierarchy();
	mOrigLoader.gotoBindPose();

	
	// copy the current skeleton pose to the skin
	Posture newPose;
	mOrigLoader.getPose(newPose);


	// step 1. copy the bad skeleton
	
	//mLoader=mOrigLoader.toVRMLloader(); // c++로 포팅이 귀찮아서 그냥 루아함수 호출함.
	LUAwrapper L;
	{
		Register_baselib(L.L);
		Register_mainlib(L.L);
		L.dofile("config.lua");
		L.dostring("package.path=package.path..\";../../taesooLib/Samples/scripts/?.lua\"");
		L.dostring("package.path=package.path..\";../../taesooLib/Samples/scripts/RigidBodyWin/?.lua\"");
		L.dostring("package.path=package.path..\";../../taesooLib/Samples/classification/lua/?.lua\"");

		L.getglobal("MotionUtil", "exportVRMLforRobotSimulation");
		L.push<MotionLoader>(mOrigLoader);
		L<<std::string("_temp_cpp.wrl")<<std::string("unknown");
		L.call(3,0);
	}
	mLoader=new VRMLloader("_temp_cpp.wrl");

	mLoader->removeAllRedundantBones();// -- clean up the skeleton
	mLoader->setPose(newPose);
	mLoader->setCurPoseAsInitialPose(); // adjust skeleton so that the current pose becomes the identity pose (all local joint orientations are identity quaternions)
	mOrigOffset=mLoader->bone(1).getOffsetTransform().translation;
	mLoader->bone(1)._getOffsetTransform().translation.zero();
	//MotionUtil.removeSlidingJoints(mLoader); // c++로 포팅이 귀찮아서 그냥 루아함수 호출함.
	{
		L.getglobal("MotionUtil", "removeSlidingJoints");
		L.push<MotionLoader>(mLoader);
		L.call(1,0);
	}


	// you can further simplify mLoader as long as all the joint positions are kept.
	// for example, you can change rotational channels so that knee cannot twist, and so on. (see skeletonEditor_GUI.lua)
	// after that, create a mapping (PT) between the two skeletons 

	PoseTransfer2 PT(mLoader, &mOrigLoader);
	auto &mSkinLoader=mOrigLoader;

	int numVertex=mSkinLoader.getCurrMesh().numVertex();
	
	mSkinningInfo=new SkinnedMeshFromVertexInfo(mSkinLoader);

	for (int i=0; i< numVertex;i++){
		intvectorn& treeIndices=mSkinningInfo->treeIndices(i);
		vector3N& localpos=mSkinningInfo->localPos(i);
		
		// localpos needs to be converted from mSkinLoader space to mLoader space.
		for (int j=0;j< localpos.size();j++){
			const transf& frameA=mSkinLoader.bone(treeIndices(j)).getFrame();
			const transf& frameB=mLoader->bone(treeIndices(j)).getFrame();
			localpos(j)=(frameB.inverse()*((frameA*localpos(j))*(entityScale_per_skinScale)));
		}
	}


	mLoader->getPose(mPose0);
	mPose0.m_aTranslations[0]=initialRootPos;
	// now let's attach an IK solver and IK handles (empty)
	mEffectors.resize(0);
	solver=MotionUtil::createFullbodyIk_MotionDOF_MultiTarget_lbfgs(mLoader->dofInfo);
	solver->setParam("damping_weight", 0,0.01);

	mLoader->setPose(mPose0);
	mLoader->getPoseDOF(mPose);
	mMesh=mSkinLoader.getCurrMesh(); // copy -- scale이 달라서 vertex 위치 업데이트 필요.
	mSkinningInfo->calcVertexPositions(*mLoader, mMesh);
#endif
}
vector3 SurfaceControl::getConPos(int conIndex) const
{
	vector3 vpos(0,0,0);
	auto& weights=info[conIndex].weights;
	auto& localpos=info[conIndex].localpos;
	auto& treeIndices=info[conIndex].treeIndices;

	for (int j=0;j< weights.size();j++)
		vpos+=(mLoader->bone(treeIndices(j)).getFrame()*localpos(j))*weights(j);
	return vpos;
}

void SurfaceControl::solveIK(vector3N const& desired_pos){

	solver->_changeNumEffectors(0);
	solver->_changeNumConstraints(numCon());

	for (int ii=0; ii<numCon(); ii++){
		auto& c=info[ii];
		solver->_setSkinningConstraint(ii, c.treeIndices, c.localpos, c.weights, desired_pos(ii)) ;
	}

	solver->_effectorUpdated();
	vector3N footPos;
	solver->IKsolve(mPose, footPos);
	mLoader->setPoseDOF(mPose);
	mSkinningInfo->calcVertexPositions(*mLoader, mMesh);

}
