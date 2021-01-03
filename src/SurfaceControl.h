
#ifndef SURFACECONTROL_H_
#define SURFACECONTROL_H_

#include <MainLib/OgreFltk/OgreMotionLoader.h>
#include <BaseLib/motion/FullbodyIK_MotionDOF.h>
class SurfaceControl
{
	double entityScale_per_skinScale;
	SkinnedMeshLoader mOrigLoader;
	VRMLloader* mLoader; // for IK
	OBJloader::Mesh mMesh;
	SkinnedMeshFromVertexInfo *mSkinningInfo;
	std::vector<SkinnedMeshFromVertexInfo::VertexInfo  >  info; 
	vector3 mOrigOffset;
	Posture mPose0;
	vectorn mPose;
	std::vector<MotionUtil::Effector> mEffectors; // unused
	MotionUtil::FullbodyIK_MotionDOF* solver;
	public:

		
		SurfaceControl( const char* fbxFile, double _entityScale_per_skinScale, const vector3& initialRootPos);
		virtual ~SurfaceControl( );
		inline OBJloader::Mesh & getCurrMesh() { return mMesh;}
		inline const OBJloader::Mesh & getCurrMesh() const { return mMesh;}


		// step 1
		void clearConstraints() { info.resize(0);}
		int numCon() const { return (int)info.size();}
		
		// step 2
		// utility functions
		void addConstraints( int vertexIndex)
		{
			info.resize(info.size()+1); auto& i=info.back();
			i.treeIndices=mSkinningInfo->treeIndices(vertexIndex); i.localpos=mSkinningInfo->localPos(vertexIndex); i.weights=mSkinningInfo->weights(vertexIndex);
		}
		void addConstraints( int v1, int v2, int v3, vector3 const & baryCoeffs)
		{
			info.resize(info.size()+1); auto& i=info.back();
			mSkinningInfo->getVertexInfo(v1, v2, v3, baryCoeffs, i.treeIndices, i.localpos, i.weights);
		}
		vector3 getConPos(int conIndex) const;

		// updates currMesh from surface constraints stored in info.
		void solveIK(vector3N const& desired_pos);

};

#endif
