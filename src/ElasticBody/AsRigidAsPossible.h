#pragma once
namespace MeshLoader
{
	class Mesh;
	class EdgeConnectivity;
}

class SurfaceDeformer
{
protected:
	OBJloader::Mesh const& mMesh;
public:
	SurfaceDeformer(OBJloader::Mesh const& mesh): mMesh(mesh){}
	virtual ~SurfaceDeformer(){}
	virtual void calcLocalCoords()=0;
	virtual void solve(intvectorn const& conIndex, vector3N const& conPos, matrixn& deformed)=0;
};

// Sorkine et al. AsRigidAsPossible
SurfaceDeformer* createAsRigidAsPossibleDeformer(OBJloader::Mesh const& mesh);
// Rotation non-invariant!
SurfaceDeformer* createSimpleDeformer(OBJloader::Mesh const& mesh);


