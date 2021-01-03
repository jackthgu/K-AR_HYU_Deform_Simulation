//================================================================================
//         DEFORMABLE BODY
// 
//                                                               junggon@gmail.com
//================================================================================

#ifndef _DEFORMABLE_BODY_
#define _DEFORMABLE_BODY_

#include "EmbeddedSurface.h"
#include "TetraMesh.h"
//=============================================================
//                 DeformableBody
//=============================================================
class DeformableBody 
{
public:
	TetraMeshLoader::TetraMesh ebody;
	EmbeddedSurface surf_embedded;

public:
	DeformableBody();
	~DeformableBody() {}

public:
	bool loadVolumeMesh(const char *file_name_);
	bool loadSurfaceMesh(const char *file_name_);
	bool syncSurface();

	// ** file format **
	//
	// (file_name_nv_)
	// ---------------------------
	//	n			// n = getNumNode()
	//	nv[0]		// nv[i] = material volume corresponding to ebody's i-th node
	//	...
	//	nv[n-1]
	// ---------------------------
	//
	// (file_name_mw_)
	// ---------------------------
	//	m			// m = getNumMesh()
	//	mw[0]		// nv[i] = material volume ratio in ebody's i-th mesh (if the tetrahedron is fully filled in, then nv[i] = 1 ideally.)
	//	...
	//	mw[m-1]
	// ---------------------------

	//bool loadMass(char *file_name_nv_, double density_);	// set ebody.Nodes[].mass
	//bool loadMeshWeight(char *file_name_mw_);				// set ebody.elastic_force.m_mesh_weight

	//void calc_NodalVolume_MeshWeight(char *file_name_nv_, char *file_name_mw_, int n_ = 10);			
											// calculate nodal volume and mesh weight by voxelizing the body
											// n_ = smallest number of grid lines on (x,y,z)-axes to voxelize the body

	void updateSurfaceNodePosition();		// update surf_embedded.Nodes[].x with current ebody.Nodes[].x
	
	void fitVolumeMesh();
	void discardNode(int idx_);
};

#endif

