//================================================================================
//         SURFACE
//================================================================================

#pragma once


//#include <vector>
//#include <string>
#include "../Mesh_old.h"
#include "TetraMesh.h"
//#include "../BaseLib/math/tvector.h"

//=============================================================
//                 EmbeddedSurface
//=============================================================
class EmbeddedSurface
{
public:
	class NodeSurf 
	{
		vector3& _x() { return _pMesh->_mesh.getVertex(_vi).pos;}
	public:
		_quadd phi;
		_quadi tetraIndex;	// (x,v,a) = sum_j phi[j] * (tetraIndex[j])->(x,v,a), (j=0,1,2,3)

		EmbeddedSurface* _pMesh;
		int _vi;
		
		NodeSurf();
		~NodeSurf() {}

		bool isDynamic()	{ return tetraIndex[0]!=-1;}
		bool findMapping(TetraMeshLoader::TetraMesh const& m);	// finds tetraIndex and calculates phi and set bDynamic = true

		void updatePosition(TetraMeshLoader::TetraMesh const& m);		// update x with phi and tetraIndex[]->x 
	};

	// node
	std::vector<NodeSurf> _nodes;	
	MeshLoader::Mesh _mesh;
	
	std::vector<std::string> strExtraInfo;			// extra information for obj export

public:
	EmbeddedSurface();
	~EmbeddedSurface() {}

public:
	int getNumNode() { return _nodes.size(); }
	inline vector3 nodePos(int vi)	{ return _mesh.getVertex(vi).pos;}
	int getNumFace() { return _mesh.numFace(); }
	inline const int* meshIndex(int fi)	{ return _mesh.getFace(fi).vertexIndex;}

	bool loadMesh(const char *file_name_);			// load surface mesh
	bool loadExtraInfo(char *file_name_);		// load extra information file

	void exportOBJ(char *file_name_);

	bool setDynamicNodeMapping(TetraMeshLoader::TetraMesh const& m);	

	void updateNodePosition(TetraMeshLoader::TetraMesh const& m);							
	
	void clearNodeForceAll();

	bool insideSurface(const vector3 &x_);				// return true if x_ is located inside the surface.
	bool distanceFromSurface(const vector3 &x_, double *pDist_ = NULL);		// return true if x_ is located inside the surface.

public:
	bool _check_mesh_index();
	intvectorn _find_neighbor_faces(int idx_);				// return indices of the neighbor faces of the node, Nodes[idx_]
	intvectorn _find_neighbor_faces(int idxs_, int idxe_);	// return indices of the neighbor faces of the edge, (Nodes[idxs_],Nodes[idxe_])
};
