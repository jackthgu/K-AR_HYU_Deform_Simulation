#include "stdafx.h"
#include "MainLib/OgreFltk/Mesh.h"
#include "BaseLib/math/Operator.h"
#include "BaseLib/math/Operator_NR.h"
#include "AsRigidAsPossible.h"

// Sorkine et al. Laplacian Surface Editing - without linear approximation.
// linear approximation didn't work for me.
class MeshLaplacian3D: public SurfaceDeformer
{
	OBJloader::EdgeConnectivity* mEC;

	// 변수 개수: N - 각 차원을 따로 품.
	std::vector<intvectorn> i_union_n;
	std::vector<vector3> delta;

public:
	MeshLaplacian3D(OBJloader::Mesh const& mesh): SurfaceDeformer(mesh){ mEC=new OBJloader::EdgeConnectivity(mesh); }
	~MeshLaplacian3D(){delete mEC;}

	virtual void calcLocalCoords();

	// solve step
	virtual void solve(intvectorn const& conIndex, vector3N const& conPos, matrixn& deformed);
};

SurfaceDeformer* createSimpleDeformer(OBJloader::Mesh const& mesh)
{
	return new MeshLaplacian3D(mesh);
}

void MeshLaplacian3D::calcLocalCoords()
{
	int n=mMesh.numVertex();
	delta.resize(n);
	i_union_n.resize(n);
	for(int i=0; i<n; i++)
	{
		
		int nae=mEC->numAdjEdges(i);

		double invDegree=1.0/(double)nae;
		vector3 v;
		i_union_n[i].setSize(0);
		i_union_n[i].reserve(nae+1);
		for(int a=0; a<nae+1; a++)
		{
			if(a==0)
			{
				v=mMesh.getVertex(i);
				delta[i]=v;
				i_union_n[i].push_back(i);
			}
			else
			{
				int adj=mEC->getAdjVertex(i,a-1);
				v=mMesh.getVertex(adj);
				delta[i]-=v*invDegree;
				i_union_n[i].push_back(adj);
			}
		}
	}
}

#include "gmm.h"
void MeshLaplacian3D::solve(intvectorn const& conIndex, vector3N const& conPos, matrixn& deformed)
{
	
	int numVar=mMesh.numVertex();
	int numCon=conIndex.size();
	deformed.setSize(mMesh.numVertex(),3);

	for(int dim=0; dim<3;dim++)
	{
		SparseQuadraticFunctionHardCon h(numVar, numCon);

		for(int i=0; i<conIndex.size(); i++)
			h.addCon(1, 1.0, conIndex[i], -1.0*conPos[i][dim]);

		vectorn value;
		
		for(int i=0; i<mMesh.numVertex(); i++)
		{
			intvectorn& index=i_union_n[i];
			double invDeg=1.0/double(index.size()-1);

			value.resize(index.size()+1);
			
			// x coordinates
			value.setAllValue(0);
			value[0]-=1.0;
			for(int nn=1; nn<index.size(); nn++)
				value[nn]+=invDeg;

			value.back()=delta[i][dim];
			
			h.addSquared(index, value);
		}

		gmm::wsmatrixn A;
		vectorn b,x;
		h.buildSystem(A, b);
		h.solve(A,b,x);

		for(int i=0; i<mMesh.numVertex(); i++)
		{
			deformed[i][dim]=x[i];
		}
	}
}


// Sorkine et al. AsRigidAsPossible
class SurfaceLaplacian:public SurfaceDeformer
{
	
	OBJloader::EdgeConnectivity* mEC;
	std::vector<intvectorn> neighbors;
	std::vector<matrix4> R;
	std::vector<vectorn> wij;
public:
	SurfaceLaplacian(OBJloader::Mesh const& mesh): SurfaceDeformer(mesh){ mEC=new OBJloader::EdgeConnectivity(mesh); }
	~SurfaceLaplacian(){delete mEC;}

	virtual void calcLocalCoords();

	// solve step
	virtual void solve(intvectorn const& conIndex, vector3N const& conPos, matrixn& deformed);

	void optimizeR(matrixn const& deformed);
};

SurfaceDeformer* createAsRigidAsPossibleDeformer(OBJloader::Mesh const& mesh)
{
	return new SurfaceLaplacian(mesh);
}

static m_real cotan_weight(vector3 const& v1, vector3 const& v2, vector3 const& v3)
{
	vector3 a,b,c;
	m_real clen;


	a.sub(v2, v1);
	b.sub(v3, v1);
	c.cross(a, b);

	clen = c.length();

	if (clen == 0.0f)
	{
		ASSERT(0);
		return 0.0f;
	}

	m_real w=(a%b)/clen;
	return ABS(w);
}
void SurfaceLaplacian::calcLocalCoords()
{
	int n=mMesh.numVertex();

	neighbors.resize(n);
	wij.resize(n);
	R.resize(n);

	vectorn cotangentWeight;
	cotangentWeight.setSize(mMesh.numFace());

	int vi[3];
	for(int i=0; i<mMesh.numFace(); i++)
	{
		vi[0]=mMesh.getFace(i).vertexIndex(0);
		vi[1]=mMesh.getFace(i).vertexIndex(1);
		vi[2]=mMesh.getFace(i).vertexIndex(2);
		cotangentWeight[i]=cotan_weight(mMesh.getVertex(vi[0]),mMesh.getVertex(vi[1]),mMesh.getVertex(vi[2]));
		ASSERT(cotangentWeight[i]==cotangentWeight[i]);
	}

	for(int i=0; i<n; i++)
	{
		int nae=mEC->numAdjEdges(i);
		neighbors[i].reserve(nae);
		neighbors[i].setSize(0);
		wij[i].resize(nae);
//#define USE_UNIFORM_WEIGHT
#ifdef USE_UNIFORM_WEIGHT
		wij[i].setAllValue(1.0/double(nae));	//uniform weights
#else
		wij[i].setAllValue(0.0);
#endif
		
		for(int a=0; a<nae; a++)
		{
			OBJloader::EdgeConnectivity::edge e=mEC->getAdjEdge(i,a);
#ifndef USE_UNIFORM_WEIGHT
			for(int ee=0; ee<e.numFaceEdge; ee++)
				wij[i][a]+=cotangentWeight[e.faceEdge[ee].faceIndex];
#endif
			//wij[i][a]=rand()%4+1;
			neighbors[i].push_back(mEC->getAdjVertex(i,a));
		}
		R[i].identity();
	}
}

#include "gmm.h"
void SurfaceLaplacian::solve(intvectorn const& conIndex, vector3N const& conPos, matrixn& deformed)
{
	int numVar=mMesh.numVertex();
	int numCon=conIndex.size();

	deformed.setSize(mMesh.numVertex(),3);

	
	for(int iter=0; iter<4; iter++)
	{
		for(int dim=0; dim<3; dim++)
		{
			SparseQuadraticFunctionHardCon h(numVar, numCon);

			for(int i=0; i<conIndex.size(); i++)
				h.addCon(1, 1.0, conIndex[i], -1.0*conPos[i][dim]);

			for(int i=0; i<mMesh.numVertex(); i++)
			{
				for(int j=0; j<neighbors[i].size(); j++)
				{
					vector3 Rot_Pij;
					Rot_Pij.sub(mMesh.getVertex(i), mMesh.getVertex(neighbors[i][j]));
					Rot_Pij.leftMult(R[i]);
					h.addSquaredWeighted(wij[i][j], 2, 1.0, i, -1.0, neighbors[i][j], -1.0*Rot_Pij[dim]);
				}
			}

			gmm::wsmatrixn A;
			vectorn b,x;
			h.buildSystem(A, b);
			h.solve(A,b,x);
			
			for(int i=0; i<mMesh.numVertex(); i++)
			{
				deformed[i][dim]=x[i];
				ASSERT(deformed[i][dim]==deformed[i][dim]);
			}
		}
		optimizeR(deformed);
	}
}

void SurfaceLaplacian::optimizeR(matrixn const& deformed)
{
	matrixn P;
	matrixn PP;
	matrixn S;
	vectorn s;
	matrixn V;
	matrixn R;
	for(int i=0; i<mMesh.numVertex(); i++)
	{
		int nvi=neighbors[i].size();

		P.setSize(3,nvi);
		PP.setSize(3,nvi);
		for(int j=0; j<nvi; j++)
		{
			vector3 eij=mMesh.getVertex(i)-mMesh.getVertex(neighbors[i][j]);
			vector3 eijp=deformed.row3(i)-deformed.row3(neighbors[i][j]);
			P.column(j).assign(eij*wij[i][j]);
			PP.column(j).assign(eijp);
		}

		S.multABt(P,PP);
		m::SVdecompose(S,s,V);

		matrixn& U=S;
		
		R.multABt(V,U);
		if(m::determinant(R)<0)
		{
			U.column(s.argMin())*=-1;
			R.multABt(V,U);
			ASSERT(m::determinant(R)>=0);
		}
		this->R[i].setValue(R[0][0], R[0][1], R[0][2], 
					R[1][0], R[1][1], R[1][2], 
					R[2][0], R[2][1], R[2][2]); 
	}
}
