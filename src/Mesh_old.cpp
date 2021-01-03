#include "stdafx.h"
#include "Mesh_old.h"
#include "BaseLib/math/Operator.h"
#include "BaseLib/image/Image.h"
#include "BaseLib/image/ImagePixel.h"

#ifndef NO_OGRE
#include "MainLib/OgreFltk/OgreMotionLoader.h"
#include <OgreMeshManager.h>
#include <OgreSubMesh.h>
#include <OgreRoot.h>
#include <OgreRenderSystem.h>
//#include "../Ogre/PLDPrimCustumSkin.h"
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>
#if OGRE_VERSION_MINOR>=12
#include <OgreMesh.h>
void _setMaterial(Ogre::SimpleRenderable* ptr, const char* name);
void MeshLoader::TriangleMesh::setMaterial(const char* mat) { _setMaterial(this, mat); }
void MeshLoader::MeshLineStrip::setMaterial(const char* mat) { _setMaterial(this, mat); }
#endif
#endif
#include <fstream>

using namespace MeshLoader;
using namespace std;
Ogre::Vector3 ToOgre(const vector3& v);

void scale_Mesh(MeshLoader::Mesh& mesh, m_real scaleFactor)
{
	for(int i=0; i<mesh.numVertex(); i++)
		mesh.m_arrayVertex[i].pos*=scaleFactor;
}






vector3 MeshLoader::Mesh::calcMeshCenter() const
{
	vector3 center(0,0,0);

	for(int i=0; i<numVertex(); i++)
		center+=getVertex(i).pos;

	center/=(double)numVertex();
	return center;
}

#ifndef NO_OGRE
void GetMeshAnimation::create(Motion const& mot, const char* meshFile, Mesh& mesh, int iframe)
{
	ASSERT(0);
	/*
	if(m_data) 
		delete (PoseTransfer*)m_data;
	if(mSkin)
		delete mSkin;

	mSkin=new SkinnedMeshLoader (meshFile);
	mSkin->mMesh.mergeDuplicateVertices();

	m_mot=(Motion*)&mot;
	// initial bone끼리 똑같은 모양일꺼다.
	m_mot->skeleton().UpdateInitialBone();
	mSkin->UpdateInitialBone();

	m_data=(void*)new PoseTransfer(&m_mot->skeleton(), mSkin);
	
	((PoseTransfer*)m_data)->setTargetSkeleton(m_mot->pose(iframe));
	mesh=mSkin->mMesh.mesh;
	mSkin->retrieveAnimatedMesh(mesh);

	if(mScaleFactor!=1.0)	scale_Mesh(mesh, mScaleFactor);*/
}

GetMeshAnimation::~GetMeshAnimation()
{
	delete mSkin;
	delete (PoseTransfer*)m_data;
}

void GetMeshAnimation::update(MeshLoader::Mesh& mesh, Posture const& pose)
{
	ASSERT(0);
	/*	
	((PoseTransfer*)m_data)->setTargetSkeleton(pose);
	mSkin->retrieveAnimatedMesh(mesh);
	if(mScaleFactor!=1.0)	scale_Mesh(mesh, mScaleFactor);*/
}

void GetMeshAnimation::update(MeshLoader::Mesh& mesh, int iframe)
{
	update(mesh, m_mot->pose(iframe));
}
#endif

bool MeshLoader::Mesh::loadObj(const char* filename)
{
	CTextFile file;
	if(!file.OpenReadFile(filename))
		return false;

	TString token;
	TString currMode="v";
	vector3N pos;
	int count=0;
	while(1)
	{
		token=file.GetToken();
		if(token.length()==0)
			break;
		if(token=="v")
		{
			double x=atof(file.GetToken());
			double y=atof(file.GetToken());
			double z=atof(file.GetToken());
			pos.pushBack(vector3(x,y,z));
		}
		else 
		{
			if(currMode=="v")
			{
				m_arrayVertex.resize(pos.size());

				for(int i=0; i<pos.size(); i++)
					m_arrayVertex[i].pos=pos[i];
				count=0;

				m_arrayFace.resize(0);
			}

			if(token=="f")
			{
				MeshLoader::Face f;
				int i=atoi(file.GetToken());
				int j=atoi(file.GetToken());
				int k=atoi(file.GetToken());
				f.setIndex(i-1,j-1,k-1);
				m_arrayFace.push_back(f);
				currMode="f";
			}
		}
	}
}

vector3& MeshLoader::Mesh::getFaceCorner(int iFace, int icorner) 
{
	return getVertex(getFace(iFace).vi(icorner)).pos;
}
vector3 MeshLoader::Mesh::calcFaceCenter(int i) const
{
	vector3 const& v1=getVertex(getFace(i).vi(0)).pos;
	vector3 const& v2=getVertex(getFace(i).vi(1)).pos;
	vector3 const& v3=getVertex(getFace(i).vi(2)).pos;

	return (v1+v2+v3)/3.0;
}
vector3 MeshLoader::Mesh::calcFaceNormal(int i) const
{
	vector3 const& v1=getVertex(getFace(i).vi(0)).pos;
	vector3 const& v2=getVertex(getFace(i).vi(1)).pos;
	vector3 const& v3=getVertex(getFace(i).vi(2)).pos;

	vector3 d1, d2;
	d1.difference(v1,v2);
	d2.difference(v2,v3);
	vector3 n;
	n.cross(d1,d2);
	n.normalize();
	return n;
}

void MeshLoader::Mesh::transform(matrix4 const& b)
{
	quater q;
	q.setRotation(b);

	for(int i=0; i<m_arrayVertex.size(); i++)
	{
		m_arrayVertex[i].pos.leftMult(b);
		m_arrayVertex[i].normal.rotate(q);
	}
}

void MeshLoader::Mesh::loadMesh(int numVertex, int numFace, double * vertices, int indices[][3])
{
	m_arrayVertex.resize(numVertex);
	for(int i=0; i<numVertex; i++)
		m_arrayVertex[i].pos=vector3(vertices[i*3], vertices[i*3+1], vertices[i*3+2]);

	m_arrayFace.resize(numFace);
	for(int i=0; i<numFace; i++)
	{
		m_arrayFace[i].vertexIndex[0]=indices[i][0];
		m_arrayFace[i].vertexIndex[1]=indices[i][1];
		m_arrayFace[i].vertexIndex[2]=indices[i][2];
	}
}

bool MeshLoader::Mesh::loadMesh(const char* file_name_)
{
	int i, n, m;
	vector3 x_tmp;
	ifstream fin;

	fin.open(file_name_);

	if ( !fin.is_open() ) return false;

	// get the number of nodes and meshes
	fin >> n >> m;	

	m_arrayVertex.resize(n);
	m_arrayFace.resize(m);

	// get initial node positions from file
	for (i=0; i<n; i++) {
		// node position in {mesh file reference frame}
		fin >> x_tmp[0] >> x_tmp[1] >> x_tmp[2];

		// position of nodes in {body}
		m_arrayVertex[i].pos=x_tmp;
	}

	// get mesh indexes from file
	for (i=0; i<m; i++) {
		int* mesh_index=m_arrayFace[i].vertexIndex;
		fin >> mesh_index[0] >> mesh_index[1] >> mesh_index[2];
	}

	fin.close();
	return true;
}

bool MeshLoader::Mesh::saveObj(const char* filename, bool vn, bool vt)
{
	std::ofstream fout;

	// ---------------- read mesh information from file -------------------

	fout.open(filename);

	if ( !fout.is_open() ) return false;

	// get the number of nodes and mesh elements
	int n=m_arrayVertex.size();
	int m=m_arrayFace.size();

	// get initial node positions from file
	for (int i=0; i<n; i++) {
		vector3& x_tmp=m_arrayVertex[i].pos;

		// node position in {mesh file reference frame}
		fout << "v " <<x_tmp.x <<" "<< x_tmp.y <<" "<< x_tmp.z<<endl;
	}

	if(vn)
	{
		for (int i=0; i<n; i++) {
			vector3& x_tmp=m_arrayVertex[i].normal;

			// node position in {mesh file reference frame}
			fout << "vn " <<x_tmp.x <<" "<< x_tmp.y <<" "<< x_tmp.z<<endl;
		}
	}

	if(vt)
	{
		for (int i=0; i<n; i++) {
			vector2& x_tmp=m_arrayVertex[i].texCoord;

			// node position in {mesh file reference frame}
			fout << "vt " <<x_tmp(0) <<" "<< x_tmp(1) <<endl;
		}
	}
	// get mesh indexes from file
	for (int i=0; i<m; i++) {
		fout << "f "<< m_arrayFace[i].vi(0) +1;
		if(vn)
			fout << "/"<< m_arrayFace[i].vi(0) +1;
		if(vt)
			fout << "/"<< m_arrayFace[i].vi(0) +1;

		fout << " " << m_arrayFace[i].vi(1) +1;
		if(vn)
			fout << "/"<< m_arrayFace[i].vi(1) +1;
		if(vt)
			fout << "/"<< m_arrayFace[i].vi(1) +1;

		fout << " " << m_arrayFace[i].vi(2) +1;
		if(vn)
			fout << "/"<< m_arrayFace[i].vi(2) +1;
		if(vt)
			fout << "/"<< m_arrayFace[i].vi(2) +1;
		fout<<endl;
	}

	fout.close();
	return true;
}

bool MeshLoader::Mesh::saveMesh(const char *file_name_)
{
	int n, m;
	vector3 x_tmp;
	std::ofstream fout;

	// ---------------- read mesh information from file -------------------

	fout.open(file_name_);

	if ( !fout.is_open() ) return false;

	// get the number of nodes and mesh elements

	n=m_arrayVertex.size();
	m=m_arrayFace.size();
	fout << n <<" "<< m<<endl;	

	// get initial node positions from file
	for (int i=0; i<n; i++) {
		x_tmp=m_arrayVertex[i].pos;

		// node position in {mesh file reference frame}
		fout << x_tmp.x <<" "<< x_tmp.y <<" "<< x_tmp.z<<endl;

	}

	// get mesh indexes from file
	for (int i=0; i<m; i++) {
		fout << m_arrayFace[i].vi(0) <<" "<< m_arrayFace[i].vi(1) <<" "<< m_arrayFace[i].vi(2)<<endl;
	}

	fout.close();
	return true;
}

void MeshLoader::Mesh::saveMesh(const char* identifier, const char* filename)
{
	FILE* file=fopen(filename, "wt");

	TString id=identifier;
	TString uid=id.toUpper();
	
	fprintf(file,"#ifndef %s_MESH_H_\n", uid.ptr());
	fprintf(file,"#define %s_MESH_H_\n\n", uid.ptr());
	fprintf(file,"const int %s_NUM_TRIANGLES =%d;\n", uid.ptr(), numFace());
	fprintf(file,"const int %s_NUM_VERTICES = %d;\n", uid.ptr(), numVertex());
	fprintf(file,"const int %s_NUM_INDICES  = %s_NUM_TRIANGLES * 3;\n", uid.ptr(), uid.ptr());

	{ // pos
		fprintf(file,"static double gVertices%s[%s_NUM_VERTICES * 3] = {\n", id.ptr(), uid.ptr());
	
		for(int i=0; i<numVertex()-1; i++)
		{
			vector3 p=m_arrayVertex[i].pos;
			fprintf(file,"%f, %f, %f, \n", p.x, p.y, p.z);
		}
		vector3 p=m_arrayVertex[numVertex()-1].pos;
		fprintf(file,"%f, %f, %f}; \n", p.x, p.y, p.z);
	}

	{ // normal
		fprintf(file,"static double gNormals%s[%s_NUM_VERTICES * 3] = {\n", id.ptr(), uid.ptr());

		for(int i=0; i<numVertex()-1; i++)
		{
			vector3 p=m_arrayVertex[i].normal;
			fprintf(file,"%f, %f, %f, \n", p.x, p.y, p.z);
		}
		vector3 p=m_arrayVertex[numVertex()-1].normal;
		fprintf(file,"%f, %f, %f}; \n", p.x, p.y, p.z);

		fprintf(file,"static int gIndices%s[%s_NUM_TRIANGLES][3] = {\n", id.ptr(), uid.ptr());
	}

	{	// face
		for(int i=0; i<numFace()-1; i++)
		{
			int* vi=m_arrayFace[i].vertexIndex;
			fprintf(file,"{%d, %d, %d}, \n", vi[0], vi[1], vi[2]);
		}
		int* vi=m_arrayFace[numFace()-1].vertexIndex;
		fprintf(file,"{%d, %d, %d}};\n#endif \n", vi[0], vi[1], vi[2]);
	}

	fclose(file);
}

void MeshLoader::Mesh::copyFrom(Mesh const& otherMesh)
{
	m_arrayVertex.resize(otherMesh.numVertex());
	m_arrayFace.resize(otherMesh.numFace());

	for(int i=0; i<otherMesh.numFace(); i++)
		m_arrayFace[i]=otherMesh.m_arrayFace[i];

	for(int i=0; i<otherMesh.numVertex(); i++)
		m_arrayVertex[i]=otherMesh.m_arrayVertex[i];
}

void MeshLoader::Mesh::eliminateSharedVertex(Mesh const& otherMesh)
{
	m_arrayVertex.resize(otherMesh.numFace()*3);
	m_arrayFace.resize(otherMesh.numFace());
	for(int i=0; i<otherMesh.numFace(); i++)
	{
		for(int j=0; j<3; j++)
		{
			m_arrayFace[i].vertexIndex[j]=i*3+j;
			m_arrayVertex[i*3+j].pos=otherMesh.m_arrayVertex[otherMesh.m_arrayFace[i].vi(j)].pos;
			m_arrayVertex[i*3+j].normal=otherMesh.m_arrayVertex[otherMesh.m_arrayFace[i].vi(j)].normal;
			m_arrayVertex[i*3+j].texCoord=otherMesh.m_arrayVertex[otherMesh.m_arrayFace[i].vi(j)].texCoord;
			m_arrayVertex[i*3+j].color=otherMesh.m_arrayVertex[otherMesh.m_arrayFace[i].vi(j)].color;			
		}		
	}
}

void MeshLoader::Mesh::calculateVertexNormal()
{

	vector3N faceNormal;
	faceNormal.setSize(numFace());

	for(int i=0; i<numFace(); i++)
	{
		faceNormal(i)=calcFaceNormal(i);
	}

	for(int i=0; i<numVertex(); i++)
		getVertex(i).normal.setValue(0,0,0);

	for(int i=0; i<numFace(); i++)
	{
		getVertex(getFace(i).vi(0)).normal+=faceNormal(i);
		getVertex(getFace(i).vi(1)).normal+=faceNormal(i);
		getVertex(getFace(i).vi(2)).normal+=faceNormal(i);
	}

	for(int i=0; i<numVertex(); i++)
		getVertex(i).normal.normalize();
}


void MeshConnectivity::setAdj(FaceEdge const& fe1, FaceEdge const& fe2)
{
	adjFaceIndex(fe1.faceIndex, fe1.vertexIndex)=fe2.faceIndex;
	adjFaceVertexIndex(fe1.faceIndex, fe1.vertexIndex)=fe2.vertexIndex;

	adjFaceIndex(fe2.faceIndex, fe2.vertexIndex)=fe1.faceIndex;
	adjFaceVertexIndex(fe2.faceIndex, fe2.vertexIndex)=fe1.vertexIndex;
}

FaceEdge MeshConnectivity::prev(FaceEdge const& fe) const
{
	FaceEdge out(adjFace(fe));
	if(!out.isNull())
		out.vertexIndex=(out.vertexIndex+1)%3;
	
	ASSERT(out.isNull() || out.source(mMesh)==fe.source(mMesh));
	return out;
}

FaceEdge MeshConnectivity::next(FaceEdge const& fe) const
{
	FaceEdge out(adjFace(FaceEdge(fe.faceIndex, (fe.vertexIndex+2)%3)));
	ASSERT(out.isNull() || out.source(mMesh)==fe.source(mMesh));
	return out;
}

MeshConnectivity::MeshConnectivity(Mesh & mesh):mMesh(mesh){ EdgeConnectivity edges(mesh); _init(mesh, edges);}
MeshConnectivity::MeshConnectivity(Mesh & mesh, EdgeConnectivity const & edges):mMesh(mesh){	_init(mesh, edges);}
		


void MeshConnectivity::_init(Mesh & mesh, EdgeConnectivity const& edges)
{
	
	vertexToFaceEdge.resize(mesh.numVertex());

	adjFaceIndex.setSize(mesh.numFace(), 3);
	adjFaceIndex.setAllValue(-1);
	adjFaceVertexIndex.setSize(mesh.numFace(), 3);
	adjFaceVertexIndex.setAllValue(-1);

	for(int i=0; i<mesh.numFace(); i++)
	{
		vertexToFaceEdge[mesh.getFace(i).vi(0)]=FaceEdge(i,0);
		vertexToFaceEdge[mesh.getFace(i).vi(1)]=FaceEdge(i,1);
		vertexToFaceEdge[mesh.getFace(i).vi(2)]=FaceEdge(i,2);
	}

	mesh.isBoundaryVertex.resize(mesh.numVertex());
	mesh.isBoundaryVertex.clearAll();
	for(int i=0; i<edges.numEdges(); i++)
	{
		EdgeConnectivity::edge& edge=edges.getEdge(i);
		
		if(edge.numFaceEdge==2)
		{
			setAdj(edge.faceEdge[0],edge.faceEdge[1]);
		}
		else
		{
			mesh.isBoundaryVertex.setAt(edges.source(i));
			mesh.isBoundaryVertex.setAt(edges.target(i));
		}
	}

	if(mesh.isBoundaryVertex.size())
	{
		for(int i=0; i<mesh.numVertex(); i++)
		{
			if(mesh.isBoundaryVertex(i))
			{
				FaceEdge e=vertexToFaceEdge[i];

				while(!prev(e).isNull())
					e=prev(e);

				vertexToFaceEdge[i]=e;
			}
		}
	}
}

#ifndef NO_OGRE
TriangleMesh ::TriangleMesh ()
:MeshEntity()
{
	mRenderOp.vertexData=NULL;
	mRenderOp.indexData= NULL;
	mRenderOp.operationType = Ogre::RenderOperation::OT_TRIANGLE_LIST;
	mRenderOp.useIndexes = true; 

	this->setMaterial("green");
}

TriangleMesh ::~TriangleMesh ()
{
	delete mRenderOp.indexData;
	delete mRenderOp.vertexData; 

}

void TriangleMesh ::firstInit()
{
	const bool useNormal=true;
	// Initialization stuff 
	delete mRenderOp.vertexData;
	mRenderOp.vertexData=NULL;
	
	delete mRenderOp.indexData;
	mRenderOp.indexData=NULL;

	setVisible(true);

	if(!mRenderOp.vertexData)
	{
		mRenderOp.vertexData= new Ogre::VertexData(); 
		mRenderOp.vertexData->vertexStart = 0; 
		mRenderOp.vertexData->vertexCount = numVertex();

		mRenderOp.indexData=new Ogre::IndexData();
		mRenderOp.indexData->indexStart=0;
		mRenderOp.indexData->indexCount=numFace()*3;

		// define the vertex format
		Ogre::VertexDeclaration* vertexDecl = mRenderOp.vertexData->vertexDeclaration;
		Ogre::VertexBufferBinding *bind = mRenderOp.vertexData->vertexBufferBinding; 

		size_t currOffset = 0;
		// positions
		vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
		currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);

		if(useNormal)
		{
			// normals
			vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT3, Ogre::VES_NORMAL);
			currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT3);
		}
		// two dimensional texture coordinates
		vertexDecl->addElement(0, currOffset, Ogre::VET_FLOAT2, Ogre::VES_TEXTURE_COORDINATES, 0);
		currOffset += Ogre::VertexElement::getTypeSize(Ogre::VET_FLOAT2);

		Ogre::HardwareVertexBufferSharedPtr vbuf =
			Ogre::HardwareBufferManager::getSingleton().createVertexBuffer( 
			vertexDecl->getVertexSize(0), 
			mRenderOp.vertexData->vertexCount, 
			Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY); 

		bind->setBinding(0, vbuf); 

		// allocate index buffer
		mRenderOp.indexData->indexBuffer = 
			Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
			Ogre::HardwareIndexBuffer::IT_16BIT, 
			mRenderOp.indexData->indexCount, 
			Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);


	}



	// Drawing stuff 

	vector3 min, max;

	Ogre::HardwareVertexBufferSharedPtr vbuf =mRenderOp.vertexData->vertexBufferBinding->getBuffer(0);
	Ogre::Real *prPos = (Ogre::Real*)(vbuf ->lock(Ogre::HardwareBuffer::HBL_DISCARD)); 

	Ogre::HardwareIndexBufferSharedPtr iBuf = mRenderOp.indexData->indexBuffer;
	unsigned short* pIndices = static_cast<unsigned short*>(iBuf->lock(Ogre::HardwareBuffer::HBL_DISCARD));


	min.x=min.y=min.z=FLT_MAX;
	max.x=max.y=max.z=-FLT_MAX;
	for(int i=0; i<numVertex(); i++)
	{
		vector3& pp=m_arrayVertex[i].pos;
		*prPos++ = pp.x; 
		*prPos++ = pp.y; 
		*prPos++ = pp.z; 

		min.x=std::min(pp.x, min.x);
		min.y=std::min(pp.x, min.y);
		min.z=std::min(pp.x, min.z);

		max.x=std::max(pp.x, max.x);
		max.y=std::max(pp.x, max.y);
		max.z=std::max(pp.x, max.z);

		if(useNormal)
		{
			*prPos++= m_arrayVertex[i].normal.x;
			*prPos++= m_arrayVertex[i].normal.y;
			*prPos++= m_arrayVertex[i].normal.z;
		}

		*prPos++ = m_arrayVertex[i].texCoord(0);
		*prPos++ = m_arrayVertex[i].texCoord(1);
	}

	for(int i=0; i<numFace(); i++)
	{
		*pIndices++=(unsigned short)m_arrayFace[i].vertexIndex[0];
		*pIndices++=(unsigned short)m_arrayFace[i].vertexIndex[1];
		*pIndices++=(unsigned short)m_arrayFace[i].vertexIndex[2];
	}
	vbuf->unlock(); 
	iBuf->unlock();

	//mBox.setExtents(ToOgre(min), ToOgre(max)); // 2D폴리곤에서 이상동작함.. 여유공간을 두어야할듯.
	mBox.setInfinite();
}

void TriangleMesh::update()
{
	firstInit();
}
#endif
EdgeConnectivity::EdgeConnectivity(Mesh const& mesh)
{
	// construct mesh graph
	mMeshGraph.clear();

	for(int i=0; i<mesh.numVertex(); i++)
	{
		nodeT v=mMeshGraph.newNode();
		v->vertexIndex=i;
	}

	int error=0;
	for(int i=0; i<mesh.numFace(); i++)
	{
		for(int fe=0; fe<3; fe++)
		{
			int v1=mesh.getFace(i).vi(fe);
			int v2=mesh.getFace(i).vi((fe+1)%3);

			ASSERT(v1>=0 && v1<mesh.numVertex());
			ASSERT(v2>=0 && v2<mesh.numVertex());

			nodeT s=mMeshGraph.findNode(v1);
			nodeT t=mMeshGraph.findNode(v2);
			edgeT e=mMeshGraph.findEdge(s, t);
			if(!e)
				e=mMeshGraph.newEdge(s,t);

			int nfe=e.data().numFaceEdge;

			if(nfe<2)
			{
				e.data().faceEdge[nfe].faceIndex=i;
				e.data().faceEdge[nfe].vertexIndex=fe;
				e.data().numFaceEdge++;
			}
			else
			{
				Msg::print("Edge(%d,%d): the number of face edges is larger than 2\n", v1, v2);
				error++;
			}

		}
	}
	if(error)
		Msg::msgBox("Warning! total %d edges have more than 2 face edges",error);
}
  
int EdgeConnectivity::numEdges() const	{return mMeshGraph.numEdges();}
EdgeConnectivity::edge& EdgeConnectivity::getEdge(int i) const { return mMeshGraph.findEdge(i).data();}		
 

int FaceEdge::source(Mesh & mesh) const				
{ 
	return mesh.getFace(faceIndex).vi(vertexIndex);
}
int FaceEdge::target(Mesh & mesh) const	
{ 
	return mesh.getFace(faceIndex).vi((vertexIndex+1)%3);
}
int FaceEdge::cross(Mesh & mesh) const	
{ 
	return mesh.getFace(faceIndex).vi((vertexIndex+2)%3);
}


Vertex& FaceEdge::getSource(Mesh & mesh)
{ 
	return mesh.getVertex(source(mesh));}
Vertex& FaceEdge::getTarget(Mesh & mesh)
{ 
	return mesh.getVertex(target(mesh));}
bool FaceEdge::isNull() const	
{ 
	return faceIndex==-1;}
bool FaceEdge::operator==(const FaceEdge& other)
{ 
	return faceIndex==other.faceIndex && vertexIndex==other.vertexIndex; }
bool FaceEdge::operator!=(const FaceEdge& other)
{ 
	return !((*this)==other); }


int EdgeConnectivity::source(int iEdge) const	
{
	return mMeshGraph.findEdge(iEdge).v1().index();
}

int EdgeConnectivity::target(int iEdge) const	
{ 
	return mMeshGraph.findEdge(iEdge).v2().index();
}

int EdgeConnectivity::numAdjEdges(int vertexIndex) const
{
	return mMeshGraph.findNode(vertexIndex).degree();
}

int EdgeConnectivity::getAdjVertex(int vertexIndex, int i) const
{
	nodeT source=mMeshGraph.findNode(vertexIndex);
	return source.edge(i).target(source).index();
}

EdgeConnectivity::edge& EdgeConnectivity::getAdjEdge(int vertexIndex, int i) const
{
	TUGL::edge_struct* e=mMeshGraph.findNode(vertexIndex).getEdgePtr(i);
	return *((EdgeConnectivity::edge*)(e->_data));
}

bool EdgeConnectivity::isConnected(int vertex1, int vertex2)
{
	return mMeshGraph.findEdge(mMeshGraph.findNode(vertex1), mMeshGraph.findNode(vertex2))!=NULL;
}

#ifndef NO_OGRE
MeshLineStrip::MeshLineStrip()
:MeshEntity()
{
	edges=NULL;
   mRenderOp.vertexData = NULL;
   this->setMaterial("solidblue");    
}

MeshLineStrip::~MeshLineStrip()
{
	delete edges;
}

void MeshLineStrip::_createGraph()
{	
	// construct mesh graph
	delete edges;
	edges=new EdgeConnectivity(*this);
}

void MeshLineStrip::firstInit()
{
	_createGraph();

	update();
}

void minVec3(vector3& a, vector3 const& b)
{
	a.x=MIN(a.x, b.x);
	a.y=MIN(a.y, b.y);
	a.z=MIN(a.z, b.z);
}

void maxVec3(vector3& a, vector3 const& b)
{
	a.x=MAX(a.x, b.x);
	a.y=MAX(a.y, b.y);
	a.z=MAX(a.z, b.z);
}

void MeshLineStrip::update()
{
	// when vertex position is changed you can call update();
	delete mRenderOp.vertexData;
	mRenderOp.vertexData= new Ogre::VertexData(); 

   mRenderOp.indexData = 0; 
   mRenderOp.vertexData->vertexCount = edges->mMeshGraph.numEdges()*2; 
   mRenderOp.vertexData->vertexStart = 0; 
   mRenderOp.operationType = Ogre::RenderOperation::OT_LINE_LIST;//, OT_LINE_STRIP 
   mRenderOp.useIndexes = false; 

   if(edges->mMeshGraph.numEdges()==0)
	   return;

   Ogre::VertexDeclaration *decl = mRenderOp.vertexData->vertexDeclaration; 
   Ogre::VertexBufferBinding *bind = mRenderOp.vertexData->vertexBufferBinding; 

   decl->addElement(POSITION_BINDING, 0, Ogre::VET_FLOAT3, Ogre::VES_POSITION); 

   Ogre::HardwareVertexBufferSharedPtr vbuf = 
      Ogre::HardwareBufferManager::getSingleton().createVertexBuffer( 
         decl->getVertexSize(POSITION_BINDING), 
         mRenderOp.vertexData->vertexCount, 
         Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY); 

   bind->setBinding(POSITION_BINDING, vbuf); 

   // Drawing stuff 
   int size = edges->mMeshGraph.numEdges();

   vector3 minv(FLT_MAX, FLT_MAX, FLT_MAX);
   vector3 maxv(-FLT_MAX, -FLT_MAX, -FLT_MAX);

   Ogre::Real *prPos = (Ogre::Real*)(vbuf->lock(Ogre::HardwareBuffer::HBL_DISCARD)); 

   EdgeConnectivity::edgeT e;
   TUGL_for_all_edges(e, edges->mMeshGraph)
   {
	   for(int i=0; i<2; i++)
	   {
		   vector3 const& pp=(i==0)?m_arrayVertex[e.v1().index()].pos:m_arrayVertex[e.v2().index()].pos;
		  *prPos++ = pp.x; 
		  *prPos++ = pp.y; 
		  *prPos++ = pp.z;

		  minVec3(minv, pp);
		  maxVec3(maxv, pp);
	   } 
   }

   vbuf->unlock(); 

   mBox.setExtents(ToOgre(minv), ToOgre(maxv));
}

void MeshLineStripReduced::update()
{
	m_real thickness=2.0;

	// when vertex position is changed you can call update();
	delete mRenderOp.vertexData;
	mRenderOp.vertexData= new Ogre::VertexData(); 

	mRenderOp.indexData = 0; 
	mRenderOp.vertexData->vertexCount = numFace()*6;
	mRenderOp.vertexData->vertexStart = 0; 
	mRenderOp.operationType = Ogre::RenderOperation::OT_LINE_LIST;//, OT_LINE_STRIP 
	mRenderOp.useIndexes = false; 

	if(numFace()==0)
	   return;

	Ogre::VertexDeclaration *decl = mRenderOp.vertexData->vertexDeclaration; 
	Ogre::VertexBufferBinding *bind = mRenderOp.vertexData->vertexBufferBinding; 

	decl->addElement(POSITION_BINDING, 0, Ogre::VET_FLOAT3, Ogre::VES_POSITION); 

	Ogre::HardwareVertexBufferSharedPtr vbuf = 
	  Ogre::HardwareBufferManager::getSingleton().createVertexBuffer( 
		 decl->getVertexSize(POSITION_BINDING), 
		 mRenderOp.vertexData->vertexCount, 
		 Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY); 

	bind->setBinding(POSITION_BINDING, vbuf); 

	// Drawing stuff 

	vector3 minv(FLT_MAX, FLT_MAX, FLT_MAX);
	vector3 maxv(-FLT_MAX, -FLT_MAX, -FLT_MAX);

	Ogre::Real *prPos = (Ogre::Real*)(vbuf->lock(Ogre::HardwareBuffer::HBL_DISCARD)); 

	vector3 pos[3], offsetPos[3];

	for(int face=0; face<numFace(); face++)
	{
		for(int i=0; i<3; i++)
		{
		   pos[i]=m_arrayVertex[m_arrayFace[face].vertexIndex[i]].pos;
			  minVec3(minv, pos[i]);
			  maxVec3(maxv, pos[i]);
		}

		// 내심 구하기.
		m_real d01=pos[0].distance(pos[1]);
		m_real d12=pos[1].distance(pos[2]);
		m_real d20=pos[2].distance(pos[0]);

		vector3 center=(pos[0]*d12+pos[1]*d20+pos[2]*d01)/(d01+d12+d20);
	   
		//// scaling
		//pos[0].interpolate(0.1, pos[0], center);
		//pos[1].interpolate(0.1, pos[1], center);
		//pos[2].interpolate(0.1, pos[2], center);
		//for(int i=0; i<3; i++)
		//{
		//  *prPos++ = pos[i].x; 
		//  *prPos++ = pos[i].y; 
		//  *prPos++ = pos[i].z;
		//  *prPos++ = pos[(i+1)%3].x; 
		//  *prPos++ = pos[(i+1)%3].y; 
		//  *prPos++ = pos[(i+1)%3].z;
		//} 

		// offsetting (CCW-order를 가정하였다.)

		//                                  c thick  v1
		//       v2     v1          //       \     | 
		//		  +----+			//		  +----+
		//        |   /				//         \   |
		//        |  /				//  offset  \  |
		//        | /				//           \ |
		//        |/				//        	  \|
		//        v0				//            v0   
		vector3 normal;
		normal.cross(pos[1]-pos[0], pos[2]-pos[0]);
		normal.normalize();

		for(int i=0; i<3; i++)
		{
			vector3 v01, c0;
			v01.difference(pos[i], pos[(i+1)%3]);
			c0.difference(pos[i], center);
			c0.normalize();

			quater q;
			q.axisToAxis(v01, c0);
			m_real theta=q.rotationAngleAboutAxis(normal);

			// thickness/offset=sin theta;
			offsetPos[i]=pos[i]+c0*(thickness/sin(theta));
		}

		for(int i=0; i<3; i++)
		{
		  *prPos++ = offsetPos[i].x; 
		  *prPos++ = offsetPos[i].y; 
		  *prPos++ = offsetPos[i].z;
		  *prPos++ = offsetPos[(i+1)%3].x; 
		  *prPos++ = offsetPos[(i+1)%3].y; 
		  *prPos++ = offsetPos[(i+1)%3].z;
		}
	}

	vbuf->unlock(); 

	mBox.setExtents(ToOgre(minv), ToOgre(maxv));
}
#endif
void MeshLoader::createPlane(Mesh& mesh, int numSegX, int numSegZ, m_real sizeX, m_real sizeZ)
{
	// z|
	//  + ->x    numSegX=2, numSegZ=2, sizeX=sizeZ=2인경우.

	//  0   1   2
	//  +---+---+  2
	//  | / | / |  
	//  +---+---+  1
	//  | / | / |
	//  +---+---+  0
	mesh.m_arrayVertex.resize((numSegX+1)*(numSegZ+1));
	mesh.m_arrayFace.resize(numSegX*numSegZ*2);
	for(int i=0; i<=numSegZ; i++)
	{
		for(int j=0; j<=numSegX; j++)
		{
			int index=i*(numSegZ+1)+j;

			m_real x=sop::map((m_real)j, 0, numSegX, 0, sizeX);
			m_real z=sop::map((m_real)i, 0, numSegZ, 0, sizeZ);
			mesh.m_arrayVertex[index].pos.setValue(x,0,z);
		}
	}

	for(int i=0; i<numSegZ; i++)
	{
		for(int j=0; j<numSegX; j++)
		{
			// insert lower triangle
			int faceIndex=i*(numSegZ)+j;
			mesh.m_arrayFace[faceIndex].vertexIndex[0]=i*(numSegZ+1)+j;
			mesh.m_arrayFace[faceIndex].vertexIndex[1]=i*(numSegZ+1)+j+1;
			mesh.m_arrayFace[faceIndex].vertexIndex[2]=(i+1)*(numSegZ+1)+j+1;

			// insert upper triangle
			faceIndex+=numSegX*numSegZ;
			mesh.m_arrayFace[faceIndex].vertexIndex[0]=i*(numSegZ+1)+j;
			mesh.m_arrayFace[faceIndex].vertexIndex[1]=(i+1)*(numSegZ+1)+(j+1);
			mesh.m_arrayFace[faceIndex].vertexIndex[2]=(i+1)*(numSegZ+1)+j;
		}
	}
}

class Raw2Bytes : public _tmat<unsigned short>
{
public:
	Raw2Bytes():_tmat<unsigned short>(){}
	template <class T>
	Raw2Bytes(const T& other):_tmat<unsigned short>(other){}
};




void MeshLoader::createTerrain(Mesh& mesh, const char* filename, int sizeX, int sizeY, m_real width, m_real height, m_real heightMax, int ntexSegX, int ntexSegZ)
{
	ASSERT(sizeof(short)==2);
	Raw2Bytes image;
	image.setSize(sizeY, sizeX);
	FILE* file=fopen(filename, "rb");
	fread( (void*)(&image(0,0)), sizeof(short), sizeX*sizeY, file);
	
	fclose(file);

	/*// for my notebook, where 512 by 512 mesh doesn't load.
	sizeX=256;
	sizeY=256;*/

	
#define SAVE_BMP
#ifdef SAVE_BMP
	CImage temp;
	
	temp.Create(sizeX, sizeY);//, 24);
	CImagePixel pixel(&temp);
#endif

	mesh.m_arrayVertex.resize(sizeX*sizeY);

	for(int y=0; y<sizeY; y++)
	{
		for(int x=0; x<sizeX; x++)
		{
			int index=y*sizeX+x;
			
			m_real& tu=mesh.m_arrayVertex[index].texCoord(0);
			m_real& tv=mesh.m_arrayVertex[index].texCoord(1);

			tu=m_real(x)*m_real(ntexSegX)/m_real(sizeX-1);
			tv=m_real(y)*m_real(ntexSegZ)/m_real(sizeY-1);
			
			vector3& pos=mesh.m_arrayVertex[index].pos;
			pos.x=m_real(x)*width/m_real(sizeX-1);
			pos.z=m_real(y)*height/m_real(sizeY-1);
			pos.y=heightMax*m_real(image(y,x))/65536.0;

#ifdef SAVE_BMP
			int color=m_real(image(y,x))/65536.0*255;
			pixel.SetPixel(x, y, CPixelRGB8(color, color, color));
#endif
		}
	}

#ifdef SAVE_BMP
	temp.Save("terrain.bmp");
#endif

	int numSegX=sizeX-1;
	int numSegZ=sizeY-1;
	mesh.m_arrayFace.resize(numSegX*numSegZ*2);


	vector3N normalSum(mesh.numVertex());
	normalSum.setAllValue(vector3(0,0,0));

	intvectorn normalCount(mesh.numVertex());
	normalCount.setAllValue(0);

	for(int i=0; i<numSegZ; i++)
	{
		for(int j=0; j<numSegX; j++)
		{
			// insert lower triangle
			int faceIndex=i*(numSegX)+j;
			{
				Face& f=mesh.m_arrayFace[faceIndex];
				f.vertexIndex[0]=i*(numSegX+1)+j;
				f.vertexIndex[1]=(i+1)*(numSegX+1)+j+1;
				f.vertexIndex[2]=i*(numSegX+1)+j+1;

				vector3 v0=mesh.getVertex(f.vi(0)).pos;
				vector3 v1=mesh.getVertex(f.vi(1)).pos;
				vector3 v2=mesh.getVertex(f.vi(2)).pos;

				vector3 faceNormal;
				faceNormal.cross(v1-v0, v2-v0);
				faceNormal.normalize();

				for(int vv=0; vv<3; vv++)
				{
					normalSum[f.vi(vv)]+=faceNormal;
					normalCount[f.vi(vv)]++;
				}
			}

			// insert upper triangle
			faceIndex+=numSegX*numSegZ;
			Face& f=mesh.m_arrayFace[faceIndex];
			f.vertexIndex[0]=i*(numSegX+1)+j;
			f.vertexIndex[1]=(i+1)*(numSegX+1)+j;
			f.vertexIndex[2]=(i+1)*(numSegX+1)+(j+1);

			vector3 v0=mesh.getVertex(f.vi(0)).pos;
			vector3 v1=mesh.getVertex(f.vi(1)).pos;
			vector3 v2=mesh.getVertex(f.vi(2)).pos;

			vector3 faceNormal;
			faceNormal.cross(v1-v0, v2-v0);
			faceNormal.normalize();

			for(int vv=0; vv<3; vv++)
			{
				normalSum[f.vi(vv)]+=faceNormal;
				normalCount[f.vi(vv)]++;
			}
		}
	}

	for(int i=0; i<mesh.numVertex(); i++)
	{
		mesh.m_arrayVertex[i].normal=normalSum[i]/m_real(normalCount[i]);
		mesh.m_arrayVertex[i].normal.normalize();
	}
}

void MeshLoader::createPlane(Mesh& mesh, const char* imageFile, int numSeg, m_real size)
{
	Mesh temp;
	createPlane(temp, numSeg, numSeg, size, size);

	CImage image;
	image.Load(imageFile);

	CImagePixel pixel(&image);

	bitvectorn validVertex;
	validVertex.resize(temp.numVertex());
	validVertex.clearAll();

	m_real width=image.GetWidth()-1;
	m_real height=image.GetHeight()-1;
	for(int i=0; i<=numSeg; i++)
	{
		for(int j=0; j<=numSeg; j++)
		{
			int index=i*(numSeg+1)+j;

			m_real x=sop::map((m_real)j, 0, numSeg,0.05,0.95);	// 이미지의 바깥쪽 5프로는 버렸다.
			m_real z=sop::map((m_real)i, 0, numSeg,0.05,0.95);

			int count;
			int R=pixel.GetPixel(x*width,z*height, count).R;
			printf("%d ", R);
			if(R<128)
				validVertex.setAt(index);			
		}
		printf("\n");
	}

	// delaunay triangulation

	mesh.m_arrayVertex.resize(validVertex.count());

	int vi=0;
	for(int i=0; i<temp.numVertex(); i++)
	{
		if(validVertex[i])
		{
			mesh.m_arrayVertex[vi]=temp.m_arrayVertex[i];
			vi++;
		}
	}

	Msg::msgBox("call the following sentence outside this function");
	//Triangulation::delaunayTriangulation(mesh);	
	
}


using namespace Ogre;
#ifndef NO_OGRE
#include "OgreAxisAlignedBox.h"
#endif
// Ogre::Mesh와 MeshLoader::Mesh간의 자료교환을 담당하는 클래스.


MeshToEntity::MeshToEntity(const MeshLoader::Mesh& mesh, const char* meshId, Option option)
:mInputMesh(mesh)
{
	mSavedOption=option;
	#ifndef NO_OGRE
	if(MeshManager::getSingleton().resourceExists(meshId))
	{
		/*
		Ogre::MeshPtr pmesh=MeshManager::getSingleton().getByName(meshId);
		mMesh=(Ogre::Mesh*)pmesh.get();

		Ogre::ResourcePtr ptr=MeshManager::getSingleton().getByName(meshId);

		if(mMesh->sharedVertexData->vertexCount != mesh.numVertex())
		{
			MeshManager::getSingleton().remove(ptr);
		}
		else
			return;
		return;*/

		Ogre::ResourcePtr ptr=MeshManager::getSingleton().getByName(meshId);
		MeshManager::getSingleton().remove(ptr);
	}
	mMesh= (Ogre::Mesh*)MeshManager::getSingleton().createManual(meshId, ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME).get();

	SubMesh *pMeshVertex = mMesh->createSubMesh();

	mMesh->sharedVertexData = new VertexData();
	VertexData* vertexData = mMesh->sharedVertexData;


	// define the vertex format
	VertexDeclaration* vertexDecl = vertexData->vertexDeclaration;
	size_t currOffset = 0;
	// positions
	vertexDecl->addElement(0, currOffset, VET_FLOAT3, VES_POSITION);
	currOffset += VertexElement::getTypeSize(VET_FLOAT3);

	if(option.useNormal)
	{
		// normals
		vertexDecl->addElement(0, currOffset, VET_FLOAT3, VES_NORMAL);
		currOffset += VertexElement::getTypeSize(VET_FLOAT3);
	}
	
	if(option.useTexCoord)
	{
		// two dimensional texture coordinates
		vertexDecl->addElement(0, currOffset, VET_FLOAT2, VES_TEXTURE_COORDINATES, 0);
		currOffset += VertexElement::getTypeSize(VET_FLOAT2);
	}

	if(option.useColor)
	{
		// two dimensional texture coordinates
		vertexDecl->addElement(0, currOffset, VET_COLOUR, VES_DIFFUSE);
		currOffset += VertexElement::getTypeSize(VET_COLOUR);
	}

	HardwareVertexBuffer::Usage usage=HardwareBuffer::HBU_STATIC_WRITE_ONLY;
	HardwareVertexBuffer::LockOptions lockOption=HardwareBuffer::HBL_DISCARD;
	if(option.dynamicUpdate)
	{
		usage=HardwareBuffer::HBU_DYNAMIC ;
		lockOption=HardwareBuffer::HBL_NORMAL ;
	}

	// allocate the vertex buffer
	vertexData->vertexCount = mesh.numVertex();
	HardwareVertexBufferSharedPtr vBuf = HardwareBufferManager::getSingleton().createVertexBuffer(vertexDecl->getVertexSize(0), vertexData->vertexCount, usage, false);
	VertexBufferBinding* binding = vertexData->vertexBufferBinding;
	binding->setBinding(0, vBuf);

	unsigned char* vertex=static_cast<unsigned char*>(vBuf->lock(lockOption));
	//float* prPos = static_cast<float*>(vBuf->lock(lockOption));

	// allocate index buffer
	pMeshVertex->indexData->indexCount = mesh.numFace()*3;
	pMeshVertex->indexData->indexBuffer = HardwareBufferManager::getSingleton().createIndexBuffer(HardwareIndexBuffer::IT_16BIT, pMeshVertex->indexData->indexCount, usage, false);
	HardwareIndexBufferSharedPtr iBuf = pMeshVertex->indexData->indexBuffer;
	unsigned short* pIndices = static_cast<unsigned short*>(iBuf->lock(lockOption));

	
	// Drawing stuff 
	Ogre::AxisAlignedBox box;

	const VertexDeclaration::VertexElementList& elemList = vertexDecl->getElements();

	RenderSystem* rs = Root::getSingleton().getRenderSystem();
	ASSERT(rs);

	for(int i=0; i<mesh.numVertex(); i++)
	{

		for (VertexDeclaration::VertexElementList::const_iterator elem = elemList.begin();
			elem != elemList.end(); ++elem)
		{
			float* pFloat=0;
			RGBA* pRGBA=0;
			switch(elem->getType())
			{
			case VET_FLOAT1:
			case VET_FLOAT2:
			case VET_FLOAT3:
				elem->baseVertexPointerToElement(vertex, &pFloat);
				break;
			case VET_COLOUR:
			case VET_COLOUR_ABGR:
			case VET_COLOUR_ARGB:
				elem->baseVertexPointerToElement(vertex, &pRGBA);
				break;
			default:
				// nop ?
				break;
			};

			switch(elem->getSemantic())
			{
			case VES_POSITION:
				{
					vector3 const& pp=mesh.m_arrayVertex[i].pos;
					*pFloat++ = pp.x;
					*pFloat++ = pp.y;
					*pFloat++ = pp.z;
					box.merge(Ogre::Vector3(pp.x, pp.y, pp.z));
				}
				break;
			case VES_NORMAL:
				ASSERT(option.useNormal);
				*pFloat++ = mesh.m_arrayVertex[i].normal.x;
				*pFloat++ = mesh.m_arrayVertex[i].normal.y;
				*pFloat++ = mesh.m_arrayVertex[i].normal.z;
				break;
			case VES_TEXTURE_COORDINATES:
				ASSERT(option.useTexCoord);
				ASSERT(VertexElement::getTypeCount(elem->getType())==2);
				*pFloat++ = mesh.m_arrayVertex[i].texCoord(0);
				*pFloat++ = mesh.m_arrayVertex[i].texCoord(1);
				break;
			case VES_DIFFUSE:
				ASSERT(option.useColor);
				{
					const CPixelRGBA& c=mesh.m_arrayVertex[i].color;
					Ogre::ColourValue colour(
						float(c.R)/float(255), 
						float(c.G)/float(255), 
						float(c.B)/float(255), 
						float(c.A)/float(255));
					rs->convertColourValue(colour, pRGBA++);
				}
				break;
			default:
				// nop ?
				break;
			};
		}

		vertex += vBuf->getVertexSize();

/*
		vector3 const& pp=mesh.m_arrayVertex[i].pos;
		*prPos++ = pp.x; 
		*prPos++ = pp.y; 
		*prPos++ = pp.z; 

		box.merge(Ogre::Vector3(pp.x, pp.y, pp.z));

		if(option.useNormal)
		{
			*prPos++= mesh.m_arrayVertex[i].normal.x;
			*prPos++= mesh.m_arrayVertex[i].normal.y;
			*prPos++= mesh.m_arrayVertex[i].normal.z;
		}

		if(option.useTexCoord)
		{
			*prPos++ = mesh.m_arrayVertex[i].texCoord(0);
			*prPos++ = mesh.m_arrayVertex[i].texCoord(1);
		}

		if(option.useColor)
		{
			*prPos++=
				rs = Root::getSingleton().getRenderSystem();
				if (rs)
					rs->convertColourValue(mTempVertex.colour, pRGBA++);
		}*/
	}

	for(int i=0; i<mesh.numFace(); i++)
	{
		*pIndices++=(unsigned short)mesh.m_arrayFace[i].vertexIndex[0];
		*pIndices++=(unsigned short)mesh.m_arrayFace[i].vertexIndex[1];
		*pIndices++=(unsigned short)mesh.m_arrayFace[i].vertexIndex[2];
	}
	vBuf->unlock(); 
	iBuf->unlock();

	// Generate face list
	pMeshVertex ->useSharedVertices=true;

	//mMesh->_setBounds(Ogre::AxisAlignedBox(ToOgre(min), ToOgre(max)), false);
	//mMesh->_setBoundingSphereRadius(min.distance(max)*2);
	
	mMesh->_setBounds(box);
	mMesh->_setBoundingSphereRadius(box.getMinimum().distance(box.getMaximum()));

	mMesh->load();
	
	if(option.buildEdgeList)
	{
		mMesh->buildEdgeList();
	}
#endif
}

void MeshLoader::MeshToEntity::updatePositions()
{
  #ifndef NO_OGRE
	Option& option=mSavedOption;

	const VertexElement* posElem = mMesh->sharedVertexData->vertexDeclaration->findElementBySemantic(Ogre::VES_POSITION);
	HardwareVertexBufferSharedPtr vBuf = mMesh->sharedVertexData->vertexBufferBinding->getBuffer(posElem->getSource());

	unsigned char* vertex=static_cast<unsigned char*>(vBuf->lock(HardwareBuffer::HBL_NORMAL ));

	const MeshLoader::Mesh& mesh=mInputMesh;

	float* prPos;
	// Drawing stuff 
	vector3 min, max;

	min.x=min.y=min.z=FLT_MAX;
	max.x=max.y=max.z=-FLT_MAX;

	Ogre::AxisAlignedBox box;

	for(int i=0; i<mesh.numVertex(); i++)
	{
		vector3 const& pp=mesh.m_arrayVertex[i].pos;
		posElem->baseVertexPointerToElement(vertex, &prPos);

		*prPos++ = pp.x; 
		*prPos++ = pp.y; 
		*prPos++ = pp.z; 
		
		box.merge(Ogre::Vector3(pp.x, pp.y, pp.z));

		vertex += vBuf->getVertexSize();
	}
	vBuf->unlock();

	mMesh->_setBounds(box);
	mMesh->_setBoundingSphereRadius(box.getMinimum().distance(box.getMaximum()));

	if(option.buildEdgeList)
	{
		mMesh->freeEdgeList();
		mMesh->buildEdgeList();
	}
#endif

}


void MeshLoader::MeshToEntity::updatePositionsAndNormals()
{
  #ifndef NO_OGRE
	Option& option=mSavedOption;

	updatePositions();

	if(option.useNormal)
	{

		const VertexElement* normalElem = mMesh->sharedVertexData->vertexDeclaration->findElementBySemantic(Ogre::VES_NORMAL);
		HardwareVertexBufferSharedPtr vBuf = mMesh->sharedVertexData->vertexBufferBinding->getBuffer(normalElem->getSource());

		unsigned char* vertex=static_cast<unsigned char*>(vBuf->lock(HardwareBuffer::HBL_NORMAL ));

		const MeshLoader::Mesh& mesh=mInputMesh;

		float* prnormal;
		for(int i=0; i<mesh.numVertex(); i++)
		{
			normalElem->baseVertexPointerToElement(vertex, &prnormal);

			*prnormal++= mesh.m_arrayVertex[i].normal.x;
			*prnormal++= mesh.m_arrayVertex[i].normal.y;
			*prnormal++= mesh.m_arrayVertex[i].normal.z;

			vertex += vBuf->getVertexSize();
		}
		vBuf->unlock();
	}

	if(option.buildEdgeList)
	{
		mMesh->freeEdgeList();
		mMesh->buildEdgeList();
	}
#endif
}

Ogre::Entity* MeshLoader::MeshToEntity::createEntity(const char* entityName, const char* materialName)
{
  #ifdef NO_OGRE
    return NULL;
#else
	if(RE::ogreSceneManager()->hasEntity(entityName))
		RE::ogreSceneManager()->destroyEntity(entityName);

	Ogre::Entity* entity=RE::ogreSceneManager()->createEntity(entityName, mMesh->getName());
	mEntity=entity;
	entity->setMaterialName(materialName);
	return entity;
#endif
}

#ifndef NO_OGRE
void testMeshToEntity()
{
	// options
	bool runtime_modify=true;
	bool use_stencil_shadow=true;

	if(runtime_modify && use_stencil_shadow)
	{
		// ManualObject를 사용할 수 없다.

		MeshLoader::Mesh mesh;
		mesh.m_arrayVertex.resize(8);
		mesh.m_arrayVertex[0].pos=vector3(100,0,0); mesh.m_arrayVertex[0].texCoord=vector2(0,0);
		mesh.m_arrayVertex[1].pos=vector3(100,100,0); mesh.m_arrayVertex[1].texCoord=vector2(1,0);
		mesh.m_arrayVertex[2].pos=vector3(0,0,0); mesh.m_arrayVertex[2].texCoord=vector2(1,1);
		mesh.m_arrayVertex[3].pos=vector3(0,100,0); mesh.m_arrayVertex[3].texCoord=vector2(0,1);

		// Back face
		mesh.m_arrayVertex[4].pos=vector3(0,0,-100); mesh.m_arrayVertex[4].texCoord=vector2(1,0);
		mesh.m_arrayVertex[5].pos=vector3(0,100,-100); mesh.m_arrayVertex[5].texCoord=vector2(1,1);
		mesh.m_arrayVertex[6].pos=vector3(100,0,-100); mesh.m_arrayVertex[6].texCoord=vector2(0,0);
		mesh.m_arrayVertex[7].pos=vector3(100,100,-100); mesh.m_arrayVertex[7].texCoord=vector2(0,1);

		const bool bConcave=true;
		if(bConcave)
		{
			mesh.m_arrayVertex[0].pos=vector3(200,0,0); 
			mesh.m_arrayVertex[1].pos=vector3(200,100,-50); 
			mesh.m_arrayVertex[2].pos=vector3(0,50,0); 
		}


		mesh.m_arrayFace.resize(12);

		// front face
		mesh.m_arrayFace[0].setIndex(0,1,2);
		mesh.m_arrayFace[1].setIndex(1,3,2);

		// left
		mesh.m_arrayFace[2].setIndex(2,3,4);
		mesh.m_arrayFace[3].setIndex(3,5,4);

		// top face
		mesh.m_arrayFace[4].setIndex(3,1,5);
		mesh.m_arrayFace[5].setIndex(5,1,7);

		// right face
		mesh.m_arrayFace[6].setIndex(0,6,1);
		mesh.m_arrayFace[7].setIndex(6,7,1);

		// back face
		mesh.m_arrayFace[8].setIndex(5,6,4);
		mesh.m_arrayFace[9].setIndex(5,7,6);

		// bottom face
		mesh.m_arrayFace[10].setIndex(0,2,4);
		mesh.m_arrayFace[11].setIndex(0,4,6);

		mesh.calculateVertexNormal();

		mesh.transform(matrix4(quater(TO_RADIAN(30),1,0,0), vector3(0,100,0)));

		mesh.calculateVertexNormal();

		// create a new scene node, and then attach the manualObject to the scene node, it is the same as attach a entity to a scene node
		Ogre::SceneNode* node1 = RE::ogreRootSceneNode()->createChildSceneNode( "ManualObject" );

		MeshLoader::MeshToEntity::Option option;
		option.useNormal=false;
		option.buildEdgeList=true;
		option.dynamicUpdate=true;
		MeshLoader::MeshToEntity mc(mesh, "ManualMesh", option);

		/*		if(runtime_modify)
		{
		mesh.m_arrayVertex[0].pos=vector3(200,0,0); 
		mesh.m_arrayVertex[1].pos=vector3(200,100,0); 
		mesh.m_arrayVertex[2].pos=vector3(0,50,0); 
		mc.updatePositions();
		}*/

		Ogre::Entity* entity=RE::ogreSceneManager()->createEntity("Manual", "ManualMesh");
		entity->setMaterialName("Examples/EnvMappedRustySteel");
		node1->attachObject(entity);
	}

}
#endif

Entity* MeshLoader::createMeshEntity(Mesh const& mesh, const char* entityName, const char* materialName,MeshToEntity::Option opt)
{
#ifndef NO_OGRE
  TString meshId=RE::generateUniqueName();
		
	MeshToEntity mc(mesh, meshId, opt);
	Ogre::Entity* pPlaneEnt = mc.createEntity(entityName, materialName);

	//Ogre::Entity* pPlaneEnt = RE::ogreSceneManager()->createEntity( entityName, meshId.ptr() );

	pPlaneEnt->setCastShadows(false);
	return pPlaneEnt;
#else
  return NULL;
#endif
}

