#ifndef MESHO_OLD_H
#define MESHO_OLD_H
#pragma once

#include "BaseLib/math/tvector.h"
#include "BaseLib/image/Image.h"
#include "BaseLib/motion/Motion.h"
#include "BaseLib/utility/TGL.h"
#include "BaseLib/utility/TUGL.h"
#include "MainLib/OgreFltk/Line3D.h"
class SkinnedMeshLoader;

namespace Ogre
{
	class Mesh;
	class Entity;
}
// all classes in this file are deprecated.
struct CPixelRGBA
{
	CPixelRGBA(){}
	CPixelRGBA(uchar RR, uchar GG, uchar BB, uchar AA):R(RR),G(GG),B(BB), A(AA){}
	uchar R;
	uchar G;
	uchar B;
	uchar A;

	void setColor(float RR, float GG, float BB, float AA=1.0)
	{
		R=ROUND(RR*255);
		G=ROUND(GG*255);
		B=ROUND(BB*255);
		A=ROUND(AA*255);
	}
};
namespace MeshLoader
{


	// no virtual function
	struct Vertex
	{
		vector3 pos;
		vector3 normal;
		vector2 texCoord;
		CPixelRGBA color;
	};

	struct Face
	{
		int vertexIndex[3];
		int vi(int i) const {return vertexIndex[i];}
		void setIndex(int i, int j, int k)	{ vertexIndex[0]=i;vertexIndex[1]=j;vertexIndex[2]=k;}
		void operator=(const Face& other)	{ for(int i=0; i<3; i++) vertexIndex[i]=other.vi(i);}
	};

	class Mesh
	{
		public:
		// Data
		std::vector<Vertex> m_arrayVertex;
		std::vector<Face>   m_arrayFace;
		bitvectorn isBoundaryVertex;	// valid only when using delaunayCGAL at the moment.
		TString m_meshName;
		TString m_materialName;

		Mesh(void){}
		Mesh(const Mesh& otherMesh){copyFrom(otherMesh);}
		virtual ~Mesh(void){}

		int numFace() const {return m_arrayFace.size();}
		Face const& getFace(int i) const	{return m_arrayFace[i];}
		Face & getFace(int i)				{return m_arrayFace[i];}
		vector3& getFaceCorner(int iFace, int icorner) ;
		vector3 calcFaceCenter(int i) const;
		vector3 calcFaceNormal(int i) const;

		vector3 calcMeshCenter() const;

		int numVertex() const{ return m_arrayVertex.size();}
		Vertex & getVertex(int i)			 {return m_arrayVertex[i];}
		Vertex const& getVertex(int i) const {return m_arrayVertex[i];}

		//////////////////////////////////////////////
		// tools
		//////////////////////////////////////////////

		// assumes that m_arrayFace and {m_arrayVertex[i].pos} are all valid.
		void calculateVertexNormal();

		void eliminateSharedVertex(Mesh const& otherMesh);

		void transform(matrix4 const& b);

		void copyFrom(Mesh const& otherMesh);
		void operator=(Mesh const& otherMesh) { copyFrom(otherMesh);}

		// export to a C++ header file. ex) saveMesh("Bunny", "BunnyMesh.h")
		void saveMesh(const char* identifier, const char* filename);
		void loadMesh(int numVertex, int numFace, double * vertices, int indexes[][3]);
		// export to a text file (.tri).
		bool saveMesh(const char* filename_);
		bool loadMesh(const char* filename_);

		// export to a widely used file format (.obj)
		bool saveObj(const char* filename, bool vn, bool vt);
		bool loadObj(const char* filename);
	};

	struct FaceEdge
	{
		FaceEdge():faceIndex(-1), vertexIndex(-1){}
		FaceEdge(int fi, int vi):faceIndex(fi), vertexIndex(vi){}

		// faceIndex번째 face의 vertexIndex와 (vertexIndex+1)%3을 연결하는 에지
		int faceIndex;
		int vertexIndex;	// in [0,1,2].

		// retrieve global vertex index
		int source(Mesh & mesh) const;
		int target(Mesh & mesh) const;
		int cross(Mesh& mesh) const;	// 맞은편 vertex

		Vertex& getSource(Mesh & mesh);
		Vertex& getTarget(Mesh & mesh);
		bool isNull() const;
		bool operator==(const FaceEdge& other);
		bool operator!=(const FaceEdge& other);
	};

	// build edge array and adjecent faces to the edges.
	class EdgeConnectivity
	{
	public:
		struct vertex
		{
			int vertexIndex;
		};

		struct edge
		{
			FaceEdge faceEdge[2];
			int numFaceEdge;
			edge() { numFaceEdge=0;}
		};

		typedef TUGL::edge<vertex, edge> edgeT;
		typedef TUGL::node<vertex, edge> nodeT;
		TUGL::graph<vertex, edge>  mMeshGraph;

		EdgeConnectivity(Mesh const& mesh);

		int numEdges() const;
		int numAdjEdges(int vertexIndex) const;
		bool isConnected(int vertex1, int vertex2);
		edge& getAdjEdge(int vertexIndex, int i) const;
		int getAdjVertex(int vertexIndex, int i) const;
		edge& getEdge(int i) const;
		int source(int iEdge) const;
		int target(int iEdge) const;
	};

	// build face adjacencies. O(V)+O(E)
	class MeshConnectivity
	{
		void _init(Mesh & mesh, EdgeConnectivity const & edges);
	public:
		MeshConnectivity(Mesh & mesh);
		MeshConnectivity(Mesh & mesh, EdgeConnectivity const & edges);

		FaceEdge vertexToFace(int vertex)			{ return vertexToFaceEdge[vertex];}
		inline FaceEdge adjFace(FaceEdge const& fe) const { return FaceEdge(adjFaceIndex(fe.faceIndex, fe.vertexIndex), adjFaceVertexIndex(fe.faceIndex, fe.vertexIndex));}
		// fe.source()에 인접한 다음 FaceEdge를 return한다. 다음FaceEdge의 source()는 fe.source()와 동일하다.
		FaceEdge next(FaceEdge const& fe) const;
		FaceEdge prev(FaceEdge const& fe) const;

		struct IncidentFaceIterator
		{
			IncidentFaceIterator(MeshConnectivity* pp, FaceEdge const& f){self=pp; fe=f; bFirst=true;}
			IncidentFaceIterator(MeshConnectivity* pp, FaceEdge const& f, bool bf){self=pp; fe=f; bFirst=bf;}
			MeshConnectivity* self;
			FaceEdge fe;
			bool bFirst;	// 맨처음에 cyclic vertice의 경우 begin()이 end()와 같으므로, 루프가 바로 종료되는것을 막기위한 변수.
			void operator++( )		{ fe=self->next(fe); bFirst=false; }
			void operator--( )		{ fe=self->prev(fe);}

			// 마지막 원소인지 알아볼때 쓸수있음. (if (i.next()==end())
			IncidentFaceIterator next() { return IncidentFaceIterator (self, self->next(fe), false);}
			IncidentFaceIterator prev() { return IncidentFaceIterator (self, self->prev(fe), false);}

			bool operator==(const IncidentFaceIterator& other) { if(bFirst) return false;  return fe==other.fe; }
			bool operator!=(const IncidentFaceIterator& other) { return !((*this)==other); }
			FaceEdge* operator->()	{ return &fe;}
			FaceEdge& operator*()	{ return fe;}
		};

		/////////////////////////////////////
		//
		// For(MeshConnectivity::IncidentFaceIterator i=mc.beginAdjFace(v); i!=mc.endAdjFace(v); ++i)
		//
		IncidentFaceIterator beginAdjFace(int vertexIndex)		{ return IncidentFaceIterator(this, vertexToFaceEdge[vertexIndex]);}
		IncidentFaceIterator endAdjFace(int vertexIndex)		{ if(mMesh.isBoundaryVertex(vertexIndex))	return IncidentFaceIterator(this, FaceEdge()); return beginAdjFace(vertexIndex);}


	private:
		std::vector<FaceEdge> vertexToFaceEdge;	// vertex에 달려있는 첫번째 faceEdge return
		intmatrixn adjFaceIndex;	// adjFaceIndex(faceIndex, vertexIndex)=faceIndex2
		intmatrixn adjFaceVertexIndex;

		void setAdj(FaceEdge const& fe1, FaceEdge const& fe2);

		Mesh & mMesh;

	};


#ifndef NO_OGRE
	// retrieve the results of vertex weighted skinning . mesh.firstInit() is not called inside the function!
	class GetMeshAnimation
	{
		SkinnedMeshLoader* mSkin;
		void* m_data;
		m_real mScaleFactor;
		Motion * m_mot;
	public:
		GetMeshAnimation(m_real scaleFactor=1.0):mSkin(NULL),m_data(NULL), mScaleFactor(scaleFactor), m_mot(NULL){}
		~GetMeshAnimation();
		void setScaleFactor(m_real scaleF)	{ mScaleFactor=scaleF;}
		m_real getScaleFactor() const		{ return mScaleFactor;}
		void create(Motion const& mot, const char* meshFile, Mesh& mesh, int iframe);
		void update(Mesh& mesh, int iframe);
		void update(Mesh& mesh, Posture const& pose);
	};
#endif


	// Mesh 그리기.

	// Mesh자료구조를 Ogre::Entity자료구조로 변환한다. 메시가 바뀌면, updatePositions()를 호출하면 Entity도 변경된다.
	class MeshToEntity
	{
	public:
		struct Option
		{
			Option() { useColor=false, useNormal=true, useTexCoord=true; buildEdgeList=false; dynamicUpdate=false;}
			bool useColor;
			bool useNormal;
			bool useTexCoord;
			bool buildEdgeList;
			bool dynamicUpdate;
		};
#ifndef NO_OGRE
		Ogre::Mesh* mMesh;
#endif
		MeshToEntity(const Mesh& mesh, const char* ogreMeshName, Option option=Option ());
		void updatePositions();
		void updatePositionsAndNormals();

		// created entities will not be removed automatically when destructing a MeshToEntity object.
		Ogre::Entity* createEntity(const char* entityName, const char* materialName="white");
		Ogre::Entity* getLastCreatedEntity() const	{ return mEntity;}

	private:
		const MeshLoader::Mesh& mInputMesh;
		Ogre::Entity* mEntity;
		Option mSavedOption;
	};

	Ogre::Entity* createMeshEntity(Mesh const& mesh, const char* entityName, const char* materialName="white", MeshToEntity::Option option=MeshToEntity::Option());

	// deprecated. use createMeshEntity or MeshToEntity when you need dynamic update.
#ifndef NO_OGRE
	class MeshEntity : public Mesh, public SimplerRenderable
	{
	public:
		MeshEntity ():Mesh(), SimplerRenderable(){}
		virtual void firstInit()=0;		// initialize structure.
		virtual void update()=0;		// when only vertex positions are changed, you can call update();
	};


	// for rendering of Mesh. Each vertex of a face should have valid positions, normals, and texture coordinates.
	class TriangleMesh : public MeshEntity
	{
	public:
		TriangleMesh ();
		~TriangleMesh ();

#if OGRE_VERSION_MINOR>=12
		void setMaterial(const char*);
#endif
		void firstInit();	// construct mesh graph
		void update();		// when only vertex position is changed you can call update();
	};

	// for drawing of Mesh using lines.
	class MeshLineStrip: public MeshEntity
	{
	public:
		MeshLineStrip();
		~MeshLineStrip();
#if OGRE_VERSION_MINOR>=12
		void setMaterial(const char*);
#endif

		EdgeConnectivity* edges;
		void _createGraph();
		void firstInit();	// construct mesh graph
		void update();		// when vertex position is changed you can call update();
	};

	// for drawing of Mesh using lines.
	class MeshLineStripReduced: public MeshEntity
	{
	public:
		MeshLineStripReduced():MeshEntity(){}
		void firstInit(){ update();}
		void update();		// when vertex position is changed you can call update();
	};

#endif
	// file: 16bit per pixel raw file. -> creates Ogre::simpleRendererble
	void createTerrain(Mesh& mesh, const char* filename, int imageSizeX, int imageSizeY, m_real sizeX, m_real sizeZ, m_real heightMax, int ntexSegX, int ntexSegZ);


	void createPlane(Mesh& mesh, int numSegX, int numSegZ, m_real sizeX, m_real sizeZ);
	void createPlane(Mesh& mesh, const char* imageFile, int numSeg, m_real size);
}
#endif
