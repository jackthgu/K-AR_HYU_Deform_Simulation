#ifndef TETRAMESH_H
#define TETRAMESH_H
#pragma once

#include "BaseLib/math/tvector.h"
#include <OgreMesh.h>
#include <PhysicsLib/TRL/eigenSupport.h>
typedef _tvector<int,4> _quadi;
typedef _tvector<double,4> _quadd;

namespace TetraMeshLoader
{
	class TetraMesh
	{
	public:

		struct Node
		{
			vector3 pos;
/*			int index;
			std::vector<Node*> pNeighbors;
			void setNeighbors(std::vector<Node*> const& pNodes_) 
			{
				pNeighbors = pNodes_;
			}*/
		};
		
		std::vector<Node> _vmesh_vertex;
		std::vector<_quadi> _vmesh_index;	// node index for volume mesh

		TetraMesh() {}
		~TetraMesh() {}



		void transform(matrix4 const& b);

	public:
		int getNumNode() const { return _vmesh_vertex.size(); }
		vector3 const& nodePos(int i) const	{ return _vmesh_vertex[i].pos;}
		vector3 & nodePos(int i)			{ return _vmesh_vertex[i].pos;}

		int getNumTetra() const { return _vmesh_index.size(); }
		_quadi& getTetra(int i) {return _vmesh_index[i];}
		_quadi const& getTetra(int i)const {return _vmesh_index[i];}
		vector3& getTetraCorner(int iTetra, int icorner) ;
		_tvector<int,3> getTetraFace(int iTetra, int iface) const;

		vector3 calcTetraCenter(int i) const;
		
		// loads volume mesh data written in {mesh file reference frame}
		bool loadMesh(const char *file_name_);	
		bool saveMesh(const char *file_name_);

		void normalizeIndex(int start=0, int end=INT_MAX);
		void compareMesh(TetraMesh dest);
	};

	// TetraMesh 그리기.

	// Mesh자료구조를 Ogre::Entity자료구조로 변환한다. 메시가 바뀌면, updatePositions()를 호출하면 Entity도 변경된다.
	class TetraMeshToEntity
	{		
	public:
		struct Option
		{
			Option() { useRandomColor=true, useNormal=true, buildEdgeList=false; dynamicUpdate=false;}
			bool useRandomColor;
			bool useNormal;
			bool buildEdgeList;
			bool dynamicUpdate;
		};
		
		Ogre::Mesh* mMesh;
		TetraMeshToEntity(const TetraMesh& mesh, const char* ogreMeshName, Option option=Option ());
		void updatePositions();
		void updatePositionsAndNormals();

		Ogre::Entity* createEntity(const char* entityName, const char* materialName="white");
		Ogre::Entity* getLastCreatedEntity() const	{ return mEntity;}

	private:
		const TetraMesh& mInputMesh;
		Ogre::Entity* mEntity;
		Option mSavedOption;
	};

	Ogre::Entity* createEntityFromTetraMesh(TetraMesh const& mesh, const char* entityName, const char* materialName="white", TetraMeshToEntity::Option option=TetraMeshToEntity::Option());
}

#endif
