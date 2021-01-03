#pragma once

#include "Mesh_old.h"

btRigidBody*	btLocalCreateRigidBody(float mass, const btTransform& startTransform,btCollisionShape* shape, Ogre::SceneNode* parentNode);
btBvhTriangleMeshShape* createConcaveCollisionShape(MeshLoader::Mesh& mesh);

inline btVector3 ToBullet(vector3 const& a)
{
	return btVector3(a.x, a.y, a.z);
}

inline btQuaternion ToBullet(quater const& a)
{
	return btQuaternion (a.x, a.y, a.z, a.w);
}


inline vector3 ToBase(btVector3 const& a)
{
	return vector3(a.x(), a.y(), a.z());
}


#include "Terrain.h"

class btTerrainTriangleMeshShape: public btBvhTriangleMeshShape
{
public:
	MeshLoader::Terrain* mTerrain;
	btTerrainTriangleMeshShape(MeshLoader::Terrain* terr, btStridingMeshInterface* meshInterface, 
		bool useQuantizedAabbCompression, bool buildBvh = true)
		:mTerrain(terr),btBvhTriangleMeshShape(meshInterface, useQuantizedAabbCompression, buildBvh)
	{
	}

	~btTerrainTriangleMeshShape(){delete mTerrain;}

};


btBvhTriangleMeshShape* createTerrainCollisionShape(const char* filename, int sizeX, int sizeY, m_real width, m_real height, m_real heightMax);

m_real getTerrainHeight(btRigidBody* terrain, btVector3 const& gpos);
void getTerrainHeight(btRigidBody* terrain, vector3 const& in, vector3 & out, vector3& normal);
