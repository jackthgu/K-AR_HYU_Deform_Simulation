#include "stdafx.h"
#include "softbody2/Physics.h"
#include "softbody2/Physics_ParticleSystem.h"

//#define MACRO_BULLET_TO_STRING(b) #b
//
//
//#ifdef _DEBUG
//#define MACRO_BULLET_LIB(a) MACRO_BULLET_TO_STRING(../dependencies/bullet-2.68/out/debug_dbl8/build/##a/##a##.lib)
//#else
//#define MACRO_BULLET_LIB(a) MACRO_BULLET_TO_STRING(../dependencies/bullet-2.68/out/release_dbl8/build/##a/##a.lib)
//#endif
//
//
//
//#pragma comment(lib, MACRO_BULLET_LIB(libbulletdynamics))
//#pragma comment(lib, MACRO_BULLET_LIB(libbulletcollision))
////#pragma comment(lib, MACRO_BULLET_LIB(libbulletsoftbody))
//#pragma comment(lib, MACRO_BULLET_LIB(libbulletmath))
//

#include "btBulletDynamicsCommon.h"
#include "Ogre_ShapeDrawer.h"
#include "bulletTools.h"
btRigidBody* createOgreRigidBody(const btRigidBody::btRigidBodyConstructionInfo& constructionInfo, Ogre::SceneNode* parent);
void updateRigidBodyOgreNode(const btRigidBody* pBody);
btRigidBody*	btLocalCreateRigidBody(float mass, const btTransform& startTransform,btCollisionShape* shape, Ogre::SceneNode* parentNode)
{
	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0,0,0);
	if (isDynamic)
		shape->calculateLocalInertia(mass,localInertia);

	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
	btRigidBody::btRigidBodyConstructionInfo cInfo(mass,myMotionState,shape,localInertia);
	btRigidBody* body = createOgreRigidBody(cInfo, parentNode);

	updateRigidBodyOgreNode(body);
	return body;
}

btTriangleMesh* btCreateTriangleMesh(MeshLoader::Mesh& mesh)
{
	btTriangleMesh* meshInterface= new btTriangleMesh();

	for ( int i=0, ni=mesh.numFace();i<ni;i++)
	{
		btVector3 vertex0=ToBullet(mesh.m_arrayVertex[mesh.m_arrayFace[i].vi(0)].pos);
		btVector3 vertex1=ToBullet(mesh.m_arrayVertex[mesh.m_arrayFace[i].vi(1)].pos);
		btVector3 vertex2=ToBullet(mesh.m_arrayVertex[mesh.m_arrayFace[i].vi(2)].pos);


		meshInterface->addTriangle(vertex0,vertex1,vertex2);
	}
	
	int ni=meshInterface->getNumTriangles();
	return meshInterface;
}

btBvhTriangleMeshShape* createConcaveCollisionShape(MeshLoader::Mesh& mesh)
{	
	btBvhTriangleMeshShape* trimeshShape = new btBvhTriangleMeshShape(btCreateTriangleMesh(mesh),true);
	return trimeshShape;
}

//#define TERRAIN_TEST
#ifdef TERRAIN_TEST
#include "../MainLib/OgreFltk/Line3D.h"
#endif
btBvhTriangleMeshShape* createTerrainCollisionShape(const char* filename, int sizeX, int sizeY, m_real width, m_real height, m_real heightMax)
{
	MeshLoader::Terrain* terrain=new MeshLoader::Terrain(filename, sizeX, sizeY, width, height, heightMax, 1,1);

#ifdef TERRAIN_TEST
	const double SPHERE_HALF_RADIUS=0.3;
	QuadList* pDrawCon=new QuadList(vector3(0,1,0), SPHERE_HALF_RADIUS*2);
	pDrawCon->setCastShadows(false);
	pDrawCon->setMaterial("greenCircle");

	sizeX*=2;
	sizeX-=1;
	sizeY*=2;
	sizeY-=1;
	pDrawCon->begin(sizeX*sizeY);
	for(int i=0; i<sizeY; i++)
	{
		for(int j=0; j<sizeX; j++)
		{
			double x=double(j)/(sizeX-1.0)*width;
			double z=double(i)/(sizeY-1.0)*height;
			//pDrawCon->quad(i*sizeY+j, vector3(x,terrain->maxHeight(vector2(x, z)),z));
			vector3 normal;
			pDrawCon->quad(i*sizeY+j, vector3(x,terrain->height(vector2(x, z), normal),z));
		}
	}
	pDrawCon->end();
	
	Ogre::SceneNode* sc=RE::ogreRootSceneNode()->createChildSceneNode();
	sc->attachObject(pDrawCon);
	sc->setPosition(-200,-20, -200);
#endif
	return (btBvhTriangleMeshShape* )new btTerrainTriangleMeshShape(terrain, btCreateTriangleMesh(*terrain), true);
}

#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld2.h"
#include "BulletSoftBody/btSoftBody2Helpers.h"



bool			CheckContactTaesoo(	btSoftBody* psb,
									btRigidBody* prb,
									btVector3 const& x,
									btScalar margin,
									btSoftBody::sCti& cti)
{	
	btVector3			nrm;
	btCollisionShape*	shp=prb->getCollisionShape();
	if(shp->isConvex())
	{
		btConvexShape*		csh=static_cast<btConvexShape*>(shp);
		const btTransform&	wtr=prb->getInterpolationWorldTransform();

		btScalar			dst=psb->m_worldInfo->m_sparsesdf.Evaluate(	wtr.invXform(x),
																		csh,
																		nrm,
																		margin);
		if(dst<0)
		{
			cti.m_body		=	prb;
			cti.m_normal	=	wtr.getBasis()*nrm;


			cti.m_offset	=	-dot(	cti.m_normal,
										x-cti.m_normal*dst);
			return(true);
		}
		return(false);
	}
	else 
	{
		btTerrainTriangleMeshShape*		csh=static_cast<btTerrainTriangleMeshShape*>(shp);
		const btTransform&	wtr=prb->getInterpolationWorldTransform();
/*		btScalar			dst=psb->m_worldInfo->m_sparsesdf.Evaluate(	wtr.invXform(x),
																		csh,
																		nrm,
																		margin);
*/
		btVector3 localX=wtr.invXform(x);

		vector2 proj_localX(localX.x(), localX.z());
		m_real maxHeight=csh->mTerrain->maxHeight(proj_localX);
		
		if(localX.y()>maxHeight+margin)
			return false;

		vector3 normal;
		btScalar dst=localX.y()-(csh->mTerrain->height(proj_localX, normal)+margin);

		nrm=ToBullet(normal);
		
		if(dst<0)
		{
			cti.m_body		=	prb;
			cti.m_normal	=	wtr.getBasis()*nrm;
			cti.m_offset	=	-dot(	cti.m_normal,
										x-cti.m_normal*dst);
			return(true);
		}
	}
	return(false);
}

bool			CheckContactTaesoo(	btSoftBody2* psb,
									btRigidBody* prb,
									btSoftBody2::Node& n,
									btScalar margin,
									btSoftBody2::sCti& cti)
{
	btVector3 x=n.x();
	btScalar marginOffset=0;
	if(psb->m_pSystem->mContacts.contactStatus[n.m_index]!=Physics_Contacts::no_contact)
		marginOffset=margin;
	btVector3			nrm;
	btCollisionShape*	shp=prb->getCollisionShape();
	if(shp->isConvex())
	{
		btConvexShape*		csh=static_cast<btConvexShape*>(shp);
		const btTransform&	wtr=prb->getInterpolationWorldTransform();

		btScalar			dst=psb->m_worldInfo->m_sparsesdf.Evaluate(	wtr.invXform(x),
																		csh,
																		nrm,
																		margin);
		if(dst-marginOffset<0)
		{
			cti.m_body		=	prb;
			cti.m_normal	=	wtr.getBasis()*nrm;
			cti.m_dp=dst;
			cti.m_dp2=dst-marginOffset;
			return(true);
		}
		return(false);
	}
	else 
	{
		btTerrainTriangleMeshShape*		csh=static_cast<btTerrainTriangleMeshShape*>(shp);
		const btTransform&	wtr=prb->getInterpolationWorldTransform();
/*		btScalar			dst=psb->m_worldInfo->m_sparsesdf.Evaluate(	wtr.invXform(x),
																		csh,
																		nrm,
																		margin);
*/
		btVector3 localX=wtr.invXform(x);

		vector2 proj_localX(localX.x(), localX.z());
		m_real maxHeight=csh->mTerrain->maxHeight(proj_localX);
		
		if(localX.y()>maxHeight+margin)
			return false;

		vector3 normal;
		btScalar dst=localX.y()-(csh->mTerrain->height(proj_localX, normal)+margin);

		nrm=ToBullet(normal);
		
		if(dst-marginOffset<0)
		{
			cti.m_body		=	prb;
			cti.m_normal	=	wtr.getBasis()*nrm;
			cti.m_dp=dst;
			cti.m_dp2=dst-marginOffset;
			return(true);
		}
	}
	return(false);
}

m_real getTerrainHeight(btRigidBody* terrain, btVector3 const& gpos)
{
	btCollisionShape* shp=terrain->getCollisionShape();
					
	// terrain
	btTerrainTriangleMeshShape*		csh=static_cast<btTerrainTriangleMeshShape*>(shp);
	const btTransform&	wtr=terrain->getInterpolationWorldTransform();
	btVector3 localX=wtr.invXform(gpos);
	vector3 normal;
	localX.setY(csh->mTerrain->height(vector2(localX.x(), localX.z()), normal));
	return wtr(localX).y();
}
void getTerrainHeight(btRigidBody* terrain, vector3 const& in, vector3 & out, vector3& normal)
{
	btCollisionShape* shp=terrain->getCollisionShape();
					
	// terrain
	btTerrainTriangleMeshShape*		csh=static_cast<btTerrainTriangleMeshShape*>(shp);
	const btTransform&	wtr=terrain->getInterpolationWorldTransform();
	btVector3 localX=wtr.invXform(ToBullet(in));
	double height=csh->mTerrain->height(vector2(localX.x(), localX.z()), normal);
	localX.setY(height);
	out=ToBase(wtr(localX));
	//printf("%s %s\n", ToBase(localX).output().ptr(), out.output().ptr());
}

