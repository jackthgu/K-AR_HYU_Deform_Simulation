#include "stdafx.h"
#include <Ogre.h>
#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"

#include "BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpa2.h"
#include "LinearMath/btQuickprof.h"
//#include "BMF_Api.h"
#include "BunnyMesh.h"
#include "TorusMesh.h"
#include <stdio.h> //printf debugging
#include "LinearMath/btConvexHull.h"
#include "BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"

#include "Ogre_ShapeDrawer.h"
#include "../../MainLib/OgreFltk/renderer.h"
#include "../../MainLib/OgreFltk/pldprimskin.h"
#include "../../MainLib/OgreFltk/FltkRenderer.h"
#include "../../MainLib/OgreFltk/RE.h"



#include "bulletTools.h"
Ogre_DebugDrawer::Ogre_DebugDrawer()
:m_debugMode(0)
{

	mLine=NULL;

	RE::renderer().mRoot->addFrameListener(this);
}

Ogre_DebugDrawer::~Ogre_DebugDrawer()
{
	RE::renderer().mRoot->removeFrameListener(this);
}


void	Ogre_DebugDrawer::setDebugMode(int debugMode)
{
	m_debugMode = debugMode;

}


/*
void	GLDebugDrawer::drawContactPoint(const btVector3& pointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color)
{
	if (m_debugMode & btIDebugDraw::DBG_DrawContactPoints)
	{
		btVector3 to=pointOnB+normalOnB*distance;
		const btVector3&from = pointOnB;
		glBegin(GL_LINES);
		glColor3f(color.getX(), color.getY(), color.getZ());
		glVertex3d(from.getX(), from.getY(), from.getZ());
		glVertex3d(to.getX(), to.getY(), to.getZ());
		glEnd();


		glRasterPos3f(from.x(),  from.y(),  from.z());
		char buf[12];
		sprintf(buf," %d",lifeTime);
		BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);


	}
}
*/

void Ogre_DebugDrawer::flush()
{
	if(!mLine)
	{
		mLine=new LineList();

		if(!mNode)
		{
			mNode.reset(RE::createSceneNode(RE::generateUniqueName()), (void(*)(Ogre::SceneNode* node))RE::removeEntity);			
		}

		mNode->attachObject(mLine);

	}

	mLine->begin(linesS.size());
	for(int i=0; i<linesS.size(); i++)
	{		
		mLine->line(i, linesS[i], linesE[i]);
	}
	mLine->end();
}







/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
/*
#ifdef WIN32 //needed for glut.h
#include <windows.h>
#endif

//think different
#if defined(__APPLE__) && !defined (VMDMESA)
#include <OpenGL/gl.h>
#include <OpenGL/glu.h>
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include "GlutStuff.h"*/

#include "Ogre_ShapeDrawer.h"
#include "BulletCollision/CollisionShapes/btPolyhedralConvexShape.h"
#include "BulletCollision/CollisionShapes/btTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btBoxShape.h"
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include "BulletCollision/CollisionShapes/btConeShape.h"
#include "BulletCollision/CollisionShapes/btCylinderShape.h"
#include "BulletCollision/CollisionShapes/btTetrahedronShape.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "BulletCollision/CollisionShapes/btCapsuleShape.h"
#include "BulletCollision/CollisionShapes/btConvexTriangleMeshShape.h"
#include "BulletCollision/CollisionShapes/btUniformScalingShape.h"
#include "BulletCollision/CollisionShapes/btStaticPlaneShape.h"
///
#include "BulletCollision/CollisionShapes/btShapeHull.h"

#include "LinearMath/btTransformUtil.h"


#include "LinearMath/btIDebugDraw.h"
//for debugmodes
//#include "BMF_Api.h"
#include <stdio.h> //printf debugging



class GlDrawcallback : public btTriangleCallback
{

	Ogre::ManualObject* manual;
public:

	
	GlDrawcallback(Ogre::ManualObject* mm)
		:manual(mm)
	{
		manual->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);
	}

	virtual void processTriangle(btVector3* triangle,int partId, int triangleIndex)
	{
		manual->position(Ogre::Vector3(triangle[0].getX(), triangle[0].getY(), triangle[0].getZ()));
		manual->colour(1, 0, 0);

		manual->position(Ogre::Vector3(triangle[1].getX(), triangle[1].getY(), triangle[1].getZ()));
		manual->colour(0, 1, 0);

		manual->position(Ogre::Vector3(triangle[2].getX(), triangle[2].getY(), triangle[2].getZ()));
		manual->colour(0, 0, 1);
	}

};


class btOgreRigidBody : public btRigidBody
{
public:
	btOgreRigidBody (	const btRigidBody::btRigidBodyConstructionInfo& constructionInfo, Ogre::SceneNode* parent):
	  btRigidBody(constructionInfo)
	  {
		mNode=parent->createChildSceneNode();
#ifdef _DEBUG
		static int id=0;
		mId=id++;
#endif
	  }

	  ~btOgreRigidBody(){ RE::removeEntity(mNode);}
  	Ogre::SceneNode* mNode;
#ifdef _DEBUG
	int mId;
#endif

};


btRigidBody* createOgreRigidBody(const btRigidBody::btRigidBodyConstructionInfo& constructionInfo, Ogre::SceneNode* parent)
{

	btOgreRigidBody* prb=new btOgreRigidBody(constructionInfo, parent);


	btCollisionShape* shape=prb->getCollisionShape();

	if (shape->isConvex())
	{
		btConvexShape* convexShape = (btConvexShape*)shape;
		
		//create a hull approximation
		btShapeHull* hull = new btShapeHull(convexShape);
	
		
		btScalar margin = shape->getMargin();
		hull->buildHull(margin);
			
		if (hull->numTriangles () > 0)
		{
			int index = 0;
			const unsigned int* idx = hull->getIndexPointer();
			const btVector3* vtx = hull->getVertexPointer();
			
			Ogre::ManualObject* manual = RE::ogreSceneManager()->createManualObject(RE::generateUniqueName().ptr());

			//glBegin (GL_TRIANGLES);

			manual->begin("white", Ogre::RenderOperation::OT_TRIANGLE_LIST);

			for (int i = 0; i < hull->numTriangles (); i++)
			{
				int i1 = index++;
				int i2 = index++;
				int i3 = index++;
				btAssert(i1 < hull->numIndices () &&
					i2 < hull->numIndices () &&
					i3 < hull->numIndices ());

				int index1 = idx[i1];
				int index2 = idx[i2];
				int index3 = idx[i3];
				btAssert(index1 < hull->numVertices () &&
					index2 < hull->numVertices () &&
					index3 < hull->numVertices ());

				btVector3 v1 = vtx[index1];
				btVector3 v2 = vtx[index2];
				btVector3 v3 = vtx[index3];
				btVector3 normal = (v3-v1).cross(v2-v1);
				normal.normalize ();
				
			//	glNormal3f(normal.getX(),normal.getY(),normal.getZ());
				manual->position(v1.x(), v1.y(), v1.z());
				manual->normal(normal.getX(),normal.getY(),normal.getZ());
				
				manual->position(v2.x(), v2.y(), v2.z());
				manual->normal(normal.getX(),normal.getY(),normal.getZ());

				manual->position(v3.x(), v3.y(), v3.z());
				manual->normal(normal.getX(),normal.getY(),normal.getZ());
				
			}

/*								for (int i = 0; i < hull->numTriangles (); i++)
			{
				manual->triangle(i*3, i*3+1, i*3+2);
			}
*/
			manual->end();

			
			prb->mNode->attachObject(manual);

			delete hull;

		} 
	
	}
	/// for polyhedral shapes
	if (shape->isPolyhedral())
	{

		// 일반적인 경우 모두 동작. 구현 엉성함.
		btPolyhedralConvexShape* polyshape = (btPolyhedralConvexShape*) shape;
		
		{
			//glRasterPos3f(0.0,  0.0,  0.0);
			//BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),polyshape->getExtraDebugInfo());

			//glColor3f(1.f, 1.f, 1.f);
			int i;
			for (i=0;i<polyshape->getNumVertices();i++)
			{
				btPoint3 vtx;
				polyshape->getVertex(i,vtx);
				//glRasterPos3f(vtx.x(),  vtx.y(),  vtx.z());
				char buf[12];
				sprintf(buf," %d",i);
	//				BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
			}

			for (i=0;i<polyshape->getNumPlanes();i++)
			{
				btVector3 normal;
				btPoint3 vtx;
				polyshape->getPlane(normal,vtx,i);
				btScalar d = vtx.dot(normal);

				//glRasterPos3f(normal.x()*d,  normal.y()*d, normal.z()*d);
				char buf[12];
				sprintf(buf," plane %d",i);
	//				BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),buf);
				
			}
		}

			
	}

	if (shape->isConcave())//>getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE||shape->getShapeType() == GIMPACT_SHAPE_PROXYTYPE)
	//		if (shape->getShapeType() == TRIANGLE_MESH_SHAPE_PROXYTYPE)
	{
#define TERRAIN_ONLY
#ifdef TERRAIN_ONLY
		// rendering을 깔끔하게 하고 싶은 경우 사용.
		btTerrainTriangleMeshShape*		csh=static_cast<btTerrainTriangleMeshShape*>(shape);

				
		Ogre::Entity* entity=MeshLoader::createMeshEntity(*csh->mTerrain, RE::generateUniqueName());


		entity->setMaterialName("CrowdEdit/Terrain1");
//		entity->setMaterialName("checkboard/crowdEditing");
//		entity->setMaterialName("checkboard/crowdEditing2");
		entity->setCastShadows(false);
//		entity->setNormaliseNormals(true);
		prb->mNode->attachObject(entity);

#else
		btTriangleMeshShape* concaveMesh = (btTriangleMeshShape*) shape;
		//btVector3 aabbMax(btScalar(1e30),btScalar(1e30),btScalar(1e30));
		//btVector3 aabbMax(100,100,100);//btScalar(1e30),btScalar(1e30),btScalar(1e30));

		//todo pass camera, for some culling
		btVector3 aabbMax(btScalar(1e30),btScalar(1e30),btScalar(1e30));
		btVector3 aabbMin(-btScalar(1e30),-btScalar(1e30),-btScalar(1e30));

		Ogre::ManualObject* manual = RE::ogreSceneManager()->createManualObject(RE::generateUniqueName().ptr());

		GlDrawcallback drawCallback(manual);

		concaveMesh->processAllTriangles(&drawCallback,aabbMin,aabbMax);

		manual->end();

		prb->mNode->attachObject(manual);
#endif
	}

	//glDisable(GL_DEPTH_BUFFER_BIT);
	//glRasterPos3f(0,0,0);//mvtx.x(),  vtx.y(),  vtx.z());
	//if (debugMode&btIDebugDraw::DBG_DrawText)
	{
	//			BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),shape->getName());
	}

	//if (debugMode& btIDebugDraw::DBG_DrawFeaturesText)
	{
		//BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),shape->getExtraDebugInfo());
	}

	return prb;
}





void Ogre_ShapeDrawer::pushMatrix(btScalar* m)
{
	matrix4 t;
	t.setValue(	m[0], m[4], m[8], m[12], 
				m[1], m[5], m[9], m[13], 
				m[2], m[6], m[10], m[14], 
				m[3], m[7], m[11], m[15]);

	if(mMatrixStack.size()==0)
		mMatrixStack.push_back(t);
	else
		mMatrixStack.push_back(mMatrixStack.back()*t);


}
void Ogre_ShapeDrawer::popMatrix()
{
	mMatrixStack.pop_back();
}

void Ogre_ShapeDrawer::drawOpenGL(btScalar* m, const btRigidBody* body, const btCollisionShape* shape, const btVector3& color,int	debugMode)
{
	pushMatrix(m);

	if (shape->getShapeType() == UNIFORM_SCALING_SHAPE_PROXYTYPE)
	{
		const btUniformScalingShape* scalingShape = static_cast<const btUniformScalingShape*>(shape);
		const btConvexShape* convexShape = scalingShape->getChildShape();
		float	scalingFactor = (float)scalingShape->getUniformScalingFactor();
		{
			btScalar tmpScaling[4][4]={{scalingFactor,0,0,0},
				{0,scalingFactor,0,0},
				{0,0,scalingFactor,0},
				{0,0,0,1}};
			
			drawOpenGL( (btScalar*)tmpScaling,body, convexShape,color,debugMode);
		}
	
		popMatrix();
		return;
	}

	if (shape->getShapeType() == COMPOUND_SHAPE_PROXYTYPE)
	{
		const btCompoundShape* compoundShape = static_cast<const btCompoundShape*>(shape);
		for (int i=compoundShape->getNumChildShapes()-1;i>=0;i--)
		{
			btTransform childTrans = compoundShape->getChildTransform(i);
			const btCollisionShape* colShape = compoundShape->getChildShape(i);
			btScalar childMat[16];
			childTrans.getOpenGLMatrix(childMat);
			drawOpenGL(childMat,body, colShape,color,debugMode);
		}

	} else
	{
		//drawCoordSystem();
	    
		//glPushMatrix();
		//glEnable(GL_COLOR_MATERIAL);
		//glColor3f(color.x(),color.y(), color.z());

		

		bool useWireframeFallback = true;

		const btOgreRigidBody* ogreshape=dynamic_cast<const btOgreRigidBody*>(body);

		if(ogreshape)
		{
			//BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),shape->getExtraDebugInfo());
			quater q;
			vector3 pos;
			q.setRotation(mMatrixStack.back());
			pos.translation(mMatrixStack.back());
			ogreshape->mNode->resetToInitialState();
			ogreshape->mNode->setOrientation(ToOgre(q));
			ogreshape->mNode->setPosition(ToOgre(pos));
		}
		//		glEnable(GL_DEPTH_BUFFER_BIT);

	//	glPopMatrix();
	}
	popMatrix();
  //  glPopMatrix();
	
}


Ogre_ShapeDrawer::Ogre_ShapeDrawer()
{
	//RE::renderer().mRoot->addFrameListener(this);

}

Ogre_ShapeDrawer::~Ogre_ShapeDrawer()
{
	/*int i;
	for (i=0;i<m_shapeHulls.size();i++)
	{
		btShapeHull* hull = m_shapeHulls[i];
		hull->~btShapeHull();
		btAlignedFree(hull);
		m_shapeHulls[i] = 0;
	}
	m_shapeHulls.clear();

	RE::renderer().mRoot->removeFrameListener(this);*/
}

void updateRigidBodyOgreNode(const btRigidBody* body)
{
	const btOgreRigidBody* ogreshape=dynamic_cast<const btOgreRigidBody*>(body);

	if(ogreshape)
	{
		const btTransform&	wtr=ogreshape->getInterpolationWorldTransform();
		//BMF_DrawString(BMF_GetFont(BMF_kHelvetica10),shape->getExtraDebugInfo());
		quater q;
		vector3 pos;
		/*
		q=ToBase(body->
		q.setRotation(mMatrixStack.back());
		pos.translation(mMatrixStack.back());
		ogreshape->mNode->resetToInitialState();
		ogreshape->mNode->setOrientation(ToOgre(q));
		ogreshape->mNode->setPosition(ToOgre(pos));
		*/
		pos=ToBase(wtr(btVector3(0,0,0)));
		ogreshape->mNode->resetToInitialState();
		ogreshape->mNode->setPosition(ToOgre(pos));
	}
}
