#ifndef Ogre_ShapeDrawer_H
#define Ogre_ShapeDrawer_H
#pragma once

#include "MainLib/OgreFltk/Line3D.h"
#include <boost/smart_ptr.hpp>
#include "LinearMath/btIDebugDraw.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btVector3.h"
#include <OgreFrameListener.h>
class btCollisionShape;
class Ogre_DebugDrawer : public btIDebugDraw, public Ogre::FrameListener
{
	boost::shared_ptr<Ogre::SceneNode> mNode;
	int m_debugMode;

	vector3N linesS;
	vector3N linesE;

	LineList* mLine;
public:
	Ogre_DebugDrawer ();
	~Ogre_DebugDrawer();

	virtual void	drawLine(const btVector3& from,const btVector3& to,const btVector3& color)
	{
		linesS.pushBack(vector3(from.x(), from.y(), from.z()));
		linesE.pushBack(vector3(to.x(), to.y(), to.z()));
	}

	virtual void	drawContactPoint(const btVector3& PointOnB,const btVector3& normalOnB,btScalar distance,int lifeTime,const btVector3& color){}

	virtual void	reportErrorWarning(const char* warningString){Msg::print2(warningString);}

	virtual void	draw3dText(const btVector3& location,const char* textString){}

	virtual void	setDebugMode(int debugMode);

	virtual int		getDebugMode() const { return m_debugMode;}

	void flush();

	// Ogre::FrameListener
	virtual bool frameStarted(const Ogre::FrameEvent& evt){ return true;}
	virtual bool frameEnded(const Ogre::FrameEvent& evt)
	{
		linesS.setSize(0);
		linesE.setSize(0);
		return true;
	}

};


#include "btBulletDynamicsCommon.h"
/// OpenGL shape drawing
class Ogre_ShapeDrawer
{
	std::vector<matrix4> mMatrixStack;
public:
	Ogre_ShapeDrawer();
	virtual ~Ogre_ShapeDrawer();

	void pushMatrix(btScalar* m);
	void popMatrix();
		
	void		drawOpenGL(btScalar* m, const btRigidBody* body, const btCollisionShape* shape, const btVector3& color,int	debugMode);
	
};

void updateRigidBodyOgreNode(const btRigidBody* pBody);
#endif
