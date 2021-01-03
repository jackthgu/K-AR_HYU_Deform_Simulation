
#ifndef OpenGLNativeRenderSystem_H_
#define OpenGLNativeRenderSystem_H_
#include "GL/glew.h"
#include <GL/gl.h>
#include <OgreRenderQueueListener.h>
#include <OgreMovableObject.h>
#include <OgreSceneManager.h>
#include <OgreMaterialManager.h>
#include <OgreManualObject.h>
#include "INativeRender.h"
#include "BaseLib/utility/QPerformanceTimer.h"

#if OGRE_VERSION_MINOR>=12
#include <OgreTechnique.h>
#include <OgreRectangle2D.h>
#endif

class OpenGLNativeRenderSystemCommandsRenderQueueListener : public Ogre::RenderQueueListener
{
public:
protected:
	Ogre::MovableObject* mObject;		
	const Ogre::Camera* mCamera;		
	Ogre::SceneManager* mSceneMgr;
	std::list<INativeRender*> mNativeRender;
    QPerformanceTimer2 timer;
public:
	void addRenderer(INativeRender* p) { mNativeRender.push_back(p);}
	void pushFrontRenderer(INativeRender* p) { mNativeRender.push_front(p);}

	clock_t begintime;
	clock_t endtime;

	OpenGLNativeRenderSystemCommandsRenderQueueListener(Ogre::MovableObject* object, const Ogre::Camera* camera, Ogre::SceneManager* sceneMgr) :
	  mObject(object),
	  mCamera(camera),
	  mSceneMgr(sceneMgr)
	{
	
	}
	OpenGLNativeRenderSystemCommandsRenderQueueListener()
	{
		mObject=NULL;
		mCamera=NULL;
		mSceneMgr=NULL;
	}
	void init(Ogre::MovableObject* object, const Ogre::Camera* camera, Ogre::SceneManager* sceneMgr) 
	{
		mObject=object;
		mCamera=camera;
		mSceneMgr=sceneMgr;

#if OGRE_VERSION_MINOR>=12
		// I don't know why, but this fixes some bugs
		// Create background material
		Ogre::MaterialPtr material = Ogre::MaterialManager::getSingleton().create("Background", "General");
		//material->getTechnique(0)->getPass(0)->createTextureUnitState("rockwall.tga");
		material->getTechnique(0)->getPass(0)->createTextureUnitState("solidwhite.bmp");
		material->getTechnique(0)->getPass(0)->setDepthCheckEnabled(false);
		material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(false);
		material->getTechnique(0)->getPass(0)->setLightingEnabled(false);

		// Create background rectangle covering the whole screen
		Ogre::Rectangle2D* rect = new Ogre::Rectangle2D(true);
		rect->setCorners(-1.0, 1.0, 1.0, -1.0);
		rect->setMaterial(Ogre::MaterialManager::getSingleton().getByName("Background"));

		// Render the background before everything else
		rect->setRenderQueueGroup(Ogre::RENDER_QUEUE_BACKGROUND);

		// Use infinite AAB to always stay visible
		Ogre::AxisAlignedBox aabInf;
		aabInf.setInfinite();
		rect->setBoundingBox(aabInf);

		// Attach background to the scene
		Ogre::SceneNode* node = RE::ogreSceneManager()->getRootSceneNode()->createChildSceneNode("Background");
		node->attachObject(rect);
#endif

	}

	virtual void renderQueueStarted(Ogre::uint8 queueGroupId, const Ogre::String& invocation, 
		bool& skipThisInvocation) { 
	
		// TIMESTART
        //timer.start();


		if (queueGroupId != Ogre::RENDER_QUEUE_MAIN) 
			return;

		for(std::list<INativeRender*>::iterator i=mNativeRender.begin(); i!=mNativeRender.end(); ++i)
		{
			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			
			// save attribs
			glPushAttrib(GL_ALL_ATTRIB_BITS);

			(*i)->nativePreRender();
			
			// restore original state
			glPopAttrib();
			glPopMatrix();
		}
	}
	virtual void renderQueueEnded(Ogre::uint8 queueGroupId, const Ogre::String& invocation, 
		bool& repeatThisInvocation)
	{
		// Set wanted render queue here - make sure there are - make sure that something is on
		// this queue - else you will never pass this if.
		if (queueGroupId != Ogre::RENDER_QUEUE_MAIN) 
			return;

		// save matrices
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
		glMatrixMode(GL_PROJECTION);
		glPushMatrix();
		glMatrixMode(GL_TEXTURE);
		glPushMatrix();
		glLoadIdentity(); //Texture addressing should start out as direct.

		Ogre::RenderSystem* renderSystem = mObject->_getManager()->getDestinationRenderSystem();
		Ogre::Node* parentNode = mObject->getParentNode();
		renderSystem->_setWorldMatrix(parentNode->_getFullTransform());
		renderSystem->_setViewMatrix(mCamera->getViewMatrix());
		renderSystem->_setProjectionMatrix(mCamera->getProjectionMatrixRS());


		static Ogre::Pass* clearPass = NULL;
		if (!clearPass)
		{
			Ogre::MaterialPtr clearMat = Ogre::MaterialManager::getSingleton().getByName("BaseWhite");
			clearPass = clearMat->getTechnique(0)->getPass(0);
		}
		//Set a clear pass to give the renderer a clear renderstate
		mSceneMgr->_setPass(clearPass, true, false);

		GLboolean depthTestEnabled=glIsEnabled(GL_DEPTH_TEST);
		//glDisable(GL_DEPTH_TEST);
		GLboolean stencilTestEnabled = glIsEnabled(GL_STENCIL_TEST);
		glDisable(GL_STENCIL_TEST);


		// call native rendering function
		//////////////////
		for(std::list<INativeRender*>::iterator i=mNativeRender.begin(); i!=mNativeRender.end(); ++i)
		{
			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();
			
			// save attribs
			glPushAttrib(GL_ALL_ATTRIB_BITS);

			(*i)->nativeRender();
			
			// restore original state
			glPopAttrib();
			glPopMatrix();
		}
		//////////////////


		if (depthTestEnabled)
		{
			glEnable(GL_DEPTH_TEST);
		}
		if (stencilTestEnabled)
		{
			glEnable(GL_STENCIL_TEST);
		}

		// restore matrices
		glMatrixMode(GL_TEXTURE);
		glPopMatrix();
		glMatrixMode(GL_PROJECTION);
		glPopMatrix();
		glMatrixMode(GL_MODELVIEW);
		glPopMatrix();

		// TIMEEND
        //printf("Time: %d milliseconds\n", timer.stop());
	}
};

#endif
