
#include "stdafx.h"
#include "MainLib/OgreFltk/RE.h"
class CImage;
#include "MainLib/OgreFltk/renderer.h"
#include "FltkOpenGLrenderer.h"
FltkOpenGLrenderer::FltkOpenGLrenderer(int x, int y, int w, int h, OgreRenderer* pOgreRenderer, int renderViewOffsetX, int renderViewOffsetY)
	:FltkRenderer(x,y,w,h,pOgreRenderer, renderViewOffsetX, renderViewOffsetY)
{

}
FltkOpenGLrenderer::~FltkOpenGLrenderer()
{
	RE::ogreSceneManager()->removeRenderQueueListener(this);
}
void FltkOpenGLrenderer::firstInit(Fl_Window* topmostWin)
{
	FltkRenderer::firstInit(topmostWin);

	Ogre::ManualObject *manObj; // we will use this Manual Object as a reference point for the native rendering
	manObj = RE::ogreSceneManager()->createManualObject("sampleArea");

	// Attach to child of root node, better for culling (otherwise bounds are the combination of the 2)
	RE::ogreSceneManager()->getRootSceneNode()->createChildSceneNode()->attachObject(manObj);

	Ogre::String RenderSystemName = RE::ogreSceneManager()->getDestinationRenderSystem()->getName();
	if ("OpenGL Rendering Subsystem" == RenderSystemName)
	{
		OpenGLNativeRenderSystemCommandsRenderQueueListener::init( manObj, RE::renderer().viewport().mCam, RE::ogreSceneManager());
		RE::ogreSceneManager()->addRenderQueueListener(this);
	}
}
