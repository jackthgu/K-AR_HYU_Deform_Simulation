#ifndef FltkOpenGLrenderer_H_
#define FltkOpenGLrenderer_H_
#include "MainLib/OgreFltk/FltkRenderer.h"
#include "OpenGLNativeRenderSystem.hpp"

class FltkOpenGLrenderer : public FltkRenderer, public OpenGLNativeRenderSystemCommandsRenderQueueListener
{
	public:
		FltkOpenGLrenderer(int x, int y, int w, int h, OgreRenderer* pOgreRenderer, int renderViewOffsetX=0, int renderViewOffsetY=0);
		virtual ~FltkOpenGLrenderer();

		virtual void firstInit(Fl_Window* topmostWin);

		// use the following function to draw openGL objects.
		//void OpenGLNativeRenderSystemCommandsRenderQueueListener::
		//addRenderer(INativeRender* p) { mNativeRender.push_back(p);}
};
#endif
