
#include "stdafx.h"
#include "INativeRender.h"
#include "MainLib/OgreFltk/RE.h"
#include "FltkOpenGLrenderer.h"
Fl_NR_Window::Fl_NR_Window(int x, int y, int w, int h, const char *s)
	:Fl_Window(x,y,w,h,s)
{
	((FltkOpenGLrenderer&)RE::FltkRenderer()).addRenderer(this);
}
Fl_NR_Window::Fl_NR_Window(int w, int h, const char *s)
	:Fl_Window(w,h,s)
{
	((FltkOpenGLrenderer&)RE::FltkRenderer()).addRenderer(this);
}
