//
	 
#include "stdafx.h"
#include "MainLib/MainLib.h"

#include "MainLib/OgreFltk/FltkAddon.h"
#include "MainLib/OgreFltk/FlLayout.h"
#include "MainLib/OgreFltk/FlChoice.h"


#include <math.h>
#include "BaseLib/motion/Motion.h"
#include "BaseLib/utility/operatorString.h"
#include "MainLib/OgreFltk/MotionPanel.h"
#include "MainLib/OgreFltk/fastCapture.h"
#include "MainLib/OgreFltk/MotionManager.h"
#include "MainLib/OgreFltk/FltkRenderer.h"
#include "MainLib/OgreFltk/Line3D.h"
#include "FltkOpenGLrenderer.h"
#include "TestWin.h"
//#include "TestWin2.h"
//#include "SoftDemoWin.h"

#include <Ogre.h>
#include <Fl/Fl_Tile.H>
#include <Vision/realsense3.h>

#include "OgreFltk.h"
MotionPanel* g_motionPanel=NULL;
void Register_baselib(lua_State*L);
void Register_mainlib(lua_State*L);


#ifdef USE_BULLETDEP
FlLayout* createFlexibleBodyWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer);
FlLayout* createFlexibleBodyImplicitWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer);
FlLayout* createFlexibleBodyImplicitSimpleWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer);
FlLayout* createFlexibleBodyImplicitSimplePenaltyWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer);
FlLayout* createFlexibleBodyImplicitSimpleLCPWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer);
FlLayout* createIguanaLCPWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer);
#endif
FlLayout* createRigidBodyWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer, int argc, char* argv[], const char* a, const char* b);
FlLayout* createSpringEditWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer);
FlLayout* createTetraMeshEditWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer);
//FlLayout* createTetraDeformTestWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer);
FlLayout* createTetgenWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer);
void RigidBodyWin_firstInit(FlLayout* l);


std::vector<FlLayout*> g_testWin;
MainWin::MainWin(int w, int h, int rw, int rh, OgreRenderer * pOgreRenderer, int argc, char* argv[], const char* title) 
	: Fl_Window(w, h, title)		
{
	m_tile=new Fl_Tile (0,0,w,h);
	int RENDERER_WIDTH=rw;
	int RENDERER_HEIGHT=rh;
	int WIDTH=w;
	int HEIGHT=h;


	
	m_motionPanel=new MotionPanel(0,RENDERER_HEIGHT,WIDTH, HEIGHT-RENDERER_HEIGHT);
	m_Renderer=new FltkOpenGLrenderer(0,0,RENDERER_WIDTH,RENDERER_HEIGHT,pOgreRenderer);
	m_Renderer->end();


	
	int ww, hh;

	ww=WIDTH-RENDERER_WIDTH;
	hh=RENDERER_HEIGHT;

#ifdef USE_BULLETDEP
	int numWin=10;
#else
	int numWin=5;
#endif
	m_pParentWin=new FlChoiceWins(RENDERER_WIDTH, 0, ww, hh, numWin);
	{
		int curWin=0;
#ifdef USE_BULLETDEP
		//m_pParentWin->window(curWin++, "flexible body", createFlexibleBodyWin(0,0,ww,hh, *m_motionPanel, *m_Renderer));
		m_pParentWin->window(curWin++, "flexible body implicit", createFlexibleBodyImplicitWin(0,0,ww,hh, *m_motionPanel, *m_Renderer));
		m_pParentWin->window(curWin++, "flexible body implicit simple", createFlexibleBodyImplicitSimpleWin(0,0,ww,hh, *m_motionPanel, *m_Renderer));
		m_pParentWin->window(curWin++, "test penalty", createFlexibleBodyImplicitSimplePenaltyWin(0,0,ww,hh, *m_motionPanel, *m_Renderer));
		m_pParentWin->window(curWin++, "test scripts", createFlexibleBodyImplicitSimpleLCPWin(0,0,ww,hh, *m_motionPanel, *m_Renderer));
		m_pParentWin->window(curWin++, "Iguana LCP", createIguanaLCPWin(0,0,ww,hh, *m_motionPanel, *m_Renderer));
#endif
		m_pParentWin->window(curWin++, "spring edit win", createSpringEditWin (0,0,ww,hh, *m_motionPanel, *m_Renderer));
		m_pParentWin->window(curWin++, "tetmesh edit win", createTetraMeshEditWin (0,0,ww,hh, *m_motionPanel, *m_Renderer));
		//m_pParentWin->window(curWin++, "tetmesh deform test", createTetraDeformTestWin (0,0,ww,hh, *m_motionPanel, *m_Renderer));
		//m_pParentWin->window(curWin++, "soft demo", new SoftDemoWin(0,0,ww,hh, *m_motionPanel, *m_Renderer));
		//m_pParentWin->window(curWin++, "tools", createTools(ww,hh));
		g_testWin.push_back(createRigidBodyWin(RENDERER_WIDTH, 0, ww, hh, *m_motionPanel, *m_Renderer , argc, argv, "IguanaLCP2.lua", "../src/lua/"));
		m_pParentWin->window(curWin++, "bunny", g_testWin.back());

		g_testWin.push_back(new TestWin(RENDERER_WIDTH, 0, ww, hh, *m_motionPanel, *m_Renderer));
		m_pParentWin->window(curWin++, "opengl", g_testWin.back());

		//g_testWin.push_back(new TestWin2(RENDERER_WIDTH, 0, ww, hh, *m_motionPanel, *m_Renderer));
		m_pParentWin->window(curWin++, "tw2", g_testWin.back());
		
		Msg::verify(curWin==numWin, "NumWindow Error");
	}

	m_tile->end();
	end();		

	resizable(this);

	g_motionPanel=m_motionPanel;
}

//#include "ClassificationLib/motion/SegmentationWin.h"
//#include "ClassificationLib/motion/ClassifyWin.h"
//#include "ClassificationLib/motion/ClassifyBySVM.h"
//#include "ClassificationLib/motion/ClassifyBySVMs.h"
//#include "ClassificationLib/motion/ClassifyPoseBySVMs.h"
//#include "ClassificationLib/motion/ParamSpaceWin.h"
//#include "ClassificationLib/motion/PoseTrajWin.h"

extern ConfigTable config;
FlChoiceWins* MainWin::createTools(int ww,int hh)
{
	int RENDERER_WIDTH=config.GetInt("renderer_width");

	FlChoiceWins* unused=new FlChoiceWins(RENDERER_WIDTH, 0, ww, hh, 5, "SubMenu");
	{
		int curWin=0;
		//unused->window(curWin++, "Segmentation_Old", new SegmentationWin(0,0,ww,hh,m_motionPanel));
		//unused->window(curWin++, "Clustering", new ClassifyWin2(0,0,ww,hh,m_motionPanel));
		//unused->window(curWin++, "Clustering_Old", new ClusteringWin(0,0,ww,hh,m_motionPanel));
		//unused->window(curWin++, "ClassificationSVM", new ClassifyBySVM(0,0,ww,hh, m_motionPanel));	
		//unused->window(curWin++, "ClassificationSVMs", new ClassifyBySVMs(0,0,ww,hh, m_motionPanel));	
		//unused->window(curWin++, "Show GRC", new MotionClustering::ShowGRC(0,0,ww,hh,*m_motionPanel));	
		//unused->window(curWin++, "ShowPoses", new PoseTrajWin(0,0,ww,hh, m_motionPanel));	
		//unused->window(curWin++, "ShowSegmentation", new ShowSegmentsWin(0,0,ww,hh, m_motionPanel));	
		unused->end();		
	}
	return unused;
}
int MainWin::handle(int ev)
{
	return Fl_Window::handle(ev);
}

MainWin::~MainWin()
{
	// manually removed g_testWin so that its destructor is called before those of other sub-windows.
	for (int i=0; i<g_testWin.size(); i++)
	{
		m_tile->remove(g_testWin[i]);
		delete g_testWin[i];
		g_testWin[i]=NULL;
	}
}
OgreRenderer* renderer;
extern int mScaleFactor;

MainWin* createMainWin(int argc, char* argv[])
{
	if (!Fl::visual(FL_DOUBLE|FL_INDEX))
		printf("Xdbe not supported, faking double buffer with pixmaps.\n"); 
	Fl::scheme("plastic");

	OgreRenderer* prenderer=NULL;
	if(argc>=2 && TString(argv[1])=="--sep")
	{
		argc--;
		for(int i=1; i<argc; i++)
			argv[i]=argv[i+1];
#if !defined(_MSC_VER )&& !defined(__APPLE__)
		// linux - seperate window
#if OGRE_VERSION_MINOR >= 12 
		prenderer=new OgreRenderer("../Resource/ogreconfig_personal_sepwin.txt", "../Resource/ogreconfig_linux_sepwin.txt", "plugins_linux12.cfg", "ogre_linux12.cfg");
#else
		prenderer=new OgreRenderer("../Resource/ogreconfig_personal_sepwin.txt", "../Resource/ogreconfig_linux_sepwin.txt", "plugins_linux.cfg", "ogre_linux.cfg");
#endif
#endif
	}
	if(!prenderer)
		prenderer=new OgreRenderer();
	renderer=prenderer;
	int RENDERER_WIDTH=config.GetInt("renderer_width");
	int RENDERER_HEIGHT=config.GetInt("renderer_height");
	int WIDTH=RENDERER_WIDTH+config.GetInt("right_panel_width");
	int HEIGHT=RENDERER_HEIGHT+config.GetInt("right_panel_width");
	FlLayout::setUIscaleFactor(config.GetFloat("UI_scale_factor"));
	MainWin* win=new MainWin(WIDTH, HEIGHT, RENDERER_WIDTH, RENDERER_HEIGHT,renderer, argc, argv, "KickBoxer");
	win->show();

	win->m_Renderer->firstInit(win);
	renderer->mRoot->addFrameListener(win);

	try
	{
		{
			LUAwrapper L;
			Register_baselib(L.L);
			Register_mainlib(L.L);

			L.dofile("../Resource/scripts/loadBG_default.lua");
		}
		for (int i=0; i<g_testWin.size()-1; i++)
			RigidBodyWin_firstInit(g_testWin[i]);
	}
	catch (char* error)
	{
		Msg::msgBox("%s", error);
		assert(false);
	}
	catch(std::exception& e)
	{
		Msg::msgBox("c++ error : %s", e.what());
		ASSERT(0);
	}
	catch(...)
	{
		Msg::msgBox("some error");
		ASSERT(0);
	}
	return win;
}
void cleanup(MainWin* win)
{
	delete win;
	delete renderer;
}

#include <Fl/Fl_Gl_Window.H>
//////////////////#include <Fl/Gl.H>
//#include <Gl/Glu.h>

class TestGlWindow : public Fl_Gl_Window
{
	public:
	TestGlWindow(int width, int height, const char* title) : Fl_Gl_Window(width, height, title)
	{
		mode(FL_RGB | FL_ALPHA | FL_DEPTH | FL_DOUBLE);
	}
	void InitializeGL()
	{
		glClearColor(.1f, .1f, .1f, 1);
		glEnable(GL_DEPTH_TEST);

		// view transformations
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glFrustum(-1, 1, -1, 1, 1, 100);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		gluLookAt(0, 0, 3, 0, 0, 0, 0, 1, 0);
		// draw cube
		glBegin(GL_QUADS);

		// front
		glColor3f(1, 0, 0);
		glVertex3f(-1, 1, 1);
		glVertex3f(-1, -1, 1);
		glVertex3f(1, -1, 1);
		glVertex3f(1, 1, 1);

		// back
		glColor3f(0, 1, 0);
		glVertex3f(-1, 1, -1);
		glVertex3f(1, 1, -1);
		glVertex3f(1, -1, -1);
		glVertex3f(-1, -1, -1);

		// top
		glColor3f(0, 0, 1);
		glVertex3f(-1, 1, -1);
		glVertex3f(-1, 1, 1);
		glVertex3f(1, 1, 1);
		glVertex3f(1, 1, -1);

		// bottom
		glColor3f(1, 1, 0);
		glVertex3f(-1, -1, -1);
		glVertex3f(1, -1, -1);
		glVertex3f(1, -1, 1);
		glVertex3f(-1, -1, 1);

		// left
		glColor3f(0, 1, 1);
		glVertex3f(-1, 1, -1);
		glVertex3f(-1, -1, -1);
		glVertex3f(-1, -1, 1);
		glVertex3f(-1, 1, 1);

		// right
		glColor3f(1, 0, 1);
		glVertex3f(1, 1, 1);
		glVertex3f(1, -1, 1);
		glVertex3f(1, -1, -1);
		glVertex3f(1, 1, -1);
		glEnd();
	}


	void draw()
	{
		static bool firstTime = true;
		if (firstTime)
		{
			InitializeGL();
			firstTime = false;
		}// if

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);      // clear the color and depth buffer

		// view transformations

		// draw something
	}
};
//#ifdef _MSC_VER
//int _tmain(int argc, char* argv[])
//#else
int main(int argc, char* argv[])
//#endif
{

	//srand((unsigned)time( NULL ));
	MainWin* win=createMainWin(argc, argv);
	
   //TestGlWindow myWindow(400, 400, "CS447 Tutorial");
   //myWindow.show();
	try {
		for (;;)
		{
			if (win->visible())
			{
				if (!Fl::check())
					break;	// returns immediately
			}
			else
			{
				if (!Fl::wait()) break;	// waits until something happens
			}
			// idle time


			//win->m_Renderer->make_current();
			win->m_Renderer->ogreRenderer().renderOneFrame();

			//VisionProject::GetInstance()->RenderingLoop();
			//win->m_Renderer->m_RenderView->make_current();
		}
	}
	catch( Ogre::Exception& e ) 
	{
		Msg::msgBox(e.getFullDescription().c_str());

	}	
	catch(char * error)
	{
		Msg::msgBox("%s", error);
	}
	catch(const char* error)
	{
		Msg::msgBox("%s", error);
	}
	catch(std::exception& e)
	{
		Msg::msgBox("c++ error : %s", e.what());
		ASSERT(0);
	}
	catch(...)
	{
		Msg::msgBox("some error");
		ASSERT(0);
	}

	cleanup(win);
	return 0;
}

