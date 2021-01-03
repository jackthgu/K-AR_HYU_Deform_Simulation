#include "stdafx.h"
#include "RigidBodyWin.h"
void Register_classificationLib_bind(lua_State*L);
void Register_flexiblebody(lua_State*L);

RigidBodyWin::RigidBodyWin(int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer, const char* defaultScript, const char* _defaultScriptFolder)
:ScriptBaseWin(x,y,w,h,mp, renderer,  defaultScript,  _defaultScriptFolder)
{
}

void RigidBodyWin::initLuaEnvironment()
{
	ScriptBaseWin::initLuaEnvironment();
	Register_classificationLib_bind(L);
	Register_flexiblebody(L);
}
static int g_argc;
static char** g_argv;
static RigidBodyWin* g_win;

FlLayout* createRigidBodyWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer, int argc, char* argv[], const char * a, const char* b)
{
	g_argc=argc;
	g_argv=argv;
	return new RigidBodyWin(x,y,w,h,mp, renderer,
			a,b);
}
void RigidBodyWin::firstInit()
{
       RigidBodyWin* g_win=this;
       std::cout <<"cparam:";
       for(int i=0; i<g_argc; i++)
               std::cout << g_argv[i]<<std::endl;

       if (g_argc==2)
       {
               releaseScript();
               if (g_argv[1][0]='/')
                       loadScript( g_argv[1]);
               else
                       loadScript( defaultScriptFolder+g_argv[1]);
       }
}

void RigidBodyWin_firstInit(FlLayout* win)
{
       ((RigidBodyWin*)win)->firstInit();
}
