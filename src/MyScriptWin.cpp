#include "stdafx.h"
#include "MyScriptWin.h"
void Register_classificationLib_bind(lua_State*L);
void Register_QP(lua_State*L);

MyScriptWin::MyScriptWin(int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer, const char* defaultScript, const char* _defaultScriptFolder)
:ScriptBaseWin(x,y,w,h,mp, renderer,  defaultScript,  _defaultScriptFolder)
{
}

MyScriptWin::MyScriptWin(int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer, TStrings const& scripts)
:ScriptBaseWin(x,y,w,h,mp, renderer,  scripts)
{
}
void MyScriptWin::initLuaEnvironment()
{
	ScriptBaseWin::initLuaEnvironment();
	Register_classificationLib_bind(L);
	Register_QP(L);
}
static int g_argc;
static char** g_argv;
static MyScriptWin* g_win;

FlLayout* createMyScriptWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer, int argc, char* argv[])
{
	g_argc=argc;
	g_argv=argv;
	return new MyScriptWin(x,y,w,h,mp, renderer,
	"driverDynamic.lua",
	"../Samples/QP_controller/lua/");
}
FlLayout* createMyScriptWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer, int argc, char* argv[], const char * a, const char* b)
{
	g_argc=argc;
	g_argv=argv;
	return new MyScriptWin(x,y,w,h,mp, renderer,
			a,b);
}
FlLayout* createMyScriptWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer, int argc, char* argv[], TStrings const& scripts)
{
	g_argc=argc;
	g_argv=argv;
	return new MyScriptWin(x,y,w,h,mp, renderer, scripts);
}
void MyScriptWin::firstInit()
{
       MyScriptWin* g_win=this;
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

void MyScriptWin_firstInit(FlLayout* win)
{
       ((MyScriptWin*)win)->firstInit();
}
