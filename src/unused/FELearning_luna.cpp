
#include "stdafx.h"


#include "MainLib/WrapperLua/luna.h"
class Motion;
#include "MainLib/WrapperLua/LUAwrapper.h"
#include "BaseLib/image/Image.h"
#include "BaseLib/motion/Motion.h"
#include "BaseLib/motion/MotionLoader.h"
#include "BaseLib/math/bitVectorN.h"
#include "MainLib/OgreFltk/framemoveobject.h"
#include "MainLib/OgreFltk/timesensor.h"
#include "MainLib/OgreFltk/AnimationObject.h"
#include "MainLib/OgreFltk/FltkRenderer.h"
#include "MainLib/OgreFltk/FltkAddon.h"
#include "MainLib/OgreFltk/MotionPanel.h"
#include "MainLib/OgreFltk/pldprimskin.h"
#ifndef NO_GUI
#include <Fl/Fl_Group.H>
#include <Fl/Fl_Double_Window.H>
#include <OgrePrerequisites.h>
#include <OgreColourValue.h>
#include <OgreMovableObject.h>
#endif
#include "MainLib/OgreFltk/FlLayout.h"
#include "MainLib/WrapperLua/luna_baselib.h"
#include "MainLib/WrapperLua/luna_mainlib.h"
#include "FlexibleBodyWin.h"
#include "FELearning.h"
#include "FlexibleBodyImplicitSimpleWin.h"
#include "FlexibleBodyImplicitWin.h"
#include "luna_flexiblebody.h"
void Register_baselib(lua_State*L);
void Register_mainlib(lua_State*L);
void Register_physicsbind(lua_State*L);
void Register_classificationLib_bind(lua_State*L);
void Register_flexiblebody(lua_State*L);
#include "softbody2/Physics.h"
static void handleLUAerror(lua_State* L)
{
	printf("handleLUAerror:\n");
	luna_printStack(L);
	luaL_dostring(L, "dbg.traceBack()");
	luaL_dostring(L, "dbg.console()");
}
void FELearningWin_runLUAscript_optimize(FELearningWin& win)
{
//	TString default_script="../Resource/scripts/ui/FELearning_run.lua";
	TString default_script=FlChooseFile("choose file", "../Samples/FlexibleBody/lua/", "FELearning*.lua");

	LUAwrapper* L=new LUAwrapper();

	// export Mainlib classes and functions for use in LUA script.
	Register_baselib(L->L);
	Register_mainlib(L->L);
	Register_physicsbind(L->L);
	Register_classificationLib_bind(L->L);
	Register_flexiblebody(L->L);
	L->set<SimulatorImplicit>("world", &win.mSimulator);
	L->set<FELearningWin>("win", &win);
	if(luaL_dofile(L->L, default_script)==1)
		handleLUAerror(L->L);
	
	delete L;

}
