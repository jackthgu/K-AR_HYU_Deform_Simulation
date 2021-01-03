#pragma once

#include "../../MainLib/OgreFltk/MotionPanel.h"
#include "../../MainLib/OgreFltk/FltkMotionWindow.h"
class FltkRenderer;
class VRMLloader;
#include "../../PhysicsLib/Liegroup.h"
#include "../../PhysicsLib/ScriptBaseWin.h"
FlLayout* createMyScriptWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer);


class MyScriptWin: public ScriptBaseWin
{
public:
	MyScriptWin(int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer, const char* defaultScript, const char* _defaultScriptFolder);
	MyScriptWin(int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer, TStrings const& scripts);
	virtual ~MyScriptWin(void){}
	virtual void initLuaEnvironment();
	void firstInit();
};

