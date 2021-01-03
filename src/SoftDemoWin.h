#ifndef SOFTDEMO_WIN_H_
#define SOFTDEMO_WIN_H_
#pragma once

class MotionPanel;
class FltkRenderer;
class SoftDemo;

#include "MainLib/OgreFltk/FlLayout.h"
#include "MainLib/OgreFltk/framemoveobject.h"
class SoftDemoWin : public FlLayout, public FrameMoveObject
{
public:
	SoftDemoWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer);
	~SoftDemoWin (void);

	MotionPanel& m_motionPanel;
	FltkRenderer& mRenderer;

	SoftDemo* mDemo;
	// layout callback
	virtual void onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData);

	// FrameMoveObject
	virtual int FrameMove(float fElapsedTime);
};
#endif
