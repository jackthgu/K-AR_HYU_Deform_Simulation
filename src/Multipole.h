#pragma once

FlLayout* createMultipoleWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer);



///CcdPhysicsDemo shows basic stacking using Bullet physics, and allows toggle of Ccd (using key '1')
class MultipoleWin : public FlLayout, public FrameMoveObject
{
public:
	MultipoleWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer);
	~MultipoleWin (void);

	MotionPanel& m_motionPanel;
	FltkRenderer& mRenderer;

	// layout callback
	virtual void onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData);

	// FrameMoveObject
	virtual int FrameMove(float fElapsedTime);

	virtual void show();
	virtual void hide();
};