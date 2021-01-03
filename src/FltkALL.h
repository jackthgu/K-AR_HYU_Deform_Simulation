#ifndef FLTK_ALL_H_
#define FLTK_ALL_H_

#include "MainLib/OgreFltk/FlChoice.h"
#include "MainLib/OgreFltk/Fl_Hor_Slider2.h"
#include "MainLib/OgreFltk/FltkAddon.h"
#include "MainLib/OgreFltk/FltkMotionWindow.h"
#include <Fl/Fl_Double_Window.H>
#include <Fl/Fl_Check_Button.H>
//#include "ClassificationLib/motion/TwoPosture.h"
#include "MainLib/OgreFltk/FlLayout.h"
#include "MainLib/OgreFltk/FltkScrollPanel.h"
TwoPosture& pose2(Motion const& mot, int iframe);
class FlPlot : public Fl_Widget {
protected:
	virtual void draw(){}

	Fl_Image* mSignal;
	std::vector<matrixn> mSignalSource;
public:
	FlPlot(int X, int Y, int W, int H, const char *l=0): Fl_Widget(X,Y,W,H,l), mSignal(NULL) {}
	virtual ~FlPlot() {delete mSignal;}
	void setSignal(matrixn const& scattered){}
	void addSignal(matrixn const& scattered){}

	// update mSignal and redraw.
	void update(){}
};

#endif
