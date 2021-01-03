#ifndef INativeRender_H_
#define INativeRender_H_

#include <FL/Fl_Window.H>
class INativeRender
{
	public:
	virtual void nativePreRender(){};  // in this function, restore all the renderstates if you change any.
	virtual void nativeRender()=0;
};

class Fl_NR_Window : public INativeRender, public Fl_Window
{
	public :
	Fl_NR_Window(int x, int y, int w, int h, const char *s=0);
	Fl_NR_Window(int w, int h, const char *s=0);
};
#endif
