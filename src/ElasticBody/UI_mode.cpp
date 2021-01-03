#include "stdafx.h"
#include <OgreMovableObject.h>
#include <OgreRenderable.h>
#include "MainLib/OgreFltk/MovableText.h"
#include "BaseLib/image/Image.h"
#include "UI_mode.h"
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>
void UI_mode::setFallback(UI_mode* fallback)
{
	Msg::verify(mState!=FALLBACK, "cannot change fallback");
	mFallback=fallback;
}

int    UI_mode::handleRendererEvent(int ev)
{
	switch(ev)
	{
	case FL_MOVE:
		if(mState==READY)
		{
			onMove(Fl::event_x(), Fl::event_y());
			if(mFallback)
				mFallback->onMove(Fl::event_x(), Fl::event_y());
		}
		break;
	case FL_PUSH:            
		if(mState==READY)
		{
			pushed_x=Fl::event_x();
			pushed_y=Fl::event_y();
			
			if(Fl::event_state()&FL_BUTTON1)
			{
				if(onLDown(pushed_x, pushed_y))
				{
					mState=LDOWN;     
					return 1;
				}
				else if(mFallback&& mFallback->handleRendererEvent(ev))
				{
					mState=FALLBACK;
					return 1;
				}
			}

			if(Fl::event_state()&FL_BUTTON2)
			{
				if(onMDown(pushed_x, pushed_y))
				{
					mState=MDOWN;     
					return 1;
				}
				else if(mFallback&& mFallback->handleRendererEvent(ev))
				{
					mState=FALLBACK;
					return 1;
				}
			}

			if(Fl::event_state()&FL_BUTTON3)
			{
				if(onRDown(pushed_x, pushed_y))
				{
					mState=RDOWN;
					return 1;
				}
				else if(mFallback && mFallback->handleRendererEvent(ev))
				{
					mState=FALLBACK;
					return 1;
				}
			}
		}
		break;
	case FL_DRAG:
		if(mState==LDOWN || mState==LDRAG)				
		{
			onLDrag(pushed_x, pushed_y, Fl::event_x(), Fl::event_y());
			mState=LDRAG;
			return 1;				
		}
		
		if(mState==MDOWN || mState==MDRAG)				
		{
			onMDrag(pushed_x, pushed_y, Fl::event_x(), Fl::event_y());
			mState=MDRAG;
			return 1;				
		}

		if(mState==RDOWN || mState==RDRAG)				
		{
			onRDrag(pushed_x, pushed_y, Fl::event_x(), Fl::event_y());
			mState=RDRAG;
			return 1;				
		}

		if(mState==FALLBACK)
		{
			Msg::verify(mFallback, "?? fallback removed");
			return mFallback->handleRendererEvent(ev);
		}
		break;
	case FL_RELEASE:
	{
		int x=Fl::event_x();
		int y=Fl::event_y();
		if(mState==LDOWN)
		{
			ASSERT(pushed_x==x && pushed_y==y);
			onLUpNoDrag(x, y);
			mState=READY;
			return 1;
		}
		else if(mState==LDRAG)
		{
			onLUpDrag(pushed_x, pushed_y, x, y);
			mState=READY;
			return 1;
		}
		else if(mState==MDOWN)
		{
			ASSERT(pushed_x==x && pushed_y==y);
			onMUpNoDrag(x, y);
			mState=READY;
			return 1;
		}
		else if(mState==MDRAG)
		{
			onMUpDrag(pushed_x, pushed_y, x, y);
			mState=READY;
			return 1;
		}
		else if(mState==RDOWN)
		{
			ASSERT(pushed_x==x && pushed_y==y);
			onRUpNoDrag(x, y);
			mState=READY;
			return 1;
		}
		else if(mState==RDRAG)
		{
			onRUpDrag(pushed_x, pushed_y, x, y);
			mState=READY;
			return 1;
		}
		else if(mState==FALLBACK)
		{
			Msg::verify(mFallback, "?? fallback removed");
			int res=mFallback->handleRendererEvent(ev);

			mState=READY;
			return res;
		}
	}
	break;
			
	}

	return 0;
}	

using namespace modelingWin;

void SelectVertex::notify(int msg)
{
	switch(msg)
	{
	case MESH_VERTEX_MODIFIED:
		redraw();
		break;
	case MESH_CHANGED:
		clearSelection();
		redraw();
		break;
	}
}

void SelectVertex::clearSelection()
{
	if(mSelectedVertices.size()!=win.numVertex())
		mSelectedVertices.resize(win.numVertex());

	if(mSelectedVertices.size())
		mSelectedVertices.clearAll();

	mObjectList.clear();
}

void SelectVertex::onLDrag(int pushed_x, int pushed_y, int x, int y)
{
	if(mRect)
	{
		mRect->setCorners(TRect(pushed_x,pushed_y,Fl::event_x(),Fl::event_y()));
	}
}
void SelectVertex::onLUpDrag(int pushed_x, int pushed_y, int x, int y)
{
	if(mRect)
	{
		// marquee selection
		mRect->setCorners(TRect(pushed_x,pushed_y,Fl::event_x(),Fl::event_y()));
		std::vector<Plane> vol;
		mRect->constructSelectionVolumn(win.mRenderer, vol);

		bitvectorn backup=mSelectedVertices;

		clearSelection();
		for(int i=0; i<win.numVertex(); i++)
		{
			Sphere s(win.mesh()->getVertex(i), 0.001);
			if(s.isInside(vol))
				mSelectedVertices.setAt(i);
		}

		if((Fl::event_state()&FL_SHIFT)!=0 && backup.size()==mSelectedVertices.size())
			mSelectedVertices|=backup;

		drawSelection();
		RE::removeEntity("MarqueeNode");
		mRect=NULL;
	}
}
void SelectVertex::drawSelection()
{
	mObjectList.clear();

	mDrawVertex=new QuadList(vector3(0,1,0), 0.4);
	mDrawVertex->setCastShadows(false);
	mDrawVertex->setMaterial("lightBlueCircle");

	mDrawVertex->begin(mSelectedVertices.count());

	int c=0;
	for(int i=0; i<win.numVertex(); i++)
	{
		if(mSelectedVertices[i])
		{
			mDrawVertex->quad(c++, win.mesh()->getVertex(i));
		}					
	}
	mDrawVertex->end();
	mObjectList.registerObject(RE::generateUniqueName(), mDrawVertex);
}
void SelectVertex::onLUpNoDrag(int x, int y)
{
	// click selection
	if(mRect)
	{
		RE::removeEntity("MarqueeNode");
		mRect=NULL;
	}

	Ray ray;
	win.mRenderer.screenToWorldRay(x,y,ray);

	bool bCtrl=(Fl::event_state()&FL_CTRL);
	if(bCtrl==0)
		clearSelection();


	for(int i=0; i<win.numVertex(); i++)
	{
		Sphere s(win.mesh()->getVertex(i), 0.5);
		if(ray.intersects(s).first)
		{
			if(bCtrl)
				if (mSelectedVertices(i))
					mSelectedVertices.clearAt(i);
				else
					mSelectedVertices.setAt(i);
			else
				mSelectedVertices.setAt(i);
		}					
	}

	drawSelection();
}

bool SelectVertex::onLDown(int x, int y)
{
	if(Fl::event_alt()) return false;	// so that viewpoint can be managed.
	Ogre::SceneNode* pNode=RE::createSceneNode("MarqueeNode");
	mRect = new SelectionRectangle("Selection SelectionRectangle");
	pNode->attachObject(mRect);

	mRect->setCorners(TRect(x,y,x,y));
	return true;
}

bool VertexCorrespondence::onLDown(int x, int y)
{
	if(Fl::event_alt()) return false;	// so that viewpoint can be managed.
	return true;
}

bool VertexCorrespondence::onRDown(int x, int y)
{
	if(state>0 && state<10000)
		return true;
	return false;
}

void VertexCorrespondence::onRUpNoDrag(int x, int y)
{
	state+=10000;
	Msg::msgBox("Now you can select target vertices");
}


void VertexCorrespondence::addVertexNotification(char c, int i, int ii)
{
	vector3 pos=win.mesh()->getVertex(i)+win.mesh()->getNormal(i)*0.8;

	TString tt;
	tt.format("%c%d",c, ii);
	Ogre::MovableText* text=new Ogre::MovableText(RE::generateUniqueName().ptr(), tt.ptr(), "BlueHighway", 1, Ogre::ColourValue::Black);

	mObjectList.registerObject(tt+"__node", text)->translate(ToOgre(pos));
}
void VertexCorrespondence::onLUpNoDrag(int x, int y)
{
	if(state==0)
	{
		source.resize(0);
		target.resize(0);
		mObjectList.clear();
	}
	else if(state==-1)
	{
		mObjectList.clear();
		state=0;
		return;
	}

	Ray ray;
	win.mRenderer.screenToWorldRay(x,y,ray);

	intvectorn indexes;
	vectorn z;
	for(int i=0; i<win.numVertex(); i++)
	{
		Sphere s(win.mesh()->getVertex(i), 0.5);
		std::pair<bool, m_real> res=ray.intersects(s);
		if(res.first)
		{
			indexes.push_back(i);
			z.pushBack(res.second);
		}					
	}

	if(z.size())
	{
		int selV=indexes[z.argMin()];

		if(state<10000)
		{
			if(source.findFirstIndex(selV)==-1)
			{
				source.push_back(selV);

				addVertexNotification('s',selV, source.size());
				state++;
			}
		}
		else
		{
			if(target.findFirstIndex(selV)==-1)
			{
				target.push_back(selV);
				addVertexNotification('t',selV,  target.size());
				state++;
			}

			if(source.size()==target.size())
			{
				state=-1;
				Msg::msgBox("Selection ok");
			}
		}
	}
}
