#pragma once

#include "ModelingWin.h"
#include "MainLib/OgreFltk/PointClouds.h"
#include <Fl/Fl.H>

namespace modelingWin
{
	class ChangeView:public UI_mode
	{
		ModelingWin& win;
	public:
		ChangeView(ModelingWin& w):win(w){}

		virtual bool onLDown(int x, int y)
		{ RE::renderer().viewport().m_pViewpoint->HandleMouseMessages2(Viewpoint::LBUTTONDOWN,  x, y); return true;}
		virtual void onLDrag(int pushed_x, int pushed_y, int x, int y)
		{ RE::renderer().viewport().m_pViewpoint->HandleMouseMessages2(Viewpoint::MOUSEMOVE,  x, y); }
		virtual void onLUpNoDrag(int x, int y)
		{ RE::renderer().viewport().m_pViewpoint->HandleMouseMessages2(Viewpoint::LBUTTONUP,  x, y); }
		virtual void onLUpDrag(int pushed_x, int pushed_y, int x, int y)
		{ RE::renderer().viewport().m_pViewpoint->HandleMouseMessages2(Viewpoint::LBUTTONUP,  x, y); }
		virtual bool onMDown(int x, int y)
		{ RE::renderer().viewport().m_pViewpoint->HandleMouseMessages2(Viewpoint::RBUTTONDOWN,  x, y); return true;}
		virtual void onMDrag(int pushed_x, int pushed_y, int x, int y)
		{ RE::renderer().viewport().m_pViewpoint->HandleMouseMessages2(Viewpoint::MOUSEMOVE,  x, y); }
		virtual void onMUpNoDrag(int x, int y)
		{ RE::renderer().viewport().m_pViewpoint->HandleMouseMessages2(Viewpoint::RBUTTONUP,  x, y); }
		virtual void onMUpDrag(int pushed_x, int pushed_y, int x, int y)
		{ RE::renderer().viewport().m_pViewpoint->HandleMouseMessages2(Viewpoint::RBUTTONUP,  x, y); }
		virtual bool onRDown(int x, int y)
		{ RE::renderer().viewport().m_pViewpoint->HandleMouseMessages2(Viewpoint::MBUTTONDOWN,  x, y); return true;}
		virtual void onRDrag(int pushed_x, int pushed_y, int x, int y)
		{ RE::renderer().viewport().m_pViewpoint->HandleMouseMessages2(Viewpoint::MOUSEMOVE,  x, y); }
		virtual void onRUpNoDrag(int x, int y)
		{ RE::renderer().viewport().m_pViewpoint->HandleMouseMessages2(Viewpoint::MBUTTONUP,  x, y); }
		virtual void onRUpDrag(int pushed_x, int pushed_y, int x, int y)
		{ RE::renderer().viewport().m_pViewpoint->HandleMouseMessages2(Viewpoint::MBUTTONUP,  x, y); }


		virtual void beginMode(){win.mRenderer.viewUnlock();}
		virtual bool endMode(){win.mRenderer.viewLock(); return true;}
	};

	class VertexCorrespondence: public UI_mode
	{
		ModelingWin& win;

		int state;
	public:
		VertexCorrespondence(ModelingWin& w):win(w),state(0){}
		ObjectList mObjectList;
		intvectorn source;
		intvectorn target;

		virtual bool onLDown(int x, int y);
		virtual bool onRDown(int x, int y);
		virtual void onRUpNoDrag(int x, int y);
		virtual void onLUpDrag(int pushed_x, int pushed_y, int x, int y)
		{
			onLUpNoDrag(x,y);
		}
		void addVertexNotification(char c, int i, int ii);

		virtual void onLUpNoDrag(int x, int y);
	};

	class SelectVertex: public UI_mode
	{
	public:
		QuadList* mDrawVertex;
		void notify(int msg);
		ModelingWin& win;
		SelectionRectangle* mRect;
		bitvectorn mSelectedVertices;
		ObjectList mObjectList;
		void clearSelection();
			
		SelectVertex(ModelingWin& w):win(w),mRect(NULL),mDrawVertex(NULL){}
		virtual bool onLDown(int x, int y);
		virtual void onLDrag(int pushed_x, int pushed_y, int x, int y);
		virtual void onLUpNoDrag(int x, int y);

		void redraw()
		{
			drawSelection();
		}

		void drawSelection();
			
		virtual void onLUpDrag(int pushed_x, int pushed_y, int x, int y);
			

	};

	class SelectEdge: public UI_mode
	{
		ModelingWin& win;
	public:
		bitvectorn mSelectedEdges;
		ObjectList mObjectList;

		SelectEdge(ModelingWin& w):win(w){}
		void clearSelection()
		{
			if(mSelectedEdges.size()!=win.numEdges())
				mSelectedEdges.resize(win.numEdges());

			if(mSelectedEdges.size())
			mSelectedEdges.clearAll();
			mObjectList.clear();

		}

		virtual bool onLDown(int x, int y)
		{
			if(Fl::event_alt()) return false;	// so that viewpoint can be managed.
			return true;
		}
		virtual void onLUpDrag(int pushed_x, int pushed_y, int x, int y)
		{
			onLUpNoDrag(x,y);
		}
		virtual void onLUpNoDrag(int x, int y)
		{
			Ray ray;
			win.mRenderer.screenToWorldRay(x,y,ray);
			intersectionTest::LineSegment sx=intersectionTest::Ray2LineSegment(ray);

			m_real minDist=FLT_MAX;
			int argMin=-1;
			for(int i=0; i<win.numEdges(); i++)
			{
				intersectionTest::LineSegment l(win.mesh()->getVertex(win.edges()->source(i)), win.mesh()->getVertex(win.edges()->target(i)));
				m_real dist=l.minDist(sx);
				if(dist<minDist)
				{
					minDist=dist;
					argMin=i;
				}
			}

			clearSelection();

			if(minDist<1)
			{
				mSelectedEdges.setAt(argMin);
				redraw();
			}
		}

		virtual void redraw()
		{
			mObjectList.clear();

			for(int i=0; i<mSelectedEdges.size(); i++)
			{
				if(mSelectedEdges[i])
				{
					BillboardLineList * line=new BillboardLineList ("edge", 1,1);
					line->setMaterialName("redToOrange");
					line->line(0, win.mesh()->getVertex(win.edges()->source(i)), win.mesh()->getVertex(win.edges()->target(i)));
					mObjectList.registerObject("link", line);
					RE::moveEntity(mObjectList.registerEntity("sellink0","sphere1010.mesh"), quater(1,0,0,0), win.mesh()->getVertex(win.edges()->source(i)));
					RE::moveEntity(mObjectList.registerEntity("sellink1","sphere1010.mesh"), quater(1,0,0,0), win.mesh()->getVertex(win.edges()->target(i)));
				}
			}
		}
	};

	class SelectTriangle: public UI_mode
	{
		ModelingWin& win;
	public:
		SelectTriangle(ModelingWin& w):win(w){}
		ObjectList mObjectList;
		bitvectorn mSelectedTriangles;
		void clearSelection()
		{
			if(mSelectedTriangles.size())
			mSelectedTriangles.clearAll();
			mObjectList.clear();

		}

		virtual bool onLDown(int x, int y)
		{
			if(Fl::event_alt()) return false;	// so that viewpoint can be managed.
			return true;
		}

		virtual void onLUpDrag(int pushed_x, int pushed_y, int x, int y)
		{
			onLUpNoDrag(x,y);
		}
		virtual void onLUpNoDrag(int x, int y)
		{
			Ray ray;
			win.mRenderer.screenToWorldRay(x,y,ray);

			intvectorn colTri;
			vectorn colZ;
			OBJloader::Mesh& m=*win.mesh();

			for(int i=0; i<win.mesh()->numFace(); i++)
			{
				std::pair<bool, m_real> res=ray.intersects(
					m.getFaceCorner(i,0), 
					m.getFaceCorner(i,1), 
					m.getFaceCorner(i,2));

				if(res.first)
				{
					colTri.push_back(i);
					colZ.pushBack(res.second);
				}
			}

			clearSelection();
			
			if(colZ.size())
			{
				int selTri=colTri[colZ.argMin()];

				BillboardLineList * line=new BillboardLineList ("tri", 3,1);
				line->setMaterialName("redToOrange");

				vector3 v0=m.getFaceCorner(selTri,0); 
				vector3 v1=m.getFaceCorner(selTri,1); 
				vector3 v2=m.getFaceCorner(selTri,2);

				line->line(0, v0, v1);
				line->line(1, v1, v2);
				line->line(2, v2, v0);

				
				mObjectList.registerObject("tri", line);
			}
		}
	};

	class UI_mode_router: public UI_mode
	{
		UI_mode* mpCustumMode;
	public:
		UI_mode_router() :UI_mode() {mpCustumMode=NULL;}

		virtual bool onLDown(int x, int y){ if(mpCustumMode) return mpCustumMode->onLDown(x,y); return false;}
		virtual void onLDrag(int pushed_x, int pushed_y, int x, int y){if(mpCustumMode) mpCustumMode->onLDrag(pushed_x, pushed_y, x, y);}
		virtual void onLUpNoDrag(int x, int y){if(mpCustumMode) mpCustumMode->onLUpNoDrag(x,y);}
		virtual void onLUpDrag(int pushed_x, int pushed_y, int x, int y){if(mpCustumMode) mpCustumMode->onLUpDrag(pushed_x, pushed_y, x, y);}
	    
		virtual bool onRDown(int x, int y){if(mpCustumMode) return mpCustumMode->onRDown(x,y);return false;}
		virtual void onRDrag(int pushed_x, int pushed_y, int x, int y){if(mpCustumMode) mpCustumMode->onRDrag(pushed_x, pushed_y,x,y);}
		virtual void onRUpNoDrag(int x, int y){if(mpCustumMode) mpCustumMode->onRUpNoDrag(x,y);}
		virtual void onRUpDrag(int pushed_x, int pushed_y, int x, int y){if(mpCustumMode) mpCustumMode->onRUpDrag(pushed_x, pushed_y,x,y);}
	    

		virtual void beginMode(){if(mpCustumMode) mpCustumMode->beginMode();}
		virtual bool endMode(){ if(mpCustumMode) return mpCustumMode->endMode(); return true; }
		virtual void onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData)
		{
			if(mpCustumMode) mpCustumMode->onCallback(w,pWidget, userData);
		}

		bool resetCustumMode(UI_mode* pCustumMode) 
		{ 
			if(mpCustumMode) 
			{
				bool res=mpCustumMode->endMode();
				if(!res)
					return false;
			}

			mpCustumMode=pCustumMode;	// beginMode()?? ?ʿ?????. ?????? ???߿? call?? ???̱? ????.
		}
	};
	
}
