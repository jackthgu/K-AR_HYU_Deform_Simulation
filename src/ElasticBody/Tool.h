#pragma once
#ifndef TOOL_H_
#define TOOL_H_

#include "ModelingWin.h"
class SurfaceDeformer;
namespace modelingWin
{
	class DirectManipulation: public ModelingWin_Tool 
	{
		SelectVertex& mSelectVertex;
		ObjectList mObjectList;
		TString mCurrGizmo;
		vector3 mGizmoCenter;
		vector3 mGizmoOffset;
		int m_currMode;
		vector3 mAxis;
		enum {SELECT, TRANSLATE, ROTATE, SCALE};
	public:
		DirectManipulation(ModelingWin& win);

		virtual void beginTool(){ updateGizmo(); redraw();}
		
		void transformSelectedVertices(matrix4 const& mat);

		void notify(int msg);

		// translate all selected vertices while dragging
		virtual bool onLDown(int x, int y);
		virtual void onLDrag(int pushed_x, int pushed_y, int x, int y);
		virtual void onLUpNoDrag(int x, int y);
		virtual void onLUpDrag(int pushed_x, int pushed_y, int x, int y);

		virtual void redraw();

		// redraw is not called automatically.
		void updateGizmo();
		void onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData);
	};

	class LaplacianEditing : public ModelingWin_Tool
	{
		SelectVertex& mSelectVertex;
		bitvectorn mControlVertices;
		vector3N mControlVertexPos;
		ObjectList mObjectList;
		ObjectList mObjectVertices;
		TString mCurrGizmo;
		vector3 mGizmoCenter;
		vector3 mGizmoOffset;
		int m_currMode;
		vector3 mAxis;
		SurfaceDeformer* mLaplacian;
		ObjectList mGuideMeshList;
		enum {SELECT, SELECT_CONTROL_VERTICES, TRANSLATE, ROTATE, SCALE};
	public:
		LaplacianEditing (ModelingWin& win);

		virtual void beginTool(){ updateControlVertices(); updateGizmo(); redraw();}

		void updateControlVertices();
		void notify(int msg);

		void transformSelectedVertices(matrix4 const& mat);

		// translate only gizmo while dragging
		virtual bool onLDown(int x, int y);
		virtual void onLDrag(int pushed_x, int pushed_y, int x, int y);
		virtual void onLUpNoDrag(int x, int y);
		virtual void onLUpDrag(int pushed_x, int pushed_y, int x, int y);

		virtual void redraw();

		void updateGizmo();
		void onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int 
			userData);
	};

	class SelectVertexFromTetra: public UI_mode
	{
		TetraMeshLoader::TetraMesh& mesh;
		SelectVertex& sv;
	public:
		SelectVertexFromTetra(TetraMeshLoader::TetraMesh& m, SelectVertex& svv):mesh(m), sv(svv){}
		ObjectList mObjectList;
		void clearSelection()
		{
			sv.clearSelection();
		}

		virtual bool onLDown(int x, int y)
		{
			return true;
		}

		virtual void onLUpDrag(int pushed_x, int pushed_y, int x, int y)
		{
			onLUpNoDrag(x,y);
		}
		virtual void onLUpNoDrag(int x, int y);
	};

	class TetraMeshGenerator: public ModelingWin_Tool 
	{
	public:
		SelectVertexFromTetra sv;
		
		TetraMeshGenerator(ModelingWin& win);

		virtual void beginTool(){((UI_mode_router&)(*_win.m_modes[ModelingWin::TOOL_SPECIFIC])).resetCustumMode(&sv);}
		
		TetraMeshLoader::TetraMesh mTetraMesh;
		std::shared_ptr<TetraMeshLoader::TetraMeshToEntity> mpTetraMeshToEntity;

		void redrawTetraMesh();

		void onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData);
	};

	
}
#endif
