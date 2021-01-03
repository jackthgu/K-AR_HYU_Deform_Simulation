#ifndef MODELINGWIN_H
#define MODELINGWIN_H
#pragma once
#include "MainLib/OgreFltk/MotionPanel.h"
#include "MainLib/OgreFltk/FltkAddon.h"
#include "MainLib/OgreFltk/MotionManager.h"
#include "MainLib/OgreFltk/objectList.h"
#include "BaseLib/utility/operatorString.h"
#include "BaseLib/utility/scoped_ptr.h"
#include "MainLib/OgreFltk/Mesh.h"
#include "MainLib/OgreFltk/OgreMotionLoader.h"
#include "MainLib/OgreFltk/FlLayout.h"
#include "MainLib/OgreFltk/FltkRenderer.h"
#include "MainLib/OgreFltk/FltkMotionWindow.h"
#include "MainLib/OgreFltk/Line3D.h"
class ModelingWin;
class UI_mode
{
	int pushed_x, pushed_y;
	
	int mState;
	friend class ModelingWin;
	UI_mode* mFallback;
protected:
	enum { READY, LDOWN, LDRAG, MDOWN, MDRAG, RDOWN, RDRAG, FALLBACK, UI_MODE_IGNORE};
	int getUIState()	{ return mState;}
	int    handleRendererEvent(int ev);

	void setFallback(UI_mode* mFallback);
public:
  
	enum { MESH_VERTEX_MODIFIED, MESH_CHANGED, SELECTION_CHANGED};
	virtual void notify(int msg){}

	virtual void onMove(int x, int y){}
	virtual bool onLDown(int x, int y){return false;}
	virtual void onLDrag(int pushed_x, int pushed_y, int x, int y){}
	virtual void onLUpNoDrag(int x, int y){}
	virtual void onLUpDrag(int pushed_x, int pushed_y, int x, int y){}

	virtual bool onMDown(int x, int y){return false;}
	virtual void onMDrag(int pushed_x, int pushed_y, int x, int y){}
	virtual void onMUpNoDrag(int x, int y){}
	virtual void onMUpDrag(int pushed_x, int pushed_y, int x, int y){}

	virtual bool onRDown(int x, int y){return false;}
	virtual void onRDrag(int pushed_x, int pushed_y, int x, int y){}
	virtual void onRUpNoDrag(int x, int y){}
	virtual void onRUpDrag(int pushed_x, int pushed_y, int x, int y){}

	virtual void beginMode(){}
	virtual bool endMode(){ if(mState==READY) return true; return false;}
	virtual void onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData){}

	UI_mode()  : mState(READY), mFallback(NULL){}
}; 


class ModelingWin_Tool : public FlLayout, public UI_mode
{
protected:
	ModelingWin& _win;
public:
	ModelingWin_Tool (ModelingWin& win);
	virtual void beginTool(){}
	virtual bool endTool(){ return true; }
	virtual void onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData){}
};

class ModelingWin : public FlLayout, public FrameMoveObject, public FltkRenderer::Handler, public FltkMotionWindow ::EventReceiver
{
	void showEdges();

	void redrawMesh(bool bStructureChanged=true);

	int m_currMode;
	int m_currTool;

	Motion mMotion;
	SkinnedMeshLoader* mpSkeleton;
	PLDPrimMesh* mpSkin;
	TString mMaterialName;
	// skin으로 부터 vertex data얻어온 자료구조.
	OBJloader::Mesh* _mesh;
	scoped_ptr<OBJloader::EdgeConnectivity> mEdges;
	scoped_ptr<OBJloader::MeshConnectivity> mFaces;
	UnmergeInfo* _unmergeInfo;
	LineList* mLines;
	std::shared_ptr<Ogre::SceneNode> mNode;

public:
	Ogre::SceneNode* sceneNode()			{ return mNode.get();}
	OBJloader::Mesh* mesh()				{ return _mesh;}
	SkinnedMeshLoader* skeleton()			{ return mpSkeleton;}
	UnmergeInfo* unmergeInfo()				{ return _unmergeInfo;}
	void setUnmergeInfo(UnmergeInfo* p)		{ _unmergeInfo=p;}
	OBJloader::EdgeConnectivity* edges()	{ return mEdges.get();}
	OBJloader::MeshConnectivity* faces()	{ return mFaces.get();}
	int numEdges()	{ if(edges()) return edges()->numEdges(); return 0;}
	int numVertex() { if(mesh()) return mesh()->numVertex(); return 0;}
	ModelingWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer);
	~ModelingWin (void);

	MotionPanel& m_motionPanel;
	FltkRenderer& mRenderer;

	enum { CHANGE_VIEW, SELECT_VERTEX, SELECT_EDGE, SELECT_TRIANGLE, VERTEX_CORRESPONDENCE, TOOL_SPECIFIC, NUM_MODE};

	std::vector<std::shared_ptr<UI_mode> > m_modes;

	enum { DIRECT_MANIPULATION, LAPLACIAN_EDITING, TETRA_MESH_GENERATOR, NUM_TOOL};

	std::vector<std::shared_ptr<ModelingWin_Tool> > m_tools;

	void notify(int msg);
	// layout callback
	virtual void onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData);

	// FrameMoveObject
	virtual int FrameMove(float fElapsedTime);
	
	// FltkRenderer::Handler
	virtual int	handleRendererEvent(int ev) ;

	// Fl_Window
	virtual void show()		{FlLayout::show();mRenderer.setHandler(this);}

};
#endif
