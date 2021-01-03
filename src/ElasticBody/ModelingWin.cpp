#include "stdafx.h"
#include "TetraMesh.h"
#include "BaseLib/utility/tfile.h"
#include <boost/smart_ptr.hpp>
#include "BaseLib/math/Operator.h"
#include "MainLib/OgreFltk/PointClouds.h"
#include "MainLib/OgreFltk/MovableText.h"
#include "MainLib/OgreFltk/FlChoice.h"
#include "ModelingWin.h"
#include "UI_mode.h"
#include "Tool.h"
#include "../../BaseLib/motion/version.h"
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>

class MotionPanel;
class FltkRenderer;

using namespace modelingWin;

ModelingWin_Tool ::ModelingWin_Tool (ModelingWin& win):FlLayout(0,0,win.w(), win.h()), _win(win){}



FlLayout* createModelingWin(int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer)
{
	return new ModelingWin(x,y,w,h,mp, renderer);
}

// FrameMoveObject
int ModelingWin::FrameMove(float fElapsedTime)
{
	
	return 1;
}

ModelingWin::ModelingWin(int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer)
: FlLayout(0,0,w,h),
m_motionPanel(mp),
mRenderer(renderer)
{
	mLines=NULL;
	create("Choice", "init");
	menu(0)->size(4);
	menu(0)->item(0, "initialize");
	menu(0)->item(1, "load iguana mesh");
	menu(0)->item(2, "load iguana mesh (high)");
	menu(0)->item(3, "load iguana mesh (bound)");

	//create("Button", "Start", "Start");
	//button(0)->shortcut(FL_ALT+'s');
	
	create("Choice", "Option");
	menu(0)->size(2);
	menu(0)->item(0, "Option", 0);
	menu(0)->item(1, "Show mesh", 0, Hash("Show mesh"), FL_MENU_TOGGLE);
	menu(0)->item(1).set();
	menu(0)->value(0);

	// init UI modes
	create("Choice", "edit mode", "edit mode", 1);
	menu(0)->size(NUM_MODE);
	menu(0)->item(CHANGE_VIEW, "change viewpoint", 'q');
	menu(0)->item(SELECT_VERTEX, "select vertex", 'w');
	menu(0)->item(SELECT_EDGE, "select edge", 'e');
	menu(0)->item(SELECT_TRIANGLE, "select triangle", 'r');
	menu(0)->item(VERTEX_CORRESPONDENCE, "set vertex correspondence", 't');
	menu(0)->item(TOOL_SPECIFIC, "tool-dependent mode",'y');
	
	m_currMode=CHANGE_VIEW;
	
	menu(0)->value(m_currMode);

	m_modes.resize(NUM_MODE);
	UI_mode* changeView=new ChangeView(*this);
	m_modes[CHANGE_VIEW].reset(changeView);
	m_modes[SELECT_VERTEX].reset(new SelectVertex(*this));	
	m_modes[SELECT_VERTEX]->setFallback(changeView);
	m_modes[SELECT_EDGE].reset(new SelectEdge(*this));
	m_modes[SELECT_EDGE]->setFallback(changeView);
	m_modes[SELECT_TRIANGLE].reset(new SelectTriangle(*this));
	m_modes[SELECT_TRIANGLE]->setFallback(changeView);
	m_modes[VERTEX_CORRESPONDENCE].reset(new VertexCorrespondence(*this));
	m_modes[VERTEX_CORRESPONDENCE]->setFallback(changeView);
	m_modes[TOOL_SPECIFIC].reset(new UI_mode_router());
	m_modes[TOOL_SPECIFIC]->setFallback(changeView);

	m_modes[m_currMode]->beginMode();



	// init Tools
	create("Choice", "tools", "tools", 1);
	menu(0)->size(NUM_TOOL);
	menu(0)->item(DIRECT_MANIPULATION, "direct manipulation", 'a');
	menu(0)->item(LAPLACIAN_EDITING, "laplacian editing", 's');
	menu(0)->item(TETRA_MESH_GENERATOR, "tetra-mesh generation", 'd');

	m_currTool=DIRECT_MANIPULATION;
	
	menu(0)->value(m_currTool);

	m_tools.resize(NUM_TOOL);
	m_tools[DIRECT_MANIPULATION].reset(new DirectManipulation(*this));
	m_tools[LAPLACIAN_EDITING].reset(new LaplacianEditing(*this));
	m_tools[TETRA_MESH_GENERATOR].reset(new TetraMeshGenerator(*this));

	m_tools[m_currTool]->beginTool();

	m_tools[m_currTool]->setFallback(m_modes[m_currMode].get());


	setWidgetPos(0);
	std::vector<FlLayout*> ll;
	for(int ii=0; ii<m_tools.size(); ii++)
		ll.push_back(m_tools[ii].get());
	embedLayouts(ll, "tools", "tool menu");
	
	m_motionPanel.motionWin()->connect(*this);
	updateLayout();
	renderer.ogreRenderer().addFrameMoveObject(this);

	mpSkin=NULL;
	mpSkeleton=NULL;
}

ModelingWin::~ModelingWin(void)
{
	delete mLines;
}

void ModelingWin::showEdges()
{
	if(mesh())
	{
		mLines->begin(mEdges->numEdges());
		for(int i=0, ni=mEdges->numEdges(); i<ni; i++)
		{
			mLines->line(i, mesh()->getVertex(mEdges->source(i)), mesh()->getVertex(mEdges->target(i)));
		}

		mLines->end();
	}
}

void ModelingWin::redrawMesh(bool bStructureChanged)
{
	if(bStructureChanged)
	{
		bool bVisible=true;
		if(mpSkin)
		{
			bVisible=mpSkin->GetVisible();
			RE::remove(mpSkin);
		}

		mesh()->calculateVertexNormal();
		mpSkin=RE::createMesh(mpSkeleton, mMaterialName);

		mpSkin->ApplyAnim(mMotion);

		RE::motionPanel().motionWin()->addSkin(mpSkin);

		mEdges.reset(new OBJloader::EdgeConnectivity(*mesh()));
		mFaces.reset(new OBJloader::MeshConnectivity(*mesh(), *mEdges));
	}
	else
	{
		mpSkin->_updateMesh();
	}

	showEdges();
}



void ModelingWin::onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData)
{
	if(w.mId=="edit mode")
	{
		if(w.menu()->value()!=m_currMode)
		{
			if(m_modes[m_currMode]->endMode() && m_tools[m_currTool]->endTool() )
			{
				m_currMode=w.menu()->value();
				m_modes[m_currMode]->beginMode();
				m_tools[m_currTool]->setFallback(m_modes[m_currMode].get());
			}
			else
			{
				w.menu()->value(m_currMode);
				w.menu()->redraw();
			}
		}
	}
	else if(w.mId=="tools")
	{
		if(w.menu()->value()!=m_currTool)
		{
			if(m_tools[m_currTool]->endTool() && m_modes[m_currMode]->endMode() )
			{
				m_currTool=w.menu()->value();
				m_tools[m_currTool]->beginTool();
				m_tools[m_currTool]->setFallback(m_modes[m_currMode].get());
				

				findLayoutGroup("tools")->showLayout(m_currTool);
				redraw();
			}
			else
			{
				w.menu()->value(m_currTool);
				w.menu()->redraw();
			}
		}
	}
	else if(w.mId=="Option")
	{
		if(userData==Hash("Show mesh"))
		{
			Fl_Choice* p=w.menu();
			if(mpSkin)
				mpSkin->SetVisible(p->menu()[p->value()].value());
		}
	}
	else if(w.mId=="init")
	{
		TString tid=w.menu()->text();
		if(!mNode)
		{
			mLines=new LineList();
			mLines->setMaterial("blue");
			mNode.reset(RE::createSceneNode("ModelingWinNode"), (void(*)(Ogre::SceneNode* node))RE::removeEntity);
			mNode->attachObject(mLines);
		}

		_unmergeInfo=NULL;
		mMotion.Init(RE::motionLoader("iguana.skl"));
		RE::motion::concatFromFile(mMotion, "trc/iguana_motion_set.mot");

		if(mpSkeleton) delete mpSkeleton;
		mpSkeleton=new SkinnedMeshLoader ("iguana_pskinned.mesh", true);
		mpSkeleton->mMesh.mesh.calculateVertexNormal();
		mpSkeleton->sortBones(mMotion.skeleton());
		_unmergeInfo=mpSkeleton->mMesh.mergeDuplicateVertices(true);
			//mpSkeleton->mMesh.unmergeDuplicateVertices(
		mpSkeleton->gotoBindPose();
		Posture bindpose;
		mpSkeleton->getPose(bindpose);
		mMotion.pose(0)=bindpose;
		savePose(bindpose, "../Resource/mesh/iguana_bind.pose");
		mMaterialName="lambert6";
		mpSkeleton->mMesh.mesh.saveMesh("test.tri");
		if(tid=="load iguana mesh")
		{
			delete mpSkeleton;
			delete _unmergeInfo;
			_unmergeInfo=NULL;

			mpSkeleton=new SkinnedMeshLoader("iguana_physics.mesh");
			mpSkeleton->mMesh.mergeDuplicateVertices();
			mpSkeleton->sortBones(mMotion.skeleton());
			mpSkeleton->setPose(bindpose);
			
			mpSkeleton->setCurPoseAsBindPose();
			mMaterialName="white";

			//OBJloader::Mesh temp;
			// 내가 인덱싱만 바꾼 메쉬를 읽는다. -  volume mesh 와 호환되도록 바꾸었다.
			mpSkeleton->mMesh.mesh.loadMesh("../Resource/mesh/iguana.tri");
			//mpSkeleton->mMesh.mesh.calculateVertexNormal();
			 //.m_arrayFace=temp.m_arrayFace;
		}
		else if(tid=="load iguana mesh (bound)")
		{
			delete mpSkeleton;
			delete _unmergeInfo;
			_unmergeInfo=NULL;

			mpSkeleton=new SkinnedMeshLoader("iguana_physics.mesh");
			mpSkeleton->mMesh.mergeDuplicateVertices();
			mpSkeleton->mMesh.mesh.calculateVertexNormal();
			mpSkeleton->mMesh.reorderVertices();	// to match iguana_bound.obj
			mpSkeleton->sortBones(mMotion.skeleton());
			mpSkeleton->setPose(bindpose);
			mpSkeleton->setCurPoseAsBindPose();
			mMaterialName="white";

			OBJloader::Mesh temp;
			// 내가 인덱싱만 바꾼 메쉬를 읽는다. -  volume mesh 와 호환되도록 바꾸었다.
			temp.loadObj("../Resource/mesh/iguana_bound.obj");
			mpSkeleton->mMesh.option.buildEdgeList=true;
			mpSkeleton->mMesh.mesh=temp;
			//mpSkeleton->mMesh.mesh.m_arrayFace=temp.m_arrayFace;

		}

			_mesh=&mpSkeleton->mMesh.mesh;	// bind pose editing.

		redrawMesh();
	}
}

void testIguanaMesh()
{
	OBJloader::Mesh temp;
	temp.loadMesh("test.tri");
	//temp.loadMesh("../Resource/mesh/iguana.tri");
	temp.calculateVertexNormal();

//	RE::ogreRootSceneNode()->attachObject(OBJloader::createMeshEntity(temp, RE::generateUniqueName()));
	OBJloader::EdgeConnectivity e(temp);
}

// FltkRenderer::Handler
int	ModelingWin::handleRendererEvent(int ev) 
{
	return m_tools[m_currTool]->handleRendererEvent(ev);
}

void ModelingWin::notify(int msg)
{
	switch(msg)
	{
	case UI_mode::MESH_VERTEX_MODIFIED:
		redrawMesh(false);
		break;
	case UI_mode::MESH_CHANGED:
		redrawMesh();
		break;
	}

	for(int i=0; i<NUM_MODE;i++)
		m_modes[i]->notify(msg);

	for(int i=0; i<NUM_TOOL; i++)
		m_tools[i]->notify(msg);
}
