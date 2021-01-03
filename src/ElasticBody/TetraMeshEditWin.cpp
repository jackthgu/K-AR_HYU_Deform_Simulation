#include "stdafx.h"
#include "MainLib/OgreFltk/MotionPanel.h"
#include "MainLib/OgreFltk/Mesh.h"
#include "MainLib/OgreFltk/FlLayout.h"
#include "MainLib/OgreFltk/framemoveobject.h"
#include "MainLib/OgreFltk/FltkRenderer.h"
#include "MainLib/OgreFltk/FlChoice.h"
#include "MainLib/OgreFltk/RE.h"
#include "MainLib/OgreFltk/pldprimskin.h"
#include "DeformableBody.h"
#include "BaseLib/utility/scoped_ptr.h"
#include <boost/smart_ptr.hpp>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>

using namespace TetraMeshLoader;
class TetraMeshEditWin: public FlLayout, public FrameMoveObject
{
	DeformableBody mMesh;
	scoped_ptr<TetraMeshToEntity> mMeshViewer;
	scoped_ptr<MeshLoader::MeshToEntity> mEmbeddedMeshViewer;
	
	boost::shared_ptr<Ogre::SceneNode> mNode;
public:

	
	TetraMeshEditWin(int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer);
	~TetraMeshEditWin(void);


	MotionPanel& m_motionPanel;
	FltkRenderer& mRenderer;

	// layout callback
	virtual void onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData);

	// FrameMoveObject
	virtual int FrameMove(float fElapsedTime);
};


// FrameMoveObject
int TetraMeshEditWin::FrameMove(float fElapsedTime)
{
	return 1;
}


TetraMeshEditWin::TetraMeshEditWin(int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer)
: FlLayout(0,0,w,h),
m_motionPanel(mp),
mRenderer(renderer)
{
	create("Button", "Load a mesh", "Load a mesh");
	create("Button", "Load a embedded mesh", "Load a embedded mesh");


	create("Choice", "Option");
	menu(0)->size(2);
	menu(0)->item(0, "Option", 0);
	menu(0)->item(1, "Show mesh", 0, Hash("Show mesh"), FL_MENU_TOGGLE);
	menu(0)->item(1).set();
	menu(0)->value(0);

	updateLayout();
	renderer.ogreRenderer().addFrameMoveObject(this);
}

TetraMeshEditWin::~TetraMeshEditWin(void)
{
}


void TetraMeshEditWin::onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData)
{
	if(w.mId=="Load a embedded mesh")
	{
		if(mMesh.ebody.getNumNode()==0)
			Msg::msgBox("Load a Tetra mesh first");

		else
		{
			TString mesh;
			mesh=FlChooseFile("Choose mesh", "../Resource/mesh/","*.tri");
			
			if(mesh.length())
			{
				ASSERT(mNode.get());
				mMesh.loadSurfaceMesh(mesh);
				matrix4 m;
				m.setScaling(100,100,100);
				mMesh.surf_embedded._mesh.transform(m);
				mEmbeddedMeshViewer.reset(new MeshLoader::MeshToEntity(mMesh.surf_embedded._mesh, "__embedMesh0"));
				mEmbeddedMeshViewer->createEntity("__embedMeshEntity0");
				mNode->attachObject(mEmbeddedMeshViewer->getLastCreatedEntity());
			}
		}
	}
	else if(w.mId=="Load a mesh")
	{
		TString mesh;
		mesh=FlChooseFile("Choose mesh", "../mesh/","*.tet");
		
		if(mesh.length())
		{
			if(!mNode)
			{
				mNode.reset(RE::createSceneNode(RE::generateUniqueName()), (void(*)(Ogre::SceneNode* node))RE::removeEntity);			
			}

			mMesh.ebody.loadMesh(mesh);
			matrix4 m;
			m.setScaling(100,100,100);
			mMesh.ebody.transform(m);
			mMeshViewer.reset(new TetraMeshToEntity(mMesh.ebody, "__tetraMesh0"));
			mMeshViewer->createEntity("__tetraMeshEntity0","use_vertexcolor");
//			mMeshViewer->createEntity("__tetraMeshEntity0","use_vertexcolor_transparent");
			mNode->attachObject(mMeshViewer->getLastCreatedEntity());
		}
	}
	else if(w.mId=="Option")
	{
		if(userData==Hash("Show mesh"))
		{
			Fl_Choice* p=w.menu();
			if(p->menu()[p->value()].value())
			{
				if(mMeshViewer)
					mMeshViewer->getLastCreatedEntity()->setVisible(true);
			}
			else
			{				
				if(mMeshViewer)
					mMeshViewer->getLastCreatedEntity()->setVisible(false);
			}
		}
	}
}

FlLayout* createTetraMeshEditWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer)
{
	return new TetraMeshEditWin(x,y,w,h,mp, renderer);
}

