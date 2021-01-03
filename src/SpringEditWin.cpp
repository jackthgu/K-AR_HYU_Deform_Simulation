#include "stdafx.h"
#include "MainLib/OgreFltk/Mesh.h"
#include "BaseLib/utility/tfile.h"
#include "BaseLib/math/Operator_NR.h"
#include <boost/smart_ptr.hpp>
//#include "../MainLib/Ogre/PLDPrimCustumSkin.h"
#include "MainLib/OgreFltk/MotionPanel.h"
#include "MainLib/OgreFltk/FltkAddon.h"
#include "MainLib/OgreFltk/MotionManager.h"
#include "MainLib/OgreFltk/objectList.h"
#include "MainLib/OgreFltk/pldprimskin.h"
#include "MainLib/OgreFltk/FlLayout.h"
#include "MainLib/OgreFltk/framemoveobject.h"
#include "MainLib/OgreFltk/FltkMotionWindow.h"
#include "MainLib/OgreFltk/FltkRenderer.h"
#include "MainLib/OgreFltk/Line3D.h"
#include "MainLib/OgreFltk/PointClouds.h"
#include <Fl/Fl_Double_Window.H>
#include "MainLib/OgreFltk/FlChoice.h"
#include "MainLib/OgreFltk/Fl_Hor_Slider2.h"
#include "MainLib/OgreFltk/FltkAddon.h"
#include "MainLib/OgreFltk/FltkMotionWindow.h"
//#include "ClassificationLib/motion/TwoPosture.h"
#include "MainLib/OgreFltk/FlLayout.h"
#include "MainLib/OgreFltk/FltkScrollPanel.h"
#include "BaseLib/utility/operatorString.h"
#include "BaseLib/math/Operator.h"
//#include "../MainLib/ogrefltk/Motion.h"
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>
#include <OgreMovableObject.h>
#include <OgreRenderable.h>
#include "MainLib/OgreFltk/MovableText.h"
#include "MainLib/OgreFltk/OgreMotionLoader.h"
class MotionPanel;
class FltkRenderer;


class SpringEditWin : public FlLayout, public FrameMoveObject, public FltkRenderer::Handler, public FltkMotionWindow ::EventReceiver
{
public:
	SpringEditWin (int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer);
	~SpringEditWin (void);

	Motion* motion;
	MotionPanel& m_motionPanel;
	FltkRenderer& mRenderer;
	ObjectList mObjectList;

	// layout callback
	virtual void onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData);

	// FrameMoveObject
	virtual int FrameMove(float fElapsedTime);
	
	// FltkRenderer::Handler
	virtual int	handleRendererEvent(int ev) ;

	// Fl_Window
	virtual void show()		{FlLayout::show();mRenderer.setHandler(this);}

	// FltkMotionWindow ::EventReceiver
	virtual void OnFrameChanged(FltkMotionWindow*, int currFrame);
	
	SkinnedMeshLoader* meshAnimation;

	OBJloader::Mesh mMesh;
	boost::scoped_ptr<OBJloader::EdgeConnectivity> mEdges;
	boost::scoped_ptr<OBJloader::MeshConnectivity> mFaces;
#ifdef USE_NO_SHARED_VERTEX
	OBJloader::Mesh mMeshNoSharedVertex;
#endif
	
	// vertexdata?? ?׸??? ?? ?ڷᱸ.
	OBJloader::MeshToEntity* mpMeshToEntity;
	Ogre::Entity* mpEntity;

	intmatrixn mLinks;
	LineList* mLines;
	boost::shared_ptr<Ogre::SceneNode> mNode;

	boolN mSelectedVertices;
	boolN mSelectedEdges;
	void showEdges();
	void clearSelection();

	void drawSelection();
};

void SpringEditWin::drawSelection()
{
	mObjectList.clear();
	for(int i=0; i<mMesh.numVertex(); i++)
	{
		if(mSelectedVertices[i])
		{
			RE::moveEntity(mObjectList.registerEntity(RE::generateUniqueName(), "sphere1010.mesh"), quater(1,0,0,0), mMesh.getVertex(i));
		}					
	}
}
void SpringEditWin::clearSelection()
{

	if(mSelectedVertices.size())
	mSelectedVertices.clearAll();
	if(mSelectedEdges.size())
	mSelectedEdges.clearAll();
}

FlLayout* createSpringEditWin(int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer)
{
	return new SpringEditWin(x,y,w,h,mp, renderer);
}

// FrameMoveObject
int SpringEditWin::FrameMove(float fElapsedTime)
{
	
	return 1;
}

void SpringEditWin::OnFrameChanged(FltkMotionWindow*, int currFrame)
{
	if(mpEntity)
	{
		meshAnimation->setPose(motion->pose(currFrame));
		meshAnimation->retrieveAnimatedMesh(mMesh);
#ifdef USE_NO_SHARED_VERTEX
		mMeshNoSharedVertex.eliminateSharedVertex(mMesh);
		mMeshNoSharedVertex.calculateVertexNormal();
#else
		mMesh.calculateVertexNormal();
#endif
		mpMeshToEntity->updatePositionsAndNormals();
		showEdges();
	}
}

SpringEditWin::SpringEditWin(int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer)
: FlLayout(0,0,w,h),
m_motionPanel(mp),
mRenderer(renderer)
{
	mLines=NULL;
	mpEntity=NULL;
	create("Button", "Start", "Start");
	button(0)->shortcut(FL_ALT+'s');
	create("Choice", "selection mode", "selection mode", 1);

	menu(0)->size(2);
	menu(0)->item(0, "vertex", 'v');
	menu(0)->item(1, "edge", 'e');
	menu(0)->value(1);
	
	create("Button", "drawNormals", "drawNormals",0);
	create("Button", "drawFaceIndex", "drawFaceIndex");
	create("Button", "drawVertexIndex", "drawVertexIndex");
	create("Button", "mesh laplacian", "mesh laplacian");
	create("Button", "save selection", "save selection",0);
	create("Button", "load selection", "load selection",0);

	m_motionPanel.motionWin()->connect(*this);
	updateLayout();
	renderer.ogreRenderer().addFrameMoveObject(this);

}

SpringEditWin::~SpringEditWin(void)
{
	delete mLines;
}

void SpringEditWin::showEdges()
{
	mLines->begin(mLinks.rows());
	for(int i=0, ni=mLinks.rows(); i<ni; i++)
	{
		mLines->line(i, mMesh.getVertex(mLinks[i][0]), mMesh.getVertex(mLinks[i][1]));
	}

	mLines->end();
}

void SpringEditWin::onCallback(FlLayout::Widget const& w, Fl_Widget * pWidget, int userData)
{
	if(w.mId=="mesh laplacian")
	{
		int num_vertex=mMesh.numVertex();

		intvectorn degrees(num_vertex);

		for(int i=0; i<num_vertex; i++)
			degrees[i]=mEdges->numAdjEdges(i);

		matrixn L(num_vertex, num_vertex);
		L.setAllValue(0.0);


		const bool bUseKarniLaplacian=true;

		if(bUseKarniLaplacian)
		{
			L.setDiagonal(1.0);
			for(int i=0; i<num_vertex; i++)
				for(int j=i+1; j<num_vertex; j++)
				{
					if(mEdges->isConnected(i,j))
					{
						L[i][j]=-1.0/(double)(degrees[i]);
						L[j][i]=-1.0/(double)(degrees[j]);
					}
				}
		}
		else
		{
			matrixn w(num_vertex, num_vertex);
			w.setAllValue(0);
			for(int i=0; i<num_vertex; i++)
				for(int j=0; j<num_vertex; j++)
				{
					if(mEdges->isConnected(i,j))
				//		w[i][j]=1.0;
				w[i][j]=1.0/mMesh.getVertex(i).distance(mMesh.getVertex(j));
				}



			for(int i=0; i<num_vertex; i++)
				for(int j=0; j<num_vertex; j++)
				{
					if(i==j)
						L[i][j]=w.row(i).sum();
					else
						L[i][j]=-1.0*w[i][j];
				}
		}

		vectorn eigenValues;
		matrixn eigenVectors;
		m::eigenDecomposition(L, eigenValues, eigenVectors, 0);

		OBJloader::Mesh mesh;
		mesh=mMesh;

		RE::output("eigen values", eigenValues.output().ptr());
		for(int mode=0; mode<eigenValues.size(); mode++)
		//for(int mode=0; mode<5; mode++)
		{
			vectornView ev=eigenVectors.column(eigenValues.size()-mode-1);

			ev/=ev.maximum();
			//printf("ev%d %s\n", mode, ev.output().ptr());
			for(int i=0; i<num_vertex; i++)
			{
				float c=ev[i];
				mesh.getColor(i)=vector4(c,c,c,1.0);
			}
			
			TString nodeName;
			nodeName.format("node_mesh_laplacian%d", mode);
			Ogre::SceneNode* pnode=RE::createSceneNode(nodeName);
			OBJloader::MeshToEntity::Option o;
			o.useColor=true;
			pnode->attachObject(OBJloader::createMeshEntity(mesh, nodeName+"entity", "use_vertexcolor_only", o));
			pnode->translate(0,mode*40+40,0);
		}
	}
	else if(w.mId=="save selection")
	{
		TString fn=FlChooseFile("input file name to save", "../Resource/scripts/ui/", "*.selection", true);
		if(fn.length()==0)
			return;
		BinaryFile bf(true, fn);
		bf.pack(mSelectedVertices);
		bf.pack(mSelectedEdges);
		bf.close();
	}
	else if(w.mId=="load selection")
	{
		TString fn=FlChooseFile("input file name to save", "../Resource/scripts/ui/", "*.selection");
		if(fn.length()==0)
			return;
		BinaryFile bf(false, fn);
		bf.unpack(mSelectedVertices);
		bf.unpack(mSelectedEdges);
		bf.close();
		drawSelection();
	}
	else if(w.mId=="drawNormals")
	{
#ifdef USE_NO_SHARED_VERTEX
		OBJloader::Mesh& mesh=mMeshNoSharedVertex;
#else
		OBJloader::Mesh& mesh=mMesh;
#endif
		ASSERT(mesh.numVertex()==mesh.numNormal());
		mLines->begin(mesh.numVertex());

		for(int i=0, ni=mesh.numVertex(); i<ni; i++)
		{
			mLines->line(i, mesh.getVertex(i), mesh.getVertex(i)+mesh.getNormal(i));
		}
		mLines->end();
	}
	else if(w.mId=="drawFaceIndex")
	{

		for(int i=0; i<mMesh.numFace(); i++)
		{
			vector3 pos=mMesh.calcFaceCenter(i)+mMesh.calcFaceNormal(i)*0.8;

			Ogre::SceneNode* child=mNode->createChildSceneNode(ToOgre(pos));

			Ogre::MovableText* text=new Ogre::MovableText(RE::generateUniqueName().ptr(), sz1::format("f%d", i).ptr(), "BlueHighway", 1, Ogre::ColourValue::Black);
			//text->showOnTop(true);
			child->attachObject(text);
		}
	}
	else if(w.mId=="drawVertexIndex")
	{

		for(int i=0; i<mMesh.numVertex(); i++)
		{
			vector3 pos=mMesh.getVertex(i)+mMesh.getNormal(i)*0.8;

			Ogre::SceneNode* child=mNode->createChildSceneNode(ToOgre(pos));

			Ogre::MovableText* text=new Ogre::MovableText(RE::generateUniqueName().ptr(), sz1::format("v%d", i).ptr(), "BlueHighway", 1, Ogre::ColourValue::Black);
			//text->showOnTop(true);
			child->attachObject(text);
		}
	}
	else if(w.mId=="Start")
	{
		if(!mNode)
		{
			mLines=new LineList();
			mLines->setMaterial("blue");
			mNode.reset(RE::createSceneNode("SpringEditWinNode"), (void(*)(Ogre::SceneNode* node))RE::removeEntity);
			mNode->attachObject(mLines);
		}

		motion=RE::motionManager().createMotion("iguana");
		motion->Init(RE::motionLoader("trc/iguana_from248.bvh"));

		// get the first frame mesh.

		meshAnimation=new SkinnedMeshLoader("iguana_physics.mesh");
		meshAnimation->mMesh.mergeDuplicateVertices();
		meshAnimation->mMesh.mesh.calculateVertexNormal();
		meshAnimation->mMesh.reorderVertices();	// to match iguana_bound.obj

		meshAnimation->sortBones(motion->skeleton());


		//meshAnimation.create(*motion, "iguana_physics.mesh", mMesh, 30);
		mMesh.copyFrom(meshAnimation->mMesh.mesh);
		meshAnimation->setPose(motion->pose(30));
		meshAnimation->retrieveAnimatedMesh(mMesh);
	
		m_motionPanel.motionWin()->addSkin(RE::createEmptyAnimationObject(motion->numFrames()));

		OBJloader::MeshToEntity::Option option;
		option.buildEdgeList=true;
		option.useTexCoord=false;

#ifdef USE_NO_SHARED_VERTEX
		mMeshNoSharedVertex.eliminateSharedVertex(mMesh);
		mMeshNoSharedVertex.calculateVertexNormal();
		mpMeshToEntity=new OBJloader::MeshToEntity(mMeshNoSharedVertex, "iguanaMesh", option);
#else
		mMesh.calculateVertexNormal();
		mpMeshToEntity=new OBJloader::MeshToEntity(mMesh, "iguanaMesh", option);
#endif
		//mpEntity=mpMeshToEntity->createEntity("iguanaMeshEntity", "use_vertexcolor");
		mpEntity=mpMeshToEntity->createEntity("iguanaMeshEntity", "lightgrey_transparent");
		mNode->attachObject(mpEntity);

		mEdges.reset(new OBJloader::EdgeConnectivity(mMesh));
		mFaces.reset(new OBJloader::MeshConnectivity(mMesh, *mEdges));
		
		OBJloader::EdgeConnectivity& edges=*mEdges;
		int numEdges=edges.mMeshGraph.numEdges();
		mLinks.resize(numEdges, 2);
		
		for(int i=0; i<numEdges; i++)
		{
			mLinks[i][0]=edges.mMeshGraph.findEdge(i).v1().index();
			mLinks[i][1]=edges.mMeshGraph.findEdge(i).v2().index();
		}

		showEdges();
	}
}

inline intersectionTest::LineSegment Ray2LineSegment(Ray const& r)
{
	return intersectionTest::LineSegment (r.origin(), r.getPoint(FLT_MAX));
}

// FltkRenderer::Handler
#include <FL/Fl.H>
int	SpringEditWin::handleRendererEvent(int ev) 
{
	static int pushed_x;
	static int pushed_y;
	static SelectionRectangle* mRect=NULL;

	if(!visible()) return 0;
	switch(ev)
	{
	case FL_PUSH:
		pushed_x=Fl::event_x();
		pushed_y=Fl::event_y();
		if(Fl::event_state()&FL_SHIFT)
		{
			// marquee selection
			//mRenderer.viewLock();
			Ogre::SceneNode* pNode=RE::createSceneNode("MarqueeNode");
			mRect = new SelectionRectangle("Selection SelectionRectangle");
			pNode->attachObject(mRect);

			mRect->setCorners(TRect(pushed_x, pushed_y, pushed_x, pushed_y));
			RE::output("marqueeEvent", "push");
			return 1;
		}

			break;
//	case FL_MOVE: 
	case FL_DRAG: 
		{
			if(mRect)
			{
				mRect->setCorners(TRect(pushed_x,pushed_y,Fl::event_x(),Fl::event_y()));
				return 1;
			}

		}
		break;
	case FL_RELEASE:
		{
			if(mRect)
			{
				// marquee selection
				mRect->setCorners(TRect(pushed_x,pushed_y,Fl::event_x(),Fl::event_y()));
				std::vector<Plane> vol;
				mRect->constructSelectionVolumn(mRenderer, vol);
				clearSelection();
				mSelectedVertices.resize(mMesh.numVertex());
				mSelectedVertices.clearAll();
				for(int i=0; i<mMesh.numVertex(); i++)
				{
					Sphere s(mMesh.getVertex(i), 0.001);
					if(s.isInside(vol))
						mSelectedVertices.setAt(i);
				}

				drawSelection();
				RE::removeEntity("MarqueeNode");
				mRect=NULL;
				return 1;
			}
			else
			{
				static vector3  pushed_cursorPos;
				int x=Fl::event_x();
				int y=Fl::event_y();

				if(x!=pushed_x || y!=pushed_y)
					return 0;
				int button=Fl::event_button();
				bool bButton1=Fl::event_state()&FL_BUTTON1;
				bool bButton2=Fl::event_state()&FL_BUTTON2;
				bool bButton3=Fl::event_state()&FL_BUTTON3;

				switch(ev)
				{
				case FL_PUSH: RE::output("mouse event", "push %d %d %d", x,y, button&FL_BUTTON1);break;
				case FL_MOVE: RE::output("mouse event", "move %d %d %d", x,y, button&FL_BUTTON1);break;
				case FL_DRAG: RE::output("mouse event", "drag %d %d %d", x,y, button&FL_BUTTON1);break;
				case FL_RELEASE: RE::output("mouse event", "release %d %d %d", x,y, button&FL_BUTTON1);break;
				}
				RE::output("button", "%d", button);
				if((ev==FL_PUSH || ev==FL_RELEASE) && button != FL_LEFT_MOUSE) break;
				Ray ray;
				mRenderer.screenToWorldRay(x,y,ray);

				TString selectionMode=findMenu("selection mode")->text();

				if(selectionMode=="edge")
				{
					intersectionTest::LineSegment sx=Ray2LineSegment(ray);

					m_real minDist=FLT_MAX;
					int argMin=-1;
					for(int i=0; i<mLinks.rows(); i++)
					{
						intersectionTest::LineSegment l(mMesh.getVertex(mLinks[i][0]), mMesh.getVertex(mLinks[i][1]));
						m_real dist=l.minDist(sx);
						if(dist<minDist)
						{
							minDist=dist;
							argMin=i;
						}
					}

					clearSelection();
					BillboardLineList * line=new BillboardLineList ("edge", 1,1);
					line->setMaterialName("redToOrange");
					line->line(0, mMesh.getVertex(mLinks[argMin][0]), mMesh.getVertex(mLinks[argMin][1]));
					mObjectList.registerObject("link", line);
					RE::moveEntity(mObjectList.registerEntity("sellink0","sphere1010.mesh"), quater(1,0,0,0), mMesh.getVertex(mLinks[argMin][0]));
					RE::moveEntity(mObjectList.registerEntity("sellink1","sphere1010.mesh"), quater(1,0,0,0), mMesh.getVertex(mLinks[argMin][1]));
				}
				else
				{
					clearSelection();
					for(int i=0; i<mMesh.numVertex(); i++)
					{
						Sphere s(mMesh.getVertex(i), 3);
						if(ray.intersects(s).first)
						{

							RE::moveEntity(mObjectList.registerEntity(RE::generateUniqueName(), "sphere1010.mesh"), quater(1,0,0,0), mMesh.getVertex(i));
						}					
					}
				}
			}
		}
		break;
	}
	return 0;
}
