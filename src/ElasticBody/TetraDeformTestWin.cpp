#include "stdafx.h"

#include <OgreSubMesh.h>

#include <memory>
#include "MainLib/OgreFltk/MotionPanel.h"
#include "MainLib/OgreFltk/Mesh.h"
#include "MainLib/OgreFltk/FlLayout.h"
#include "MainLib/OgreFltk/framemoveobject.h"
#include "MainLib/OgreFltk/FltkRenderer.h"
#include "MainLib/OgreFltk/FlChoice.h"
#include "MainLib/OgreFltk/RE.h"
#include "MainLib/OgreFltk/pldprimskin.h"
#include "DeformableBody.h"
#include "BaseLib/baselib.h"
#include "BaseLib/math/Operator_NR.h"
#include "BaseLib/utility/scoped_ptr.h"
#include "femworld/IntegratedWorld.h"

#include <boost/smart_ptr.hpp>
#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <OgreEntity.h>

#include "cma/CMAwrap.h"
#include <OgrePrerequisites.h>
#include "MainLib/OgreFltk/objectList.h"
#include "MainLib/OgreFltk/FltkRenderer.h"
#include "MainLib/OgreFltk/Line3D.h"
#include "MainLib/OgreFltk/PointClouds.h"
#include <Fl/Fl_Double_Window.H>
#include "MainLib/OgreFltk/Fl_Hor_Slider2.h"
#include "MainLib/OgreFltk/FltkAddon.h"
#include "MainLib/OgreFltk/FltkMotionWindow.h"
#include "ClassificationLib/motion/TwoPosture.h"
#include "MainLib/OgreFltk/FlLayout.h"
#include "MainLib/OgreFltk/FltkScrollPanel.h"
#include "BaseLib/utility/operatorString.h"
#include "BaseLib/math/Operator.h"
//#include "../MainLib/ogrefltk/Motion.h"
#include <OgreMovableObject.h>
#include <OgreRenderable.h>
#include "MainLib/OgreFltk/MovableText.h"
#include "MainLib/OgreFltk/OgreMotionLoader.h"
//#include "saveZbuffer.h"
#include "MeanValueWeights.h"
#include "CompareHelper.h"

using namespace TetraMeshLoader;

double fitfun(vector3N const v3x, vector3N const v3y) { /* function "cigtab" */
    int i;
    double sum = 0;
//  for(i = 2; i < N; ++i)  
//   sum += x[i]*x[i]; 
    return sum;
}


MeshDeformer* createHarmonicMeshDeformer();
MeshDeformer* createMeanValueMeshDeformer();


// defined in TestWin.cpp
void getVertices(OBJloader::Mesh const& mesh, vector3N & o);
void setVertices(OBJloader::Mesh & mesh, vector3N const& o);

class TetraDeformTestWin : public FlLayout, public FrameMoveObject, public FltkRenderer::Handler {

    TetraMesh mMesh;
    IntegratedWorld mSoftWorld;


    boost::shared_ptr<Ogre::SceneNode> mNode;

    MeshDeformer *mMeshDeformer;

    //MeshDeformer *mTargetMeshInControlVolume;
	//matrixn targetDeformToControlDeform;
	LinearDeformer *mControlPointsInFeatureMesh;
	vector3N m_vertices; // for the attachment CSTs
	vector3N m_vertices_orig; // for the attachment CSTs

    bool isLoadTet;
    BillboardLineList *line;
    BillboardLineList *ano_line;

    float min_selected_mesh;
    float max_selected_mesh;

    cmaes_t evo;
    ObjectList mObjectList;
    intmatrixn mLinks;


    CMAwrap *p_cma;

    vectorn vecn_start = vectorn(5, 0, 0, 0, 0, 0);
    vectorn vecn_dest = vectorn(5, 10, 12, 7, 4, 5);

    double *const *pop;

    //OBJloader::Mesh mObjTetra;
    OBJloader::MeshToEntity *mpTetraObjViewer;
    OBJloader::MeshToEntity *mpSurfaceObjViewer;
    OBJloader::MeshToEntity *mpTargetObjViewer;
    Ogre::Entity *mpTetraObjEntity;
    Ogre::Entity *mpSurfaceObjEntity;
    Ogre::Entity *mpTargetObjEntity;

public:

	int getNumSurfaceVertex() const {
		TString surfaceVertexFile = _meshName + ".sv";

		CTextFile file;
		if (!file.OpenReadFile(surfaceVertexFile)) {
			printf("Error! Faild to open %s\n", surfaceVertexFile.ptr());
			return NULL;
		}

		int result = atoi(file.GetToken());

		file.CloseFile();

		return result;
	} // hard-coded

    TetraDeformTestWin(int x, int y, int w, int h, MotionPanel &mp, FltkRenderer &renderer);

    ~TetraDeformTestWin(void);

	void tetToObj();
    // FltkRenderer::Handler
    virtual int handleRendererEvent(int ev);

	void loadTetMesh(const char* mesh)
	{
        bool bInit = true;

		if (!mNode) {
			//mNode.reset(RE::createSceneNode(RE::generateUniqueName()), (void(*)(Ogre::SceneNode* node))RE::removeEntity);
		}
		mMesh.loadMesh(mesh);
		/*
		matrix4 s_vec = matrix4();
		s_vec.setScaling(20, 20, 20);
		mMesh.transform(s_vec);
		*/

		mSoftWorld.addBody("testmesh", mMesh, 30);

		setLinearConstraint();


		redraw();
		isLoadTet = true;

		Ogre::SceneNode* pBg=RE::renderer().viewport().mScene->getSceneNode("BackgroundNode");
		pBg->setVisible(false);
	}

    MotionPanel &m_motionPanel;
    FltkRenderer &mRenderer;

    // layout callback
    virtual void onCallback(FlLayout::Widget const &w, Fl_Widget *pWidget, int userData);

    // FrameMoveObject
    virtual int FrameMove(float fElapsedTime);

    // Fl_Window
    virtual void show() {
        FlLayout::show();
        mRenderer.setHandler(this);
    }

    void redraw();

    void initializeCMA();


    void setLinearConstraint();

    void clearSelection();

    boolN mSelectedVertices;
    boolN mSelectedEdges;
    boolN mSelectedTetra;

    void drawSelection();

    OBJloader::Mesh *tet_to_obj;
    OBJloader::Mesh *mTargetMesh;
    OBJloader::Mesh *mTargetMesh_originalShape;
    OBJloader::Mesh mGrootHighMesh;

private:
	// For apply motion vector to Obj Mesh
	OBJloader::Mesh* UpdateMesh(OBJloader::Mesh* source);
	void ChangeSimilarity(double value);
	void ChangeError(double pcm, double rms);
	Fl_Widget* _maxWidget;
	Fl_Widget* _avgWidget;
	CompareHelper _compareHelper;

	TString _meshName; // File path + name without expansion
};


// FrameMoveObject
int TetraDeformTestWin::FrameMove(float fElapsedTime) {
	mObjectList.FrameMove(fElapsedTime);
    if (isLoadTet) {
        mSoftWorld.TimeStepping();
        redraw();
    }
    return 1;
}


TetraDeformTestWin::TetraDeformTestWin(int x, int y, int w, int h, MotionPanel &mp, FltkRenderer &renderer)
        : FlLayout(0, 0, w, h),
          m_motionPanel(mp),
          mRenderer(renderer),
		  mTargetMesh(NULL),
		  tet_to_obj(NULL),
          mSoftWorld(1.0 / 60.0, 0.95) {
    create("Button", "Load a mesh", "Load a mesh");
    //create("Button", "Load a embedded mesh", "Load a embedded mesh");
    create("Button", "Load a deformed mesh", "Load a deformed mesh");
    create("Value_Slider", "activation", "activation");
    slider(0)->range(0,0.03);
    slider(0)->value(0);

    create("Value_Slider", "min_select", "min_select");
    create("Value_Slider", "max_select", "max_select");

    create("Button", "save_current", "save_current");
    create("Button", "start_optimize", "start_optimize");

    //slider(0)->range(0,1);
    //slider(0)->value(0);

    create("Choice", "Option");
    menu(0)->size(2);
    menu(0)->item(0, "Option", 0);
    menu(0)->item(1, "Show mesh", 0, Hash("Show mesh"), FL_MENU_TOGGLE);
    menu(0)->item(1).set();
    menu(0)->value(0);

	_maxWidget = create("Box", "max", "Max Error : 0.0000");
	_avgWidget = create("Box", "avg", "Avg Error : 0.0000");
    updateLayout();
    renderer.ogreRenderer().addFrameMoveObject(this);
    isLoadTet = false;
    line = NULL;
    ano_line = NULL;

    p_cma = new CMAwrap(vecn_start, vecn_dest, 10, 2);
}

TetraDeformTestWin::~TetraDeformTestWin(void) {

	if(line)
	{
        RE::ogreRootSceneNode()->detachObject(line);
        RE::ogreRootSceneNode()->detachObject(ano_line);
        delete line;
	}
}

OBJloader::Mesh *tetraToObj(TetraMesh &srctet, const char *facefile, int surface_vtx_num ) {
    int n, m;
    vector3 x_tmp;
    CTextFile file;
    if (!file.OpenReadFile(facefile)) {
        printf("Error! Faild to open %s\n", facefile);
        return NULL;
    }

    OBJloader::Mesh *tetSurf_obj = new OBJloader::Mesh();

	//tetSurf_obj->resizeBuffer(OBJloader::Buffer::VERTEX, srctet.getNumNode());
	tetSurf_obj->resizeBuffer(OBJloader::Buffer::VERTEX, surface_vtx_num);
    for (int i = 0; i < surface_vtx_num; i++) {
        tetSurf_obj->getVertex(i) = srctet.nodePos(i);
    }

    TString token;
    TString currMode = "v";
    intvectorn typeCode;

    vector3N pos;

    int num_face = atoi(file.GetToken());
    int idon = atoi(file.GetToken());

    while (1) {
        char *t = file.GetToken();
        if (t == NULL)
            break;
        token = t;
        OBJloader::Face f;
//		int index=atoi(file.GetToken());
        int i = atoi(file.GetToken());
        int j = atoi(file.GetToken());
        int k = atoi(file.GetToken());
        int temp = atoi(file.GetToken());
        f.setIndex(i - 1, j - 1, k - 1, OBJloader::Buffer::VERTEX);

//		TString temp=file.GetToken();
//		if (temp.length()>0 && temp[0]>='0' && temp[0]<='9')
//		{
//			tetSurf_obj->m_arrayFace.push_back(f);
//			// quad mesh: triangulate
//			int l=atoi(temp);
//			f.setIndex(i-1,k-1, l-1, Buffer::VERTEX);
//		}
//		else
//			file.Undo();
        tetSurf_obj->addFace(f);
    }
    file.CloseFile();

	tetSurf_obj->saveObj("surface.obj", false, false);
    return tetSurf_obj;
}


void TetraDeformTestWin::onCallback(FlLayout::Widget const &w, Fl_Widget *pWidget, int userData) {
	if (w.mId =="Load a deformed mesh"){
        TString mesh=_meshName + "_deformed.obj"; //"deformed.mesh"; // target mesh file name.
        TString mesh_orig=_meshName + ".obj"; //"../mesh/Deformed_groot/iamgroot9k2.obj"; // corresponding undeformed mesh  (with the same topology).


        OBJloader::MeshToEntity::Option o;
        o.useTexCoord = false;
        o.useNormal = false;
        o.buildEdgeList = false;
		mTargetMesh=new OBJloader::Mesh();
		mTargetMesh->loadObj(mesh.ptr());
		mTargetMesh_originalShape=new OBJloader::Mesh();
		mTargetMesh_originalShape->loadObj(mesh_orig.ptr());

		if (0)
		{
			// calculate mapping from deformed mesh to control mesh
			/*
			mTargetMeshInControlVolume = createMeanValueMeshDeformer();
			Msg::verify(tet_to_obj, "Error! tet_to_obj first!");
			mTargetMeshInControlVolume ->calculateCorrespondence(*tet_to_obj, *mTargetMesh_originalShape);

			int nv=mTargetMesh_originalShape->numVertex();
			int ntv=getNumSurfaceVertex();
			for(int i=0; i<nv; i++)
			{
				vectorn& w=((LinearMeshDeformer*)mTargetMeshInControlVolume)->getWeights(i);
				std::cout<<i <<" : "<<w.size() <<","<<w.sum()<<std::endl;
			}

			// w11 w12 w13 ...         
			// w21 w22 w23 ...
			// ...
			matrixn W(nv*3, ntv*3);
			W.setAllValue(0.0);
			for(int i=0; i<nv; i++)
			{
				vectorn& w=((LinearMeshDeformer*)mTargetMeshInControlVolume)->getWeights(i);
				for(int j=0; j<ntv; j++)
				{
					double wj=w(j);
					W(i*3, j*3)=wj;
					W(i*3+1, j*3+1)=wj;
					W(i*3+2, j*3+2)=wj;
				}
			}

			matrixn &invW=targetDeformToControlDeform;
			m::pseudoInverse(invW, W);
			std::cout<< invW.rows()<<","<<invW.cols()<<std::endl;
			*/
		}
		else
		{

			mControlPointsInFeatureMesh = createKNNIDWDeformer(10, 2.0, 0.01);
			getVertices(*tet_to_obj, m_vertices);
			m_vertices.resize(getNumSurfaceVertex());
			m_vertices_orig=m_vertices;

			/*
			//ASSERT(mTargetMesh_originalShape->numVertex()==mTargetMesh->numVertex());
			vector3N temp;
			getVertices(*mTargetMesh_originalShape, temp);
			temp.resize(mTargetMesh_originalShape, mTargetMesh->numVertex());

			*mTargetMesh_originalShape=*mTargetMesh;
			setVertices(*mTargetMesh_originalShape,temp);
			*/

			mControlPointsInFeatureMesh -> calculateCorrespondence(*mTargetMesh_originalShape, m_vertices);
		}

        mpTargetObjViewer = new OBJloader::MeshToEntity(*mTargetMesh, RE::generateUniqueName(), o);
        mpTargetObjEntity = mpTargetObjViewer->createEntity("targetmesh", "lightgrey_transparent");
		mpTargetObjEntity  ->setCastShadows(false);
		auto* node=RE::ogreRootSceneNode()->createChildSceneNode("TargetMeshNode");
		RE::moveEntity(node, vector3(200,0,0));
		node->attachObject(mpTargetObjEntity);

		_compareHelper.SetReferenceMesh(mTargetMesh);
	}
    else if (w.mId == "Load a mesh") {
        TString mesh;
        mesh = FlChooseFile("Choose mesh", "../mesh/", "*.tet");
		if (mesh == nullptr) {
			return;
		}
//		mesh=TString("../mesh/iamgroot101_surface.1by10.rearranged.tet");
		_meshName = mesh.subString(0, mesh.length()-4);
		std::cout<<mesh.ptr()<<std::endl;
        if (mesh.length()) 
			loadTetMesh(mesh.ptr());
		tetToObj();
    } 
	else if (w.mId == "save_current") {
		mGrootHighMesh.saveObj(_meshName + ".mesh", false, false);
		printf("saved to deformed.mesh\n");
	}
	else if (w.mId == "Option") {
        if (userData == Hash("Show mesh")) {
            Fl_Choice *p = w.menu();
            if (p->menu()[p->value()].value()) {
            } else {
                    //mMeshViewer->getLastCreatedEntity()->setVisible(false);
            }
        }
    } else if (w.mId == "activation") {
        float v = w.slider()->value();
        for (int i = 0; i < mMesh.getNumTetra(); i++) {
            if (mSelectedTetra.size() && mSelectedTetra[i])
                mSoftWorld.setActivationLevel(i, v);
        }
    } else if (w.mId == "min_select") {
        min_selected_mesh = w.slider()->value();
    } else if (w.mId == "max_select") {
        max_selected_mesh = w.slider()->value();
    } else if (w.mId == "start_optimize") {

		/*
		vectorn targetDeform(mTargetMesh->numVertex()*3);
		for(int i=0; i<mTargetMesh->numVertex(); i++)
			targetDeform.setVec3(i*3, mTargetMesh->getVertex(i)-mTargetMesh_originalShape->getVertex(i));
		matrixn const&invW=targetDeformToControlDeform;
		vectorn controlDeform(invW.rows());

		Msg::verify(targetDeform.size()==invW.cols(), "a");
		Msg::verify(controlDeform.size()==invW.rows(), "b");
		controlDeform.column().mult(invW,targetDeform.column());
		*/
		Msg::verify(mTargetMesh, "load deformed first!");
		mControlPointsInFeatureMesh->transfer(*mTargetMesh, 1.0);

		/*
		vector3N temp;
		getVertices(*mTargetMesh, temp);
		mObjectList.registerObject("vertices2", "QuadListZ", "blueCircle", matView(temp),0);

		mObjectList.registerObject("vertices", "QuadListZ", "redCircle", matView(m_vertices),0);
		*/
		int surface_vtx_num = getNumSurfaceVertex();
		for (int i = 0; i < surface_vtx_num; i++) {
			double gain=1;
			//mSoftWorld.setAttachmentPoint(i, mMesh.nodePos(i)+gain*controlDeform.toVector3(i*3));
			mSoftWorld.setAttachmentPoint(i, m_vertices_orig(i)+gain*(m_vertices(i)-m_vertices_orig(i)));
		}

		printf("reset attachments\n");
		// todo :  compare버튼을 누르면 
		// 2. mGrootHighMesh.getVertex()와 mTargetMesh->getVertex의 오차 비교
		/*
        while (!p_cma->testForTermination().empty()) {
            p_cma->samplePopulation();
            for (int i = 0; i < p_cma->numPopulation(); i++) {

            }
            p_cma->update();
        }
		*/
    }
}

void TetraDeformTestWin::redraw() {
    vector3N cst;
    mSoftWorld.getAttachmentCst(cst);
    vector3N elts;
    mSoftWorld.getCorotateFEMCst(elts);
	bool bNew=false;
	if(false){
		if (!line) 	{
			bNew=true;
			line = new BillboardLineList(RE::generateUniqueName(), elts.rows(), 0.4);
			ano_line = new BillboardLineList(RE::generateUniqueName(), elts.rows(), 0.4);
			line->setMaterialName("red");
		}
		//ano_line->setMaterialName("blue");
		for (int i = 0; i < elts.rows() - 1; i += 2) {
			if (i > min_selected_mesh * 10000 && i < max_selected_mesh * 10000)
				ano_line->line(i, elts.row(i), elts.row(i + 1));
			else
				line->line(i, elts.row(i), elts.row(i + 1));
		}
		if(bNew)
			//for test tetra obj
			RE::ogreRootSceneNode()->attachObject(line);
		//RE::ogreRootSceneNode()->attachObject(ano_line);
	}

	if ( mpTetraObjViewer && tet_to_obj)
	{
		int surface_vtx_num = getNumSurfaceVertex(); // -.-
		for (int i = 0; i < surface_vtx_num; i++) 
		{
			tet_to_obj->getVertex(i)=mSoftWorld.getVertex(i);
			ASSERT(tet_to_obj->getVertex(i).x==tet_to_obj->getVertex(i).x);
		}

		mpTetraObjViewer->updatePositions();

		mMeshDeformer ->transfer(*tet_to_obj,1.0);
		mGrootHighMesh.calculateVertexNormal();
		mpSurfaceObjViewer->updatePositionsAndNormals();
	}

	double max, min, avg;
	if (_compareHelper.RMSE(mTargetMesh, &mGrootHighMesh, &max, &min, &avg) == 0) // Success
	{
		double pcm;
		_compareHelper.PointCloudMetricError(mTargetMesh, &mGrootHighMesh, &pcm);
		ChangeError(pcm, avg);
	}
}

FlLayout *createTetraDeformTestWin(int x, int y, int w, int h, MotionPanel &mp, FltkRenderer &renderer) {
    return new TetraDeformTestWin(x, y, w, h, mp, renderer);
}

void TetraDeformTestWin::clearSelection() {

    if (mSelectedVertices.size())
        mSelectedVertices.clearAll();
    if (mSelectedEdges.size())
        mSelectedEdges.clearAll();
}

void TetraDeformTestWin::drawSelection() {
    mObjectList.clear();
    for (int i = 0; i < mMesh.getNumNode(); i++) {
        if (mSelectedVertices[i]) {
            //RE::moveEntity(mObjectList.registerEntity(RE::generateUniqueName(), "sphere1010.mesh"), quater(1, 0, 0, 0),
            RE::moveEntity(mObjectList.registerEntityScheduled("sphere1010.mesh", 1.0), quater(1, 0, 0, 0),
                           mMesh.nodePos(i));
            double *getpos = new double[3];
            mMesh.nodePos(i).getValue(getpos);
            //printf("pos is %f %f %f \n",getpos[0],getpos[1],getpos[2]);
            //fflush(stdout);
        }
    }
}

inline intersectionTest::LineSegment Ray2LineSegment(Ray const &r) {
    return intersectionTest::LineSegment(r.origin(), r.getPoint(FLT_MAX));
}


void TetraDeformTestWin::setLinearConstraint() {
    for (int i = 0; i < mMesh.getNumTetra(); i++) {
        int musclestiffness = 4000000;
        vector3 fiber_direction(0, 1, 0);
        std::string muscle_name = "m" + std::to_string(i);
        mSoftWorld.addLinearMuscleConstraint(muscle_name.c_str(), musclestiffness, i, mMesh, fiber_direction);
    }
    int surface_vtx_num = getNumSurfaceVertex();
    for (int i = 0; i < surface_vtx_num; i++) {
        std::string muscle_name = "a" + std::to_string(i);
        int musclestiffness = 4000000;
		mSoftWorld.addAttachmentConstraint(muscle_name.c_str(), musclestiffness, i, mMesh.nodePos(i));
	}


	
    mSoftWorld.initSimulation();
}

#include <FL/Fl.H>
#include <Vision/realsense3.h>

int TetraDeformTestWin::handleRendererEvent(int ev) {
    static int pushed_x;
    static int pushed_y;
    static SelectionRectangle *mRect = NULL;

    switch (ev) {
        case FL_PUSH:
            pushed_x = Fl::event_x();
            pushed_y = Fl::event_y();
            if (Fl::event_state() & FL_SHIFT) {
                // marquee selection
                //mRenderer.viewLock();
                Ogre::SceneNode *pNode = RE::createSceneNode("MarqueeNode");
                mRect = new SelectionRectangle("Selection SelectionRectangle");
                pNode->attachObject(mRect);

                mRect->setCorners(TRect(pushed_x, pushed_y, pushed_x, pushed_y));
                RE::output("marqueeEvent", "push");
                return 1;
            }

            break;
//	case FL_MOVE: 
        case FL_DRAG: {

        }
            break;
        case FL_RELEASE: {
            if (mRect) {
                // marquee selection
                mRect->setCorners(TRect(pushed_x, pushed_y, Fl::event_x(), Fl::event_y()));
                std::vector<Plane> vol;
                mRect->constructSelectionVolumn(mRenderer, vol);
                clearSelection();
                mSelectedVertices.resize(mMesh.getNumNode());
                mSelectedVertices.clearAll();
                mSelectedTetra.resize(mMesh.getNumTetra());
                mSelectedTetra.clearAll();

                for (int i = 0; i < mMesh.getNumNode(); i++) {
                    Sphere s(mMesh.nodePos(i), 0.001);
                    if (s.isInside(vol))
                        mSelectedVertices.setAt(i);
                }
                for (int i = 0; i < mMesh.getNumTetra(); i++) {
                    Sphere d(mMesh.calcTetraCenter(i), 0.001);
                    if (d.isInside(vol)) {
                        mSelectedTetra.setAt(i);
                    }
                }

                drawSelection();
                //setLinearConstraint();
                RE::removeEntity("MarqueeNode");
                mRect = NULL;
                return 1;
            } else {
                static vector3 pushed_cursorPos;
                int x = Fl::event_x();
                int y = Fl::event_y();

                if (x != pushed_x || y != pushed_y)
                    return 0;
                int button = Fl::event_button();
                bool bButton1 = Fl::event_state() & FL_BUTTON1;
                bool bButton2 = Fl::event_state() & FL_BUTTON2;
                bool bButton3 = Fl::event_state() & FL_BUTTON3;

                switch (ev) { // TODO: ev is always FL_RELEASE here...
                    case FL_PUSH:
                        RE::output("mouse event", "push %d %d %d", x, y, button & FL_BUTTON1);
                        break;
                    case FL_MOVE:
                        RE::output("mouse event", "move %d %d %d", x, y, button & FL_BUTTON1);
                        break;
                    case FL_DRAG:
                        RE::output("mouse event", "drag %d %d %d", x, y, button & FL_BUTTON1);
                        break;
                    case FL_RELEASE:
                        RE::output("mouse event", "release %d %d %d", x, y, button & FL_BUTTON1);
                        break;
                }
                RE::output("button", "%d", button);
                if ((ev == FL_PUSH || ev == FL_RELEASE) && button != FL_LEFT_MOUSE) break;
                Ray ray;
                mRenderer.screenToWorldRay(x, y, ray);

                //TString selectionMode = findMenu("selection mode")->text();
                TString selectionMode = "none";

                if (selectionMode == "edge") {
                    intersectionTest::LineSegment sx = Ray2LineSegment(ray);

                    m_real minDist = FLT_MAX;
                    int argMin = -1;
                    for (int i = 0; i < mLinks.rows(); i++) {
                        intersectionTest::LineSegment l(mMesh.nodePos(mLinks[i][0]), mMesh.nodePos(mLinks[i][1]));
                        m_real dist = l.minDist(sx);
                        if (dist < minDist) {
                            minDist = dist;
                            argMin = i;
                        }
                    }

                    clearSelection();
					/*
                    BillboardLineList *line = new BillboardLineList("edge", 1, 1);
                    line->setMaterialName("redToOrange");
                    line->line(0, mMesh.nodePos(mLinks[argMin][0]), mMesh.nodePos(mLinks[argMin][1]));
                    mObjectList.registerObject("link", line);
                    RE::moveEntity(mObjectList.registerEntity("sellink0", "sphere1010.mesh"), quater(1, 0, 0, 0),
                                   mMesh.nodePos(mLinks[argMin][0]));
                    RE::moveEntity(mObjectList.registerEntity("sellink1", "sphere1010.mesh"), quater(1, 0, 0, 0),
                                   mMesh.nodePos(mLinks[argMin][1]));
								   */
                } 
				else if(selectionMode == "vertex")
				{
                    clearSelection();
                    for (int i = 0; i < mMesh.getNumNode(); i++) {
                        Sphere s(mMesh.nodePos(i), 3);
                        if (ray.intersects(s).first) {

                            RE::moveEntity(mObjectList.registerEntity(RE::generateUniqueName(), "sphere1010.mesh"),
                                           quater(1, 0, 0, 0), mMesh.nodePos(i));
                        }
                    }
                }
            }
        }
            break;
    }
    return 0;
}

OBJloader::Mesh* TetraDeformTestWin::UpdateMesh(OBJloader::Mesh* source) {
    auto mv = VisionProject::GetInstance()->GetMotionVector();

    for (int i = 0; i < mv.size(); ++i) {
		source->getVertex(mv[i].index) = vector3(mv[i].x, mv[i].y, mv[i].z);
    }

    return source;
}

void TetraDeformTestWin::ChangeError(double pcm, double rms) {
	char pcmString[32];
	char rmsString[32];
	sprintf(pcmString, "PCM Error : %.2lf mm", pcm);
	sprintf(rmsString, "Avg. Distance : %.2lf mm", rms);
	_maxWidget->copy_label(pcmString);
	_avgWidget->copy_label(rmsString);
	updateLayout();
}


void TetraDeformTestWin::tetToObj()
{
	//mMeshDeformer = createHarmonicMeshDeformer();
	mMeshDeformer = createMeanValueMeshDeformer();
	//mMeshDeformer = createKNNIDWMeshDeformer(30, 2.0);
	tet_to_obj = tetraToObj(mMesh, _meshName + ".face", getNumSurfaceVertex());  //"../mesh/iamgroot101_surface.1.rearranged.face"
	//mGrootHighMesh.loadObj("../mesh/Deformed_groot/iamgroot9k.obj"); // smaller mesh.
	mGrootHighMesh.loadObj(_meshName + ".obj"); // larger mesh.
	if(1)
	{
		// slightly scale down
		auto center=mGrootHighMesh.calcMeshCenter(); //?? 메시 센터가 이상하게 계산되는듯.
		matrix4 m;
		m.identity();
		m.leftMultTranslation(-center);
		m.leftMultScaling(0.99,0.99, 0.99);
		m.leftMultTranslation(center);
		mGrootHighMesh.transform(m);
	}

	// Load or save correspondence
	//auto dfn = "../mesh/iamgroot_high_hmd.def";
	auto dfn = "../mesh/iamgroot_high_mvn4.def";
	//if (!mMeshDeformer->loadCorrespondence(dfn, *tet_to_obj, mGrootHighMesh)) {
	if (1){ // mvn is fast enough.
		mMeshDeformer->calculateCorrespondence(*tet_to_obj, mGrootHighMesh);
		mMeshDeformer->saveCorrespondence(dfn);
	}

	//for test tetra obj
	//RE::ogreRootSceneNode()->detachObject(line);
	//RE::ogreRootSceneNode()->detachObject(ano_line);
	OBJloader::MeshToEntity::Option o;
	o.useTexCoord = false;
	o.useNormal = false;
	o.buildEdgeList = false;

	mpSurfaceObjViewer = new OBJloader::MeshToEntity(mGrootHighMesh, RE::generateUniqueName(), o);
	//mpSurfaceObjEntity = mpSurfaceObjViewer->createEntity("surfaceobj", "lambert6");
	mpSurfaceObjEntity = mpSurfaceObjViewer->createEntity("surfaceobj", "lightgrey_transparent");
	mpSurfaceObjEntity  ->setCastShadows(false);
	RE::ogreRootSceneNode()->attachObject(mpSurfaceObjEntity);

	if(1){
		mpTetraObjViewer = new OBJloader::MeshToEntity(*tet_to_obj, RE::generateUniqueName(), o);
		//mpTetraObjEntity = mpTetraObjViewer->createEntity("tettoobj", "lambert6");
		mpTetraObjEntity = mpTetraObjViewer->createEntity("tettoobj", "red_transparent"); // defined in color.material
		mpTetraObjEntity  ->setCastShadows(false);
		RE::ogreRootSceneNode()->attachObject(mpTetraObjEntity);
	}
	else
		mpTetraObjViewer=NULL;
}
