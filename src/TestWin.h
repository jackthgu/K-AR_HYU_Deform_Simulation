#ifndef TESTWIN_H_
#define TESTWIN_H_

#include <ElasticBody/CompareHelper.h>
#include <cma/CMAwrap.h>
#include <MainLib/OgreFltk/objectList.h>
#include <cma/cmaes.h>
#include <ElasticBody/MeanValueWeights.h>
#include <femworld/IntegratedWorld.h>
#include "MainLib/OgreFltk/VRMLloader.h"
#include "BaseLib/motion/MotionDOF.h"
#include "BaseLib/motion/FullbodyIK_MotionDOF.h"
#include "FltkOpenGLrenderer.h"
#include "MainLib/OgreFltk/RE.h"
#include <OGRE/OgreEntity.h>

class CImage;

using namespace TetraMeshLoader;

bool isTestWinInitialized();
double* estimateSurfaceVerticesFromCurBarycentrics();

class TestWin : public FlLayout, public INativeRender, public FrameMoveObject, public FltkRenderer::Handler {
	public:// modified to public for implementing the estimateSurfaceVerticesFromCurBarycentrics function.
	TetraMesh mMesh;
	IntegratedWorld mSoftWorld;


	std::shared_ptr<Ogre::SceneNode> mNode;

	MeshDeformer *mMeshDeformer;
#if 0
	LinearDeformer *mFeaturePointsInControlVolume;
#else
	LinearDeformer *mControlPointsInFeatureMesh;
	LinearDeformer *mControlPointsInFeatureMesh2;
	OBJloader::Mesh m_featureMesh;
	vector3N m_vertices; // for the attachment CSTs
	vector3N m_vertices2; // for the attachment CSTs
	vector3N m_vertices_orig2; // for the attachment CSTs
#endif

	static void updateAllFeaturePointsFromCurBarycentrics(vector3N* mFeaturePoints, vector3N* mFeaturePoints_originalShape, vector3N* mCutFeaturePoints);
	void setGLview();
	void syncViewMatrix();
	void tetToObj();
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

	intvectorn _fidxToFeatureIndex;
	intvectorn _frontToBack;
	int fIdxToFeatureIndex(int fIdx);
	
public:
	QPerformanceTimer2 timer;

	// FltkRenderer::Handler
	virtual int handleRendererEvent(int ev);

	TestWin(int x, int y, int w, int h, MotionPanel &mp, FltkRenderer &renderer);

	~TestWin();

	virtual void show() {
		FlLayout::show();

		FltkRenderer &mRenderer = RE::FltkRenderer();
		mRenderer.setHandler(this);
	}

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

	void loadTetMesh(const char *mesh);


	MotionPanel &m_motionPanel;
	FltkRenderer &mRenderer;


	void redraw();



	void initialize();

	void onCallback(FlLayout::Widget const &w, Fl_Widget *pWidget, int userData);

	// FrameMoveObject
	virtual int FrameMove(float fElapsedTime);

	void setLinearConstraint();

	void clearSelection();

	boolN mSelectedVertices;
	boolN mSelectedEdges;
	boolN mSelectedTetra;

	void drawSelection();

	OBJloader::Mesh *mTargetMesh2;
	OBJloader::Mesh *mTargetMesh_originalShape2;
	OBJloader::Mesh *tet_to_obj;
	vector3N *mFeaturePoints;
	vector3N *mFeaturePoints_originalShape;
    vector3N *mCutFeaturePoints;
	OBJloader::Mesh mGrootHighMesh;

	void NanCheck(OBJloader::Mesh *target);
    bool triggerDrawFirstPose = true;

protected:
	void nativeRender();

	void nativePreRender();

	void LoadDeformedMesh();
private:
	// For apply motion vector to Obj Mesh
	OBJloader::Mesh *UpdateMesh(OBJloader::Mesh *source);

	void ChangeError(double pcm, double rms);

	Fl_Widget* _pcmWidget;
	Fl_Widget* _rmsWidget;
	CompareHelper _compareHelper;

	TString _meshName; // File path + name without expansion

};

#endif
