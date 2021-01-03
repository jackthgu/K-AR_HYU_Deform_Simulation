#include "stdafx.h"
#include <Fl/Fl_Window.H>
#include <Fl/Fl_Double_Window.H>
#include <Fl/Fl_Button.H>
#include <Fl/Fl_Slider.H>
#include "BaseLib/motion/Motion.h"
#include "BaseLib/motion/MotionDOF.h"
#include "BaseLib/motion/FullbodyIK_MotionDOF.h"
#include "MainLib/OgreFltk/VRMLloader.h"
#include "MainLib/OgreFltk/FlLayout.h"
#include "MainLib/OgreFltk/renderer.h"
#include "TestWin.h"
#include "Vision/realsense3.h"
#include "Vision/SpacePrimitives.h"
#include "MainLib/OgreFltk/MotionPanel.h"
#include "MainLib/OgreFltk/FlChoice.h"
#include "MainLib/OgreFltk/PointClouds.h"
#include "MainLib/OgreFltk/FltkMotionWindow.h"
#include "MainLib/OgreFltk/FltkScrollPanel.h"
#include "MainLib/OgreFltk/RE.h"
#include <FL/Fl.H>
#include <BaseLib/math/Operator.h>
#include <BaseLib/math/Operator_NR.h>
#include <iostream>

#include "BaseLib/utility/QPerformanceTimer.h"

#ifdef EPS
#undef EPS
#endif
#include <opencv2/opencv.hpp>

#if 1
#define DBG_glBegin(x)
#define DBG_glVertex3f(x,y,z)
#define DBG_glEnd(x)
#else
#define DBG_glBegin(x)	glBegin(x)
#define DBG_glVertex3f(x,y,z) glVertex3f(x,y,z)
#define DBG_glEnd(x)	glEnd(x)
#endif

//#define TEST_DEFORM_USING_ARTIFICIALDATA
extern std::vector<space::Barycentric> refBarycentrics;
extern std::vector<space::Barycentric> curBarycentrics;
extern std::vector<space::Barycentric> markerBarycentrics;
extern float *glProjectionMat;

static bool drawTetEdges=false;
static bool drawTetMesh=true;

vector3N operator+ (vector3N const& a, vector3N const& b)
{
	vector3N o;
	o.setSize(a.size());
	for(int i=0; i<a.size(); i++)
		o(i)=a(i)+b(i);
	return o;
}
vector3N operator- (vector3N const& a, vector3N const& b)
{
	vector3N o;
	o.setSize(a.size());
	for(int i=0; i<a.size(); i++)
		o(i)=a(i)-b(i);
	return o;
}
vector3N operator* (double a, vector3N const& b)
{
	vector3N o;
	o.setSize(b.size());
	for(int i=0; i<b.size(); i++)
		o(i)=a*b(i);
	return o;
}
inline matrix4 toMatrix4(float* mat)
{
	matrix4 out;
	int c=0;
	for(int i=0; i<4; i++)
		for(int j=0; j<4; j++)
			out(j,i)=mat[c++];
	return out;
}
extern float g_translateScale ;
extern float g_rotationScale ;
extern int g_rightKeyCnt ;
extern int g_upKeyCnt ;
extern int g_forwardKeyCnt ;
extern int g_xRotKeyCnt ;
extern int g_yRotKeyCnt ;
extern int g_zRotKeyCnt ;
extern int g_frameCnt ;
extern cv::Mat modelview;
static int bInitialized=false;
static TestWin* g_pTestWinSingleton=NULL;

bool isTestWinInitialized()
{
	return bInitialized;
}
void getVertices(OBJloader::Mesh const& mesh, vector3N & o)
{
	o.resize(mesh.numVertex());
	for (int i=0; i< mesh.numVertex(); i++)
		o(i)=mesh.getVertex(i);
}
void setVertices(OBJloader::Mesh & mesh, vector3N const& o)
{
	ASSERT(mesh.numVertex()==o.size());
	for (int i=0; i< mesh.numVertex(); i++)
		mesh.getVertex(i)=o(i);
}

// function 1
void TestWin::updateAllFeaturePointsFromCurBarycentrics(vector3N* mFeaturePoints, vector3N* mFeaturePoints_originalShape, vector3N* mCutFeaturePoints)
{
	ASSERT(bInitialized);
#ifdef DRAW_DELTA
		BillboardLineList *line = new BillboardLineList("track", curBarycentrics.size(), 1);
		line->setMaterialName("redToOrange");
#endif
		DBG_glBegin(GL_POINTS);
		//MOutputToFile("points.txt", ("{ "));
		for (int i = 0; i < curBarycentrics.size(); i++) {
			space::Point a = curBarycentrics[i].ray.ori + (curBarycentrics[i].ray.dir * curBarycentrics[i].t);

			int fIdx=curBarycentrics[i].fIdx;
			ASSERT(fIdx>=0);
			int j=g_pTestWinSingleton->fIdxToFeatureIndex(fIdx);

			//MOutputToFile("points.txt", ("{ fIdx=%d, j=%d, vidx=vector3(%d, %d, %d), a=vector3(%f, %f, %f), coeffs=vector3(%f, %f, %f) }, ", fIdx, j, curBarycentrics[i].idx.x,curBarycentrics[i].idx.y,curBarycentrics[i].idx.z, a.x, a.y, a.z, curBarycentrics[i].coeffs[0],curBarycentrics[i].coeffs[1],curBarycentrics[i].coeffs[2] ));
			if(j==-1) continue;
			int backFace=g_pTestWinSingleton->_frontToBack(j);
			//int j=fIdxToFeatureIndex(i);


#ifdef TEST_DEFORM_USING_ARTIFICIALDATA
			static float t=0;
			//	j=i*2;

			ASSERT(j<refBarycentrics.size()&& j>=0);
			a=refBarycentrics[j].ray.ori + (refBarycentrics[j].ray.dir * refBarycentrics[j].t);

			//a=a-space::Vector(0,t*sop::map(i, 0, curBarycentrics.size()-1, 0.1, 1),0);
			a=a-space::Vector(0,t*sop::map(a.y, 10, -50, 1, 0.1),0);
			t=t+0.02;


#endif
			// curBarycentrics: 비젼쪽 입력 ->
			// mFeaturePoints -> 우리쪽에서 관리하는 feature point의 배열, 뒷면점 포함.

			ASSERT(j<mFeaturePoints->size() && j>=0);
			(*mFeaturePoints)[j]=vector3(a.x, a.y, a.z);

			if(1 && backFace>=0 ){
				vector3N& ori=*mFeaturePoints_originalShape;
				vector3N& curr=*mFeaturePoints;

				(*mFeaturePoints)[backFace]=ori(backFace)+(curr(j)-ori(j));
			}

#ifdef DRAW_DELTA
			line->line(i, (*mFeaturePoints)[j], (*mFeaturePoints_originalShape)[j]);
#endif
			DBG_glVertex3f(a.x, a.y, a.z);
		}

		//MOutputToFile("points.txt", ("},\n"));
        for (int i = 0; i < markerBarycentrics.size(); ++i) {
            space::Point a = markerBarycentrics[i].ray.ori + (markerBarycentrics[i].ray.dir * markerBarycentrics[i].t);
//            printf("Now Cut feature's Points is %d\n", markerBarycentrics.size());
            (*mCutFeaturePoints)[i]=vector3(a.x, a.y, a.z);
        }

#ifdef DRAW_DELTA
		mObjectList.registerObject("trackNode", line);
#endif

		DBG_glEnd();
}

int TestWin::fIdxToFeatureIndex(int fIdx) { int nv=mFeaturePoints->size(); return _fidxToFeatureIndex(fIdx+nv);}

void TestWin::nativePreRender() {
	//std::cout <<"NativePreRender" << std::endl;
	glClearColor(0.f, 0.f, 0.f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_LIGHTING);

	glScalef(30.0, 30.0, 30.0);

	// All polygons have been drawn.
	glEnd();
	if (findWidget("draw MR").checkButton()->value())
	{
	timer.start();
		VisionProject::GetInstance()->RenderingLoop();
		syncViewMatrix();
	printf("MRLAB time1: %d \n", timer.stop2());
	}
	else
	{
	}
	
	if(refBarycentrics.size()>0 and !bInitialized)
	{
		bInitialized=true;
		g_pTestWinSingleton=this;
		ASSERT(refBarycentrics.size()>0);

#ifdef TEST_DEFORM_USING_ARTIFICIALDATA
		if(0)
		{
			// trying to use all the vertices to see if that works
			auto* mesh=new OBJloader::Mesh();
			mesh->loadObj("../mesh/testgroot.mesh");

			{
				auto center = vector3(0,0,0);
				matrix4 m;
				m.identity();
				m.leftMultTranslation(-center);
				m.leftMultScaling(0.5, 0.5, 0.5); // taesoo scale
				m.leftMultTranslation(center);
				mesh->transform(m);
			}
			int nvo=mesh->numVertex();

			/*
			printf("nvo: %d\n", nvo);
			for(int i=0; i< refBarycentrics.size(); i++)
			{
				auto ii=refBarycentrics[i].idx;
				printf("%d %d %d\n", ii.x, ii.y, ii.z);
			}
			exit(0);
			*/

			refBarycentrics.resize(nvo);
			for(int i=0; i<nvo; i++)
			{
				auto v=mesh->getVertex(i);
				refBarycentrics[i].ray.ori=space::Point(v.x, v.y, v.z);
				refBarycentrics[i].t=0;
				refBarycentrics[i].fIdx=i;
			}
		}
#endif

		int nv=refBarycentrics.size();
		_fidxToFeatureIndex.setSize(nv*2); // so that its large enough 
		_fidxToFeatureIndex.setAllValue(-1);

		_frontToBack.setSize(nv);
		_frontToBack.setAllValue(-1);

		mFeaturePoints = new vector3N(nv);
		mFeaturePoints_originalShape = new vector3N(nv);
		for (int i = 0; i < refBarycentrics.size(); i++) {
			int fIdx=refBarycentrics[i].fIdx;
			space::Point a = refBarycentrics[i].ray.ori + (refBarycentrics[i].ray.dir * refBarycentrics[i].t);
			(*mFeaturePoints_originalShape)[i]=vector3(a.x, a.y, a.z);
			ASSERT(fIdx+nv>=0 && fIdx+nv<nv*2);
			_fidxToFeatureIndex(fIdx+nv)=i;
			//printf("%d : %d ", fIdx, i);
			if(fIdx<0)
			{
				int j=i-1; // corresponding front feature
				_frontToBack(j)=i;
			}
		}
		(*mFeaturePoints)=(*mFeaturePoints_originalShape);

        mCutFeaturePoints = new vector3N(8);    // TODO: Hard coding aobut number of knife features, Why 8?

		if(0){
			intvectorn fidx(refBarycentrics.size());
			for (int i = 0; i < refBarycentrics.size(); i++) {
				int fIdx=refBarycentrics[i].fIdx;
				fidx(i)=fIdx;
			}

			BinaryFile f;
			f.openWrite("featurepoints.dat");
			f.pack(*mFeaturePoints);
			f.pack(fidx);
			f.close();
		}

		{
			// "Load a deformed mesh"

			mControlPointsInFeatureMesh = createKNNIDWDeformer(10, 2.0, 0.01);
			auto v=mFeaturePoints;
			m_featureMesh.resize(v->size(), 0);
			setVertices(m_featureMesh, *v);
			getVertices(*tet_to_obj, m_vertices);
			mControlPointsInFeatureMesh -> setEmbeddedMesh(tet_to_obj);
			mControlPointsInFeatureMesh -> calculateCorrespondence(m_featureMesh, m_vertices);
		}
	}
}

void TestWin::syncViewMatrix()
{
	RE::renderer().viewport(0).setCustomProjectionMatrix(toMatrix4(glProjectionMat));
	float mvArr[16];
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	setGLview();
	glGetFloatv(GL_MODELVIEW_MATRIX, mvArr);
	glPopMatrix();
	RE::renderer().viewport(0).changeView(toMatrix4(mvArr));
	//std::cout<<toMatrix4(mvArr).getColumn(3)<<std::endl;
}
void TestWin::setGLview()
{
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	cv::Mat quat = cv::Mat::zeros(4, 1, CV_32FC1);
	double cy = cos(g_rotationScale * g_zRotKeyCnt * 0.5 * 3.141592 / 180.0f);
	double sy = sin(g_rotationScale * g_zRotKeyCnt * 0.5 * 3.141592 / 180.0f);
	double cp = cos(g_rotationScale * g_yRotKeyCnt * 0.5 * 3.141592 / 180.0f);
	double sp = sin(g_rotationScale * g_yRotKeyCnt * 0.5 * 3.141592 / 180.0f);
	double cr = cos(g_rotationScale * g_xRotKeyCnt * 0.5 * 3.141592 / 180.0f);
	double sr = sin(g_rotationScale * g_xRotKeyCnt * 0.5 * 3.141592 / 180.0f);

	quat.at<float>(0, 0) = cy * cp * cr + sy * sp * sr;
	quat.at<float>(1, 0) = cy * cp * sr - sy * sp * cr;
	quat.at<float>(2, 0) = sy * cp * sr + cy * sp * cr;
	quat.at<float>(3, 0) = sy * cp * cr - cy * sp * sr;

	float q0 = quat.at<float>(0, 0);
	float q1 = quat.at<float>(1, 0);
	float q2 = quat.at<float>(2, 0);
	float q3 = quat.at<float>(3, 0);


	cv::Mat rotMat = cv::Mat::zeros(3, 3, CV_32FC1);
	rotMat.at<float>(0, 0) = 1 - 2*(q2 * q2 + q3 * q3); rotMat.at<float>(0, 1) = 2*(q1*q2 - q0*q3); rotMat.at<float>(0, 2) = 2*(q0 * q2 + q1 * q3);
	rotMat.at<float>(1, 0) = 2 *(q1*q2+q0 * q3); rotMat.at<float>(1, 1) = 1 - 2*(q1*q1 + q3 * q3); rotMat.at<float>(1, 2) = 2 * (q2 * q3 - q0 * q1);
    rotMat.at<float>(2, 0) = 2 *(q1 * q3 - q0 * q2); rotMat.at<float>(2, 1)= 2*(q0*q1+q2*q3); rotMat.at<float>(2, 2)= 1-2*(q1*q1 + q2*q2);
	cv::Mat mvMat = cv::Mat::zeros(4, 4, CV_32FC1);
	mvMat(cv::Rect(0, 0, 3, 3)) += rotMat;
	mvMat.at<float>(0, 3) = g_translateScale * g_rightKeyCnt;
	mvMat.at<float>(1, 3) = g_translateScale * g_upKeyCnt;
	mvMat.at<float>(2, 3) = -g_translateScale * g_forwardKeyCnt;
	mvMat.at<float>(3, 3) = 1.0;

    mvMat = modelview * mvMat;
	mvMat= mvMat.t();
	glLoadMatrixf((GLfloat*)mvMat.data);
//    q.w() = cy * cp * cr + sy * sp * sr;
//    q.x() = cy * cp * sr - sy * sp * cr;
//    q.y() = sy * cp * sr + cy * sp * cr;
//    q.z() = sy * cp * cr - cy * sp * sr;

	//glRotatef(g_rotationScale * g_zRotKeyCnt, 0.0f, 0.0f, 1.0f);
	//glRotatef(g_rotationScale * g_yRotKeyCnt, 0.0f, 1.0f, 0.0f);
	//glRotatef(g_rotationScale * g_xRotKeyCnt, 1.0f, 0.0f, 0.0f);
}
void TestWin::nativeRender() {

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	//glLoadMatrixf(ProjectionMatrix);
	//perspectiveGL(60, (double)FRAME_WIDTH / (double)FRAME_HEIGHT, 0.1, 1000);
	glLoadMatrixf(glProjectionMat);

	setGLview();

	float oldPointSize;
	glGetFloatv(GL_POINT_SIZE, &oldPointSize);
	glPointSize(16.0f);
	/*
	glColor3f(1.0f, 1.0f, 0.0f);
	for (int i = 0; i < ref3DCorners.size(); i++) {
		glBegin(GL_POINTS);
		glVertex3f(ref3DCorners[i].x, ref3DCorners[i].y, ref3DCorners[i].z);
		glEnd();
	}
	*/

	glDisable(GL_LIGHTING);

	glColor3f(0.0f, 0.0f, 1.0f);
	RE::output("curbarycen", "%d", curBarycentrics.size());
	// function 3
	if(bInitialized)
	{
		//std::cout << "NativeRender init" << std::endl;
		updateAllFeaturePointsFromCurBarycentrics(mFeaturePoints, mFeaturePoints_originalShape, mCutFeaturePoints);
	}

	if(0)
	{
		// draw updated feature points
		if(bInitialized)
		{
			for(int i=0; i<mFeaturePoints->size(); i++){
				mObjectList.drawSphere((*mFeaturePoints)(i) , TString("origin",i), "red_transparent", 5.0);
			}
		}
	}
	if(bInitialized)
	{
#if 0
		g_invDeformer.invDeform(mCON.conPos, g_meshOrig, g_mesh);
#else
		timer.start();
		auto& m_deformer=*mControlPointsInFeatureMesh;
		//setVertices(m_featureMesh,*mFeaturePoints);
		double gain=1.1; // -> 여기서는 1.1배 과장.
		setVertices(m_featureMesh,(*mFeaturePoints_originalShape)+gain*((*mFeaturePoints)-(*mFeaturePoints_originalShape)));
		m_deformer.transfer(m_featureMesh, 1.0);

		// KNN transfer
		// -> 결과가 m_vertices로 들어감.  (mControlPointsInFeatureMesh생성되는 부분 참고)
		//
		int surface_vtx_num = getNumSurfaceVertex();
		glColor3f(0.0f, 1.0f, 0.0f);
		DBG_glBegin(GL_POINTS);
		for (int i = 0; i < surface_vtx_num; i++) {
			// transfer결과를 스프링에 저장.
			mSoftWorld.setAttachmentPoint(i, m_vertices(i));
			auto &a=m_vertices(i);
			DBG_glVertex3f(a.x, a.y, a.z);
		}
		DBG_glEnd();

		printf("CALAB time4: %d \n", timer.stop2());
#endif
	}


	glColor3f(1.0f, 0.0f, 0.0f);
	DBG_glBegin(GL_POINTS);
	for (int i = 0; i < refBarycentrics.size(); i++) {
		space::Point a = refBarycentrics[i].ray.ori + (refBarycentrics[i].ray.dir * refBarycentrics[i].t);
		DBG_glVertex3f(a.x, a.y, a.z);
		
		ASSERT(fIdxToFeatureIndex(refBarycentrics[i].fIdx)==i);
		//mObjectList.drawSphere(vector3(a.x, a.y, a.z), TString("origin",i), "red", 5.0);
	}
	DBG_glEnd();
}


inline intersectionTest::LineSegment Ray2LineSegment(Ray const &r) {
	return intersectionTest::LineSegment(r.origin(), r.getPoint(FLT_MAX));
}

int TestWin::handleRendererEvent(int ev) {
	static int pushed_x;
	static int pushed_y;
	static SelectionRectangle *mRect = NULL;

	switch (ev) {
		case FL_KEYUP:
		case FL_KEYDOWN: {
			TString evs = "NONE";
			switch (ev) {
				case FL_KEYUP:
					evs = "KEYUP";
					break;
				case FL_KEYDOWN:
					evs = "KEYDOWN";
					break;
			}
			int key = Fl::event_key();
			VisionProject::GetInstance()->KeyInput(evs.ptr(), key);
			break;
		}
	}
// TODO: Mouse event handler part, Now we don't need it
#if 0
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
				RE::removeEntity("MarqueeNode");
				mRect = NULL;
				return 1;
			} else {
				int x = Fl::event_x();
				int y = Fl::event_y();

				if (x != pushed_x || y != pushed_y)
					return 0;
				int button = Fl::event_button();

				switch (ev) { // TODO: ev is always FL_RELEASE here...
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
					if(drawTetEdges){
						BillboardLineList *line = new BillboardLineList("edge", 1, 1);
						line->setMaterialName("redToOrange");
						line->line(0, mMesh.nodePos(mLinks[argMin][0]), mMesh.nodePos(mLinks[argMin][1]));
						mObjectList.registerObject("link", line);
					}
					RE::moveEntity(mObjectList.registerEntity("sellink0", "sphere1010.mesh"), quater(1, 0, 0, 0),
								   mMesh.nodePos(mLinks[argMin][0]));
					RE::moveEntity(mObjectList.registerEntity("sellink1", "sphere1010.mesh"), quater(1, 0, 0, 0),
								   mMesh.nodePos(mLinks[argMin][1]));
				} else if (selectionMode == "vertex") {
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
#endif
	return 0;
}


// function 4
// FrameMoveObject
int TestWin::FrameMove(float fElapsedTime) {
	//timer.start();
	mObjectList.FrameMove(fElapsedTime);
	if (isLoadTet) {
		//printf("CALAB time1: %d \n", timer.stop2());
		timer.start();
		mSoftWorld.TimeStepping();
		printf("CALAB time2: %d \n", timer.stop2());
		timer.start();
		redraw();
		printf("CALAB time3: %d \n", timer.stop2());
	}
	return 1;
}

// function 4
void TestWin::redraw() {
	//std::cout << "Redraw" << std::endl;
	if(drawTetEdges) {
		vector3N cst;
		mSoftWorld.getAttachmentCst(cst);
		vector3N elts;
		mSoftWorld.getCorotateFEMCst(elts);
		bool bNew = false;
		if (!line) {
			bNew = true;
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
		if (bNew)
			//for test tetra obj
			RE::ogreRootSceneNode()->attachObject(line);
		//RE::ogreRootSceneNode()->attachObject(ano_line);
	}

	if (tet_to_obj ) {
		int surface_vtx_num = getNumSurfaceVertex(); // -.-
		for (int i = 0; i < surface_vtx_num; i++) {
			tet_to_obj->getVertex(i) = mSoftWorld.getVertex(i);
		}

		if(mpTetraObjViewer)
			mpTetraObjViewer->updatePositions();

		mMeshDeformer->transfer(*tet_to_obj, 1.0);

		NanCheck(&mGrootHighMesh);

		mGrootHighMesh.calculateVertexNormal();
		mpSurfaceObjViewer->updatePositionsAndNormals();

    }

    // Load deformed mesh from third time.
    static int a = 0;
	if (mTargetMesh2 == nullptr && a++==2) {
		LoadDeformedMesh();    // createKNNIDW in here
	}

	double max, min, avg;
	if (_compareHelper.RMSE(&mGrootHighMesh, mTargetMesh2, &max, &min, &avg) == 0) // Success
	{
		double pcm;
		_compareHelper.PointCloudMetricError(&mGrootHighMesh, mTargetMesh2, &pcm);
		ChangeError(pcm, avg);
	}


	// Draw once red model(First pose)
	if (mTargetMesh2 != nullptr && triggerDrawFirstPose) {
		//std::cout<< "Draw first pose" << std::endl;
		triggerDrawFirstPose = false;

        OBJloader::MeshToEntity::Option o;
        o.useTexCoord = false;
        o.useNormal = false;
        o.buildEdgeList = false;

        OBJloader::Mesh testing;
		testing.copyFrom(mGrootHighMesh);

		auto mTetraObjViewer1 = new OBJloader::MeshToEntity(testing, RE::generateUniqueName(), o);
		auto mTetraObjEntity1 = mTetraObjViewer1->createEntity("firstPose",
															 "red_transparent"); // defined in color.material
		mTetraObjEntity1->setCastShadows(false);
//		RE::ogreRootSceneNode()->attachObject(mTetraObjEntity1);
	}
}


TestWin::TestWin(int x, int y, int w, int h, MotionPanel &mp, FltkRenderer &renderer) :
		FlLayout(x, y, w, h),
		m_motionPanel(mp),
		mRenderer(renderer),
		mFeaturePoints(NULL),
		mCutFeaturePoints(NULL),
		tet_to_obj(NULL),
		mpTetraObjViewer(NULL),
		mSoftWorld(1.0 / 60.0, 0.95) {
	create("Button", "init", "init");
	button(0)->shortcut(FL_ALT+'i');
	create("Check_Button", "draw MR", "draw MR");
	checkButton(0)->value(true);

	create("Button", "Load a mesh", "Load a mesh");
	create("Value_Slider", "activation", "activation");
	slider(0)->range(0, 0.03);
	slider(0)->value(0);

	create("Value_Slider", "min_select", "min_select");
	create("Value_Slider", "max_select", "max_select");

	create("Button", "save_current", "save_current");
	create("Button", "start_optimize", "start_optimize");
	create("Button", "tet_to_obj", "tet_to_obj");

    create("Button", "Load saved mesh", "Load saved mesh");

	updateLayout();


	_pcmWidget = create("Box", "pcm", "PCM Error : 0.0000");
	_rmsWidget = create("Box", "rms", "RMS Error : 0.0000");
	updateLayout();
	renderer.ogreRenderer().addFrameMoveObject(this);
	isLoadTet = false;
	line = NULL;
	ano_line = NULL;

	p_cma = new CMAwrap(vecn_start, vecn_dest, 10, 2);

}

TestWin::~TestWin() {
}

void TestWin::initialize() {
	TString mesh="../mesh/tri1.tet";
	_meshName = mesh.subString(0, mesh.length() - 4);
	loadTetMesh(mesh);
	tetToObj();
	Ogre::SceneNode* pBg=RE::renderer().viewport().mScene->getSceneNode("BackgroundNode");
	pBg->setVisible(false);

	//mObjectList.drawSphere(vector3(0,0,0), "origin", "red", 5.0);
	//mObjectList.drawSphere(vector3(0,10,0), "originy", "red", 5.0);
	// add this to native renderer list.
	((FltkOpenGLrenderer &) RE::FltkRenderer()).pushFrontRenderer(this);
	VisionProject::GetInstance()->InitializeGL();

	
	syncViewMatrix();
}


OBJloader::Mesh *tetraToObjfortestwin(TetraMesh &srctet, const char *facefile, int surface_vtx_num)
{
	std::ifstream fin;
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

		int i = atoi(file.GetToken());
		int j = atoi(file.GetToken());
		int k = atoi(file.GetToken());
		int temp = atoi(file.GetToken());
		f.setIndex(i - 1, j - 1, k - 1, OBJloader::Buffer::VERTEX);

		// For another obj file
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

	fin.close();
	return tetSurf_obj;
}

void TestWin::LoadDeformedMesh(){
	TString mesh=_meshName + "_deformed.obj"; //"deformed.mesh"; // target mesh file name.
	TString mesh_orig=_meshName + ".obj"; //"../mesh/Deformed_groot/iamgroot9k2.obj"; // corresponding undeformed mesh  (with the same topology).


	OBJloader::MeshToEntity::Option o;
	o.useTexCoord = false;
	o.useNormal = false;
	o.buildEdgeList = false;

	mTargetMesh2=new OBJloader::Mesh();
	mTargetMesh2->loadObj(mesh.ptr());
	mTargetMesh_originalShape2=new OBJloader::Mesh();
	mTargetMesh_originalShape2->loadObj(mesh_orig.ptr());

	{
		mControlPointsInFeatureMesh2= createKNNIDWDeformer(10, 2.0, 0.01);
		getVertices(*tet_to_obj, m_vertices2);
		m_vertices2.resize(getNumSurfaceVertex());
		m_vertices_orig2=m_vertices2;


		mControlPointsInFeatureMesh2 -> calculateCorrespondence(*mTargetMesh_originalShape2, m_vertices2);
	}


	mpTargetObjViewer = new OBJloader::MeshToEntity(*mTargetMesh2, RE::generateUniqueName(), o);
	mpTargetObjEntity = mpTargetObjViewer->createEntity("targetmesh", "blue_transparent");
	mpTargetObjEntity  ->setCastShadows(false);
	auto* node=RE::ogreRootSceneNode()->createChildSceneNode("TargetMeshNode");
	RE::moveEntity(node, vector3(0,0,0));
//	node->attachObject(mpTargetObjEntity);

	_compareHelper.SetReferenceMesh(mTargetMesh2);
}


void TestWin::onCallback(FlLayout::Widget const &w, Fl_Widget *pWidget, int userData) {
	if(w.mId=="draw MR")
	{
		if(!w.checkButton()->value())
		{
			auto& vp=RE::renderer().viewport(0);
			vp.setOrthographicMode(false);
#if 0
			// reinitialize view
			vp.m_pViewpoint->m_vecVPos=vector3(220, 53, 150);
			vp.m_pViewpoint->m_vecVAt=vector3(8, -27, 16);
			vp.m_pViewpoint->m_vecVUp=vector3(0, 1, 0);
			vp.m_pViewpoint->CalcHAngle();
			vp.m_pViewpoint->CalcVAngle();
			vp.m_pViewpoint->CalcDepth();
#else
			// roughly preserve current view direction.
			vp.m_pViewpoint->m_vecVUp=vector3(0, 1, 0);
			vp.m_pViewpoint->CalcHAngle();
			vp.m_pViewpoint->CalcVAngle();
			vp.m_pViewpoint->m_fDepth=255.0;
			vp.m_pViewpoint->UpdateVPosFromVHD();
			// recenter camera
			vp.m_pViewpoint->m_vecVPos-=vp.m_pViewpoint->m_vecVAt;
			vp.m_pViewpoint->m_vecVAt=vector3(0,0,0);
			vp.m_pViewpoint->UpdateVHD();
#endif
		}

	} else if (w.mId == "Load a mesh") {
		TString mesh;
		mesh = FlChooseFile("Choose mesh", "../mesh/", "*.tet");
//		mesh=TString("../mesh/iamgroot101_surface.1by10.rearranged.tet");
		_meshName = mesh.subString(0, mesh.length() - 4);
		//std::cout << mesh.ptr() << std::endl;
		if (mesh.length())
			loadTetMesh(mesh.ptr());
	} else if (w.mId == "save_current") {
		mGrootHighMesh.saveObj(_meshName + ".mesh", false, false);
		printf("saved to %s.mesh\n", _meshName.ptr(0));
	} else if (w.mId == "Option") {
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

		printf("reset attachments\n");
		// todo :  compare버튼을 누르면
		// 2. mGrootHighMesh.getVertex()와 mFeaturePoints->getVertex의 오차 비교
		/*
        while (!p_cma->testForTermination().empty()) {
            p_cma->samplePopulation();
            for (int i = 0; i < p_cma->numPopulation(); i++) {

            }
            p_cma->update();
        }
		*/
	} else if (w.mId == "tet_to_obj") {
		tetToObj();
	} else if (w.mId == "init") {
		initialize();
	} else if (w.mId == "slider1") {
		printf("%f\n", w.slider()->value());
	}
}


OBJloader::Mesh *TestWin::UpdateMesh(OBJloader::Mesh *source) {
	auto mv = VisionProject::GetInstance()->GetMotionVector();

	for (int i = 0; i < mv.size(); ++i) {
		source->getVertex(mv[i].index) = vector3(mv[i].x, mv[i].y, mv[i].z);
	}

	return source;
}


void TestWin::clearSelection() {

	if (mSelectedVertices.size())
		mSelectedVertices.clearAll();
	if (mSelectedEdges.size())
		mSelectedEdges.clearAll();
}

void TestWin::drawSelection() {
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


void TestWin::setLinearConstraint() {
	for (int i = 0; i < mMesh.getNumTetra(); i++) {
		int musclestiffness = 40000000;
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

void TestWin::loadTetMesh(const char *mesh) {
	bool bInit = true;
	//std::cout << "Load TetMesh" << std::endl;

	if (!mNode) {
		//mNode.reset(RE::createSceneNode(RE::generateUniqueName()), (void(*)(Ogre::SceneNode* node))RE::removeEntity);
	}
	mMesh.loadMesh(mesh);

	mSoftWorld.addBody("testmesh", mMesh, 30);

	setLinearConstraint();


	redraw();
	isLoadTet = true;

	Ogre::SceneNode *pBg = RE::renderer().viewport().mScene->getSceneNode("BackgroundNode");
	pBg->setVisible(false);
}
void TestWin::tetToObj()
{
	//std::cout << "Tet to obj" << std::endl;
	//mMeshDeformer = createHarmonicMeshDeformer();
	mMeshDeformer = createMeanValueMeshDeformer();
	tet_to_obj = tetraToObjfortestwin(mMesh, _meshName + ".face", getNumSurfaceVertex());  //"../mesh/iamgroot101_surface.1.rearranged.face"
	//mGrootHighMesh.loadObj("../mesh/Deformed_groot/iamgroot9k.obj"); // smaller mesh.
	mGrootHighMesh.loadObj(_meshName + ".obj"); // larger mesh.
	if (1) {
		// slightly scale down
		//auto center = mGrootHighMesh.calcMeshCenter(); //?? 메시 센터가 이상하게 계산되는듯.
		auto center = vector3(0,0,0);
		matrix4 m;
		m.identity();
		m.leftMultTranslation(-center);
		m.leftMultScaling(0.99, 0.99, 0.99); // taesoo scale
		m.leftMultTranslation(center);
		mGrootHighMesh.transform(m);
	}

	// mvn is fast enough.
	mMeshDeformer->calculateCorrespondence(*tet_to_obj, mGrootHighMesh);

	//std::cout << "Size Of Vertex" << std::endl;
	//std::cout << tet_to_obj->numVertex() << std::endl;
	//std::cout << mGrootHighMesh.numVertex() << std::endl;


	//for test tetra obj
	//RE::ogreRootSceneNode()->detachObject(line);
	//RE::ogreRootSceneNode()->detachObject(ano_line);
	OBJloader::MeshToEntity::Option o;
	o.useTexCoord = false;
	o.useNormal = false;
	o.buildEdgeList = false;
	if(drawTetMesh)
	{
	//tet_to_obj->saveObj(_meshName + ".surface.obj", false, false);
		mpTetraObjViewer = new OBJloader::MeshToEntity(*tet_to_obj, RE::generateUniqueName(), o);
		mpTetraObjEntity = mpTetraObjViewer->createEntity("tettoobj", "red_transparent"); // defined in color.material
		mpTetraObjEntity->setCastShadows(false);
//		RE::ogreRootSceneNode()->attachObject(mpTetraObjEntity);
	}
	mpSurfaceObjViewer = new OBJloader::MeshToEntity(mGrootHighMesh, RE::generateUniqueName(), o);
	//mpTetraObjEntity = mpTetraObjViewer->createEntity("tettoobj", "lambert6");
	//mpSurfaceObjEntity = mpSurfaceObjViewer->createEntity("surfaceobj", "lambert6");
	mpSurfaceObjEntity = mpSurfaceObjViewer->createEntity("surfaceobj", "green_transparent");
	mpSurfaceObjEntity->setCastShadows(false);
	RE::ogreRootSceneNode()->attachObject(mpSurfaceObjEntity);

}

void TestWin::NanCheck(OBJloader::Mesh *target) {
    int count = 0;
    for(int i =0; i < target->numVertex(); ++i) {
            if(target->getVertex(i).x !=target->getVertex(i).x) {
                //std::cout << "Vertex number " << i << " is nan | Total Nan: " << ++count << std::endl;
            }
            else if(target->getVertex(i).y != target->getVertex(i).y) {
                //std::cout << "Vertex number " << i << " is nan | Total Nan: " << ++count << std::endl;
            }
            else if(target->getVertex(i).z != target->getVertex(i).z) {
                //std::cout << "Vertex number " << i << " is nan | Total Nan: " << ++count << std::endl;
            }
    }
}

void TestWin::ChangeError(double pcm, double rms) {
	char pcmString[32];
	char rmsString[32];
	sprintf(pcmString, "PCM Error : %.2lf mm", pcm);
	sprintf(rmsString, "Avg. Distance : %.2lf mm", rms);
	_pcmWidget->copy_label(pcmString);
	_rmsWidget->copy_label(rmsString);
	updateLayout();
}

// function 2
double* estimateSurfaceVerticesFromCurBarycentrics()
{
	// without soft-body simulation
	vector3N featurepoints;
	TestWin* t=g_pTestWinSingleton;
	featurepoints=*g_pTestWinSingleton->mFeaturePoints;
//	TestWin::updateAllFeaturePointsFromCurBarycentrics(&featurepoints, t->mFeaturePoints_originalShape);
	setVertices(t->m_featureMesh,featurepoints);

	//  feature points -> tetrahedron  (KNN interpolation, 가까운점에 큰 웨이트)
	auto& m_deformer=*t->mControlPointsInFeatureMesh;
	m_deformer.transfer(t->m_featureMesh, 1.0);
	// -> 결과가 t->m_vertices로 들어감.  (mControlPointsInFeatureMesh생성되는 부분 참고)

	ASSERT(t->tet_to_obj);

	int surface_vtx_num = t->getNumSurfaceVertex(); // -.-
	auto& tet_to_obj=t->tet_to_obj;
	// 결과를 t->tet_to_obj라는 메시에 저장.
	for (int i = 0; i < surface_vtx_num; i++) {
		tet_to_obj->getVertex(i) = t->m_vertices(i);
	}

	// mMeshDeformer 를 써서 highresolution mesh로 재변환.(MeanValue weights)
	static vector3N sv;
	((LinearMeshDeformer *)(t->mMeshDeformer))->transferTo(*tet_to_obj, sv);
	return &sv[0].x;
}
