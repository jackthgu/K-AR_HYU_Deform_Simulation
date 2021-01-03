//
// Created by jin on 18. 2. 21.
//


#include "stdafx.h"
#include "MainLib/OgreFltk/VRMLloader.h"
#include "MainLib/OgreFltk/FlLayout.h"
#include "MainLib/OgreFltk/MotionPanel.h"
#include <MainLib/OgreFltk/FltkRenderer.h>
#include <MainLib/OgreFltk/FlChoice.h>
#include <Ogre.h>
#include "tetgen/src/ShapeReconstructor.h"

// TODO: Actually constant strings needs to be in constants.h
#define TESTWIN_POINT_BUTTON "Point"
#define TESTWIN_BACK_MESH_BUTTON "BackMesh"
#define TESTWIN_LINE_BUTTON "Line"
#define TESTWIN_LOADOBJ_BUTTON "Load OBJ"
#define TESTWIN_VOLUME_BUTTON "Volume"
#define TESTWIN_CUT_BUTTON "Cut"
#define TESTWIN_LOADRGBD_BUTTON "RGBD"
#define TESTWIN_SURFACE_BUTTON "Surface"
#define TESTWIN_TETRAHEDRON_BUTTON "Tet"
#define TESTWIN_SCENE_NODE "TestCppNode"

FlLayout *createTestWin(int x, int y, int w, int h);

/**
 * FlLayout class which is creating VTK Objects as Ogre Entity.
 */
class TetgenWin : public FlLayout {
public:
	Ogre::Entity *mEntity; // The entity that converted from VTK.
	boost::shared_ptr<Ogre::SceneNode> mNode; // Scene Node.
	ShapeReconstructor mSR; // ShapeReconstructor

	/**
	 * The constructor of TestWin
	 * @param x Position of layout
	 * @param y Position of layout
	 * @param w Width
	 * @param h Height
	 */
	TetgenWin(int x, int y, int w, int h) : FlLayout(0, 0, w, h) {
		create("Button", TESTWIN_LOADOBJ_BUTTON, TESTWIN_LOADOBJ_BUTTON);
		create("Button", TESTWIN_POINT_BUTTON, TESTWIN_POINT_BUTTON);
		create("Button", TESTWIN_BACK_MESH_BUTTON, TESTWIN_BACK_MESH_BUTTON);
		create("Button", TESTWIN_LINE_BUTTON, TESTWIN_LINE_BUTTON);
		create("Button", TESTWIN_VOLUME_BUTTON, TESTWIN_VOLUME_BUTTON);
		create("Button", TESTWIN_LOADRGBD_BUTTON, TESTWIN_LOADRGBD_BUTTON);
		create("Button", TESTWIN_SURFACE_BUTTON, TESTWIN_SURFACE_BUTTON);
		create("Button", TESTWIN_TETRAHEDRON_BUTTON, TESTWIN_TETRAHEDRON_BUTTON);
		//create("Button", TESTWIN_CUT_BUTTON, TESTWIN_CUT_BUTTON);
		updateLayout();
	};

	/**
	 * Callback Method
	 * @param w
	 * @param pWidget
	 * @param userData
	 */
	void onCallback(FlLayout::Widget const &w, Fl_Widget *pWidget, int userData) {
		if (w.mId == TESTWIN_LOADOBJ_BUTTON) {
			LoadOBJ();
		} else if (w.mId == TESTWIN_POINT_BUTTON) {
			GeneratePoints();
		} else if (w.mId == TESTWIN_BACK_MESH_BUTTON) {
			GenerateBackMesh();
		} else if (w.mId == TESTWIN_LINE_BUTTON) {
			GenerateLines();
		} else if (w.mId == TESTWIN_VOLUME_BUTTON) {
			GenerateVolume();
		} else if (w.mId == TESTWIN_CUT_BUTTON) {
			// Cut
		}else if (w.mId == TESTWIN_LOADRGBD_BUTTON) {
			// Load RGBD Data and show mesh
			// GeneratePoints
			// GenerateLines
		}else if (w.mId == TESTWIN_SURFACE_BUTTON) {
			// Load full model and show mesh
			//
		}else if (w.mId == TESTWIN_TETRAHEDRON_BUTTON) {
			// Load Tet and show
		}
	}

	void LoadOBJ() {
		if (!mNode) {
			mNode.reset(RE::createSceneNode(TESTWIN_SCENE_NODE), (void (*)(Ogre::SceneNode *node)) RE::removeEntity);
		}
		mNode->attachObject(mSR.LoadOBJ());
	}

	/**
	 * Create entity from VTK object
	 */
	void GeneratePoints() {
		if (!mNode) {
			mNode.reset(RE::createSceneNode(TESTWIN_SCENE_NODE), (void (*)(Ogre::SceneNode *node)) RE::removeEntity);
		}
		mNode->attachObject(mSR.GeneratePoints());
	}

	void GenerateBackMesh() {
		mNode->detachAllObjects();
		mNode->attachObject(mSR.GenerateBackMesh());
	}

	void GenerateLines() {
		mNode->detachAllObjects();
		mNode->attachObject(mSR.GenerateLines());
	}

	void GenerateVolume() {
		mNode->detachAllObjects();
		mNode->attachObject(mSR.GenerateVolume());
	}

	/**
	 * Prepare for Next Button
	 * @param id Button id
	 */
	void NextButton(const char *id) {
		removeWidgets(0);
		create("Button", id, id);
		updateLayout();
	}
};

/**
 * Global function for OgreFltk.cpp
 * @param x Position of layout
 * @param y Position of layout
 * @param w Width
 * @param h Height
 * @return TestWin class
 */
FlLayout *createTetgenWin(int x, int y, int w, int h, MotionPanel& mp,FltkRenderer& renderer) {
	return new TetgenWin(x, y, w, h);
}
