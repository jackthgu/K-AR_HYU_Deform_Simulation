//
// Created by jin on 19. 2. 20.
//

#ifndef FLEXIBLEBODY_REALSENSE3_H
#define FLEXIBLEBODY_REALSENSE3_H

#include <vector>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

class VisionProject {
public:
	void InitializeGL();

	bool RenderingLoop();

	void TerminateGL();

	void KeyInput(const char *action, int key);

	static VisionProject *GetInstance() {
		if (mInstance) {
			return mInstance;
		}
		mInstance = new VisionProject();
		return mInstance;
	}

/* Data structure for motion vector */
	class MotionVector3D {
	public:
		int index;
		double x, y, z;
	};

	std::vector<MotionVector3D> GetMotionVector() {
		return mMotionVectors;
	}
/* Data structure for motion vector */

private:
	static VisionProject *mInstance;
	std::vector<MotionVector3D> mMotionVectors;

	VisionProject() = default;

	~VisionProject() = default;

	void PerspectiveGL(GLdouble fovY, GLdouble aspect, GLdouble zNear, GLdouble zFar);

	void InitRealsense();

	void Idle();

	void ReadSavedFrame();

	void ReadSavedResults();

	void Render();

	void PostProcess();

	void SaveFrameAndResults();

	void GetMarkerPose(float scale);
};

#endif //FLEXIBLEBODY_REALSENSE3_H
