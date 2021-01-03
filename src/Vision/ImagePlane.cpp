#include <GL/glew.h>
#include "ImagePlane.h"
#include <math.h>
#include <opencv2/opencv.hpp>

int nextPowerOfTwo(int x);

ImagePlane::~ImagePlane(void) {
	glDeleteTextures(1, &texObj);
	delete[] mImagePlaneVertices;
	delete[] mImagePlaneTexcoords;
	delete[] mImagePlaneIndices;
	delete[] mInitialRect;
	delete[] mInitialRegionRest;
	delete[] mInitialRegionRestIndices;
}


ImagePlane::ImagePlane(int width, int height)
		: bShowInitialRegion(true) {
	mInitialRectLeft = -0.3f;
	mInitialRectRight = 0.3f;
	mInitialRectBottom = -0.5f;
	mInitialRectTop = 0.5f;

	mFrameWidth = width;
	mFrameHeight = height;
	mTexWidth = width;//nextPowerOfTwo(width);
	mTexHeight = height;//nextPowerOfTwo(height);
	float xRatio = ((float) mFrameWidth) / mTexWidth;
	float yRatio = ((float) mFrameHeight) / mTexHeight;
	mImagePlaneVertices = new float[12];
	mImagePlaneVertices[0] = mImagePlaneVertices[1] = mImagePlaneVertices[2] = mImagePlaneVertices[4] =
	mImagePlaneVertices[5] = mImagePlaneVertices[8] = mImagePlaneVertices[9] = mImagePlaneVertices[11] = -1.0f;
	mImagePlaneVertices[3] = mImagePlaneVertices[6] = mImagePlaneVertices[7] = mImagePlaneVertices[10] = 1.0f;

	mImagePlaneTexcoords = new float[8];
	mImagePlaneTexcoords[0] = mImagePlaneTexcoords[5] = mImagePlaneTexcoords[6] = mImagePlaneTexcoords[7] = 0.0f;
	mImagePlaneTexcoords[1] = mImagePlaneTexcoords[3] = yRatio;
	mImagePlaneTexcoords[2] = mImagePlaneTexcoords[4] = xRatio;

	mImagePlaneIndices = new unsigned char[6];
	mImagePlaneIndices[0] = mImagePlaneIndices[5] = 0;
	mImagePlaneIndices[1] = 1;
	mImagePlaneIndices[2] = mImagePlaneIndices[3] = 2;
	mImagePlaneIndices[4] = 3;

	glEnable(GL_TEXTURE_2D);
	glGenTextures(1, &texObj);
	glBindTexture(GL_TEXTURE_2D, texObj);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, mTexWidth, mTexHeight, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, 0);

	mInitialRect = new float[12];
	mInitialRect[0] = mInitialRect[9] = mInitialRectLeft;
	mInitialRect[1] = mInitialRect[4] = mInitialRectBottom;
	mInitialRect[2] = mInitialRect[5] = mInitialRect[8] = mInitialRect[11] = 1.0f;
	mInitialRect[3] = mInitialRect[6] = mInitialRectRight;
	mInitialRect[7] = mInitialRect[10] = mInitialRectTop;

	mInitialRegionRest = new float[48];
	mInitialRegionRest[0] = mInitialRegionRest[1] = mInitialRegionRest[2] = mInitialRegionRest[4] =
	mInitialRegionRest[5] = mInitialRegionRest[8] = mInitialRegionRest[9] = mInitialRegionRest[11] =
	mInitialRegionRest[12] = mInitialRegionRest[14] = mInitialRegionRest[17] = mInitialRegionRest[20] =
	mInitialRegionRest[21] = mInitialRegionRest[23] = mInitialRegionRest[24] = mInitialRegionRest[26] =
	mInitialRegionRest[29] = mInitialRegionRest[32] = mInitialRegionRest[33] = mInitialRegionRest[35] =
	mInitialRegionRest[38] = mInitialRegionRest[41] = mInitialRegionRest[44] = mInitialRegionRest[47] = -1.0f;
	mInitialRegionRest[3] = mInitialRegionRest[6] = mInitialRegionRest[15] = mInitialRegionRest[18] =
	mInitialRegionRest[19] = mInitialRegionRest[22] = mInitialRegionRest[39] = mInitialRegionRest[42] = 1.0f;

	mInitialRegionRest[7] = mInitialRegionRest[10] = mInitialRegionRest[25] = mInitialRegionRest[28] =
	mInitialRegionRest[37] = mInitialRegionRest[40] = mInitialRectBottom;
	mInitialRegionRest[13] = mInitialRegionRest[16] = mInitialRegionRest[31] = mInitialRegionRest[34] =
	mInitialRegionRest[43] = mInitialRegionRest[46] = mInitialRectTop;
	mInitialRegionRest[27] = mInitialRegionRest[30] = mInitialRectLeft;
	mInitialRegionRest[36] = mInitialRegionRest[45] = mInitialRectRight;
	mInitialRegionRestIndices = new unsigned char[24];
	mInitialRegionRestIndices[0] = mInitialRegionRestIndices[5] = 0;
	mInitialRegionRestIndices[1] = 1;
	mInitialRegionRestIndices[2] = mInitialRegionRestIndices[3] = 2;
	mInitialRegionRestIndices[4] = 3;
	mInitialRegionRestIndices[6] = mInitialRegionRestIndices[11] = 4;
	mInitialRegionRestIndices[7] = 5;
	mInitialRegionRestIndices[8] = mInitialRegionRestIndices[9] = 6;
	mInitialRegionRestIndices[10] = 7;
	mInitialRegionRestIndices[12] = mInitialRegionRestIndices[17] = 8;
	mInitialRegionRestIndices[13] = 9;
	mInitialRegionRestIndices[14] = mInitialRegionRestIndices[15] = 10;
	mInitialRegionRestIndices[16] = 11;
	mInitialRegionRestIndices[18] = mInitialRegionRestIndices[23] = 12;
	mInitialRegionRestIndices[19] = 13;
	mInitialRegionRestIndices[20] = mInitialRegionRestIndices[21] = 14;
	mInitialRegionRestIndices[22] = 15;

}

int nextPowerOfTwo(int x) {
	double val = (double) x;
	return (int) pow(2.0, ceil(log(val) / log(2.0)));
}

void ImagePlane::SetImage(void *frameData) {
	glBindTexture(GL_TEXTURE_2D, texObj);
	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, mFrameWidth, mFrameHeight, GL_BGR_EXT, GL_UNSIGNED_BYTE, frameData);
}

void ImagePlane::Draw() {
	glPushAttrib(GL_ALL_ATTRIB_BITS);
	glPushClientAttrib(GL_CLIENT_ALL_ATTRIB_BITS);

	glMatrixMode(GL_COLOR);
	glPushMatrix();
	glLoadIdentity();

	glMatrixMode(GL_TEXTURE);
	glPushMatrix();
	glLoadIdentity();

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();

	glEnable(GL_TEXTURE_2D);
	//glDisable((GL_TEXTURE_2D));
	glBindTexture(GL_TEXTURE_2D, texObj);
//	glVertexPointer(3, GL_FLOAT, 0, mImagePlaneVertices);
//	glTexCoordPointer(2, GL_FLOAT, 0, mImagePlaneTexcoords);
//	glEnableClientState(GL_VERTEX_ARRAY);
//	glEnableClientState(GL_TEXTURE_COORD_ARRAY);

	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
	glBegin(GL_QUADS);
	glTexCoord2f(0, 1);
	glVertex2f(-1, -1);
	glTexCoord2f(1, 1);
	glVertex2f(1, -1);
	glTexCoord2f(1, 0);
	glVertex2f(1, 1);
	glTexCoord2f(0, 0);
	glVertex2f(-1, 1);
	glEnd();
//	glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_BYTE, mImagePlaneIndices);
	glDisable(GL_TEXTURE_2D);
//	glDisableClientState(GL_TEXTURE_COORD_ARRAY);
//	glDisableClientState(GL_VERTEX_ARRAY);

	glMatrixMode(GL_COLOR);
	glPopMatrix();
	glMatrixMode(GL_TEXTURE);
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();

	glPopClientAttrib();
	glPopAttrib();

}

void ImagePlane::Draw2() {

	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, texObj);

	//draw  시점확인!!
	float projArr[16], mvArr[16];
	glGetFloatv(GL_PROJECTION_MATRIX, projArr);
	glGetFloatv(GL_MODELVIEW_MATRIX, mvArr);

	int vpArr[4];
	glGetIntegerv(GL_VIEWPORT, vpArr);

	cv::Mat _glprojMat(4, 4, CV_32FC1, projArr);
	cv::Mat _glmvMat(4, 4, CV_32FC1, mvArr);

	cv::Mat _projMat;
	cv::Mat _mvMat;

	_glprojMat.copyTo(_projMat);
	_glmvMat.copyTo(_mvMat);
	_projMat = _projMat.t();
	_mvMat = _mvMat.clone().t();

	float left = 0.0f;
	float right = vpArr[2];
	float top = 0.0f;
	float bot = vpArr[3];

	float z = 0.5f;

	cv::Mat quads3D(4, 4, CV_32FC1); // LT, RT, LB, RB
	quads3D.at<float>(0, 0) = (2.0f * left) / (float) vpArr[2] - 1.0f;
	quads3D.at<float>(1, 0) = 1.0f - (2.0f * top) / (float) vpArr[3];
	quads3D.at<float>(2, 0) = 2.0f * z - 1.0f;
	quads3D.at<float>(3, 0) = 1.0f;

	quads3D.at<float>(0, 1) = (2.0f * right) / (float) vpArr[2] - 1.0f;
	quads3D.at<float>(1, 1) = 1.0f - (2.0f * top) / (float) vpArr[3];
	quads3D.at<float>(2, 1) = 2.0f * z - 1.0f;
	quads3D.at<float>(3, 1) = 1.0f;

	quads3D.at<float>(0, 2) = (2.0f * left) / (float) vpArr[2] - 1.0f;
	quads3D.at<float>(1, 2) = 1.0f - (2.0f * bot) / (float) vpArr[3];
	quads3D.at<float>(2, 2) = 2.0f * z - 1.0f;
	quads3D.at<float>(3, 2) = 1.0f;

	quads3D.at<float>(0, 3) = (2.0f * right) / (float) vpArr[2] - 1.0f;
	quads3D.at<float>(1, 3) = 1.0f - (2.0f * bot) / (float) vpArr[3];
	quads3D.at<float>(2, 3) = 2.0f * z - 1.0f;
	quads3D.at<float>(3, 3) = 1.0f;
	// NDC Here

	quads3D = _projMat.inv() * quads3D;
	for(int i = 0; i < 4; i++)
	{
		quads3D.at<float>(0, i) /= quads3D.at<float>(3, i);
		quads3D.at<float>(1, i) /= quads3D.at<float>(3, i);
		quads3D.at<float>(2, i) /= quads3D.at<float>(3, i);
		quads3D.at<float>(3, i) /= quads3D.at<float>(3, i);
	}
	//

	quads3D = _mvMat.inv() * quads3D;
	for(int i = 0; i < 4; i++)
	{
		quads3D.at<float>(0, i) /= quads3D.at<float>(3, i);
		quads3D.at<float>(1, i) /= quads3D.at<float>(3, i);
		quads3D.at<float>(2, i) /= quads3D.at<float>(3, i);
		quads3D.at<float>(3, i) /= quads3D.at<float>(3, i);
	}

	printf("%f %f %f %f\n", quads3D.at<float>(2, 0), quads3D.at<float>(2, 1), quads3D.at<float>(2, 2), quads3D.at<float>(2, 3));
	{//taesoo
		// LT, RT, LB, RB
		glBegin(GL_QUADS);

		// Bottom Face.  Red, 75% opaque, magnified texture

		glNormal3f(0.0f, -1.0f, 0.0f); // Needed for lighting
		glColor4f(1.0, 1.0, 1.0, 1.0); // Basic polygon color

		double xyRatio = 1280.0f / 720.0f;
		double tcX = 1.3/xyRatio;
		double tcY = 1.3/xyRatio;
		//glTexCoord2f(0.0f, 0.0f);
		//glVertex3f(-1.0f, 1.0f, 0.5f); // LT
		glVertex3f(quads3D.at<float>(0, 0), quads3D.at<float>(1, 0), quads3D.at<float>(2, 0));
		//glVertex3f(quads3D.at<float>(0, 0), quads3D.at<float>(1, 0), 9999.0f);
		//glTexCoord2f(0.0f, 1.0);
		//glVertex3f(-1.0f, -1.0f, 0.5f); // LB
		glVertex3f(quads3D.at<float>(0, 2), quads3D.at<float>(1, 2), quads3D.at<float>(2, 0));
		//glVertex3f(quads3D.at<float>(0, 2), quads3D.at<float>(1, 2), 9999.0f);
		//glTexCoord2f(1.0f, 1.0f);
		//glVertex3f(1.0f, -1.0f, 0.5f); // RB
		glVertex3f(quads3D.at<float>(0, 3), quads3D.at<float>(1, 3), quads3D.at<float>(2, 0));
		//glVertex3f(quads3D.at<float>(0, 3), quads3D.at<float>(1, 3), 9999.0f);
		//glTexCoord2f(1.0f, 0.0f);
		//glVertex3f(1.0f, 1.0f, 0.5f); // RT
		glVertex3f(quads3D.at<float>(0, 1), quads3D.at<float>(1, 1), quads3D.at<float>(2, 0));
		//glVertex3f(quads3D.at<float>(0, 1), quads3D.at<float>(1, 1), 9999.0f);
		glEnd();
	}
	glDisable(GL_TEXTURE_2D);
}
