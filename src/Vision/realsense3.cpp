#if defined(_WIN32) || defined(_WIN32) || defined(__WIN32__) || defined(_WIN64)
#define _CRT_SECURE_NO_WARNINGS
#ifdef _DEBUG
#pragma comment(lib, "opencv_world340d.lib")
#pragma comment(lib, "glew32d.lib")
#pragma comment(lib, "libglew32d.lib")
#else
#pragma comment(lib, "opencv_world340.lib")
#pragma comment(lib, "glew32.lib")
#endif
#pragma comment(lib, "glfw3.lib")
#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "realsense2.lib")
#include <windows.h>
#endif

#include <stdio.h>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#if CV_VERSION_MAJOR>=4 && CV_VERSION_MINOR>=2
// taesoo added the following two lines for legacy codes
#include <opencv2/imgcodecs/legacy/constants_c.h>
#include <opencv2/imgproc/types_c.h>
#endif

#ifndef NO_REALSENSE
#include <librealsense2/rs.hpp>
#endif

#include "Vision/ImagePlane.h"
#include "Vision/ObjModel.h"
#include "Vision/SettingLoader.h"
#include "Vision/TrackerUtils.h"

#include "realsense3.h"

obj::ObjModel *model;
float *glProjectionMat;
float reCalcFx;
float reCalcFy;
float reCalcCx;
float reCalcCy;
Settings g_settings;

int FRAME_WIDTH;
int FRAME_HEIGHT;

#ifndef NO_REALSENSE
/* realsense variables */
rs2::context rsContext;
rs2::pipeline rsPipeline;
rs2::pipeline_profile rsPipelineProfile;
rs2::frameset rsAlignedFrameset;
rs2::config rsConfig;
rs2::frame rsColorFrame;
rs2::frame rsDepthFrame;
/* realsense variables */
#endif

/* frame variables */
cv::Mat colorMat;
cv::Mat depthMat;
cv::Mat silhouetteMat;
ImagePlane* pImagePlane;
float* depthBuffer;
/* frame variables */

/* control variables */
int baryIdx = 0;
bool reInit = false; // F1
bool drawWireFrame = true; // F2
bool drawRays = false; // F3
bool drawFeatures = true; // F4
bool drawMotionVecs = false; // F5
bool drawBarycentricRays = false; // F6
bool drawIntersectFaces = false; // F7
bool drawOnlyOne = false; // F8
bool displayColor = true;
bool saveCurFrame = false; // F9
bool pause = false;

int g_frameCnt = 0;
/* control variables */

/* initial model-view variables */
float g_translateScale = 1.0f;
float g_rotationScale = 1.0f;
//int g_rightKeyCnt = -40;
//int g_upKeyCnt = 21 ;
//int g_forwardKeyCnt = 239;
//int g_xRotKeyCnt = 36;
//int g_yRotKeyCnt = -194;
//int g_zRotKeyCnt = -65;

int g_rightKeyCnt = 0;
int g_upKeyCnt = 0;
int g_forwardKeyCnt = 0;
int g_xRotKeyCnt = 0;
int g_yRotKeyCnt = 0;
int g_zRotKeyCnt = 0;
/* initial model-view variables */

/* tracking variables */
cv::Mat modelview;
cv::Mat intrinsic;
cv::Mat protectionMask = cv::Mat(1, 1, CV_8U);
std::vector<cv::Point2f> refCorners;
std::vector<cv::Point2f> curCorners;
std::vector<cv::Point3f> ref3DCorners;
std::vector<cv::Point3f> cur3DCorners;
std::vector<cv::Point3f> objPts;
std::vector<space::Line> refRays;
std::vector<space::Barycentric> refBarycentrics;
std::vector<space::Barycentric> curBarycentrics;
std::vector<space::Barycentric> markerBarycentrics;
std::vector<float> refZDRatio;
float refZDRatioMedian;
/* tracking variables */

/* record & playback variables */
std::string g_extension;
/* record & playback variables */


VisionProject* VisionProject::mInstance = NULL;

void KeyCallback(GLFWwindow *window, int key, int scancode, int action, int mods) {
	if (action == GLFW_PRESS || action == GLFW_REPEAT) {
		switch (key) {
			case GLFW_KEY_SPACE:
				//oneshot = true;
				break;
			case GLFW_KEY_ENTER:
				printf("%d, %d, %d / %d, %d, %d\n", g_rightKeyCnt, g_upKeyCnt, g_forwardKeyCnt, g_xRotKeyCnt, g_yRotKeyCnt, g_zRotKeyCnt);
				break;
			case GLFW_KEY_TAB:
				baryIdx = (baryIdx + 1) % refBarycentrics.size();
				break;
			case GLFW_KEY_F1:
				reInit = true;
				baryIdx = 0;
				g_frameCnt =0;
				break;
			case GLFW_KEY_F2:
				drawWireFrame = !drawWireFrame;
				break;
			case GLFW_KEY_F3:
				drawRays = !drawRays;
				break;
			case GLFW_KEY_F4:
				drawFeatures = !drawFeatures;
				break;
			case GLFW_KEY_F5:
				drawMotionVecs = !drawMotionVecs;
				break;
			case GLFW_KEY_F6:
				drawBarycentricRays = !drawBarycentricRays;
				break;
			case GLFW_KEY_F7:
				drawIntersectFaces = !drawIntersectFaces;
				break;
			case GLFW_KEY_F8:
				drawOnlyOne = !drawOnlyOne;
				break;
			case GLFW_KEY_F9:
				saveCurFrame = true;
				break;
			case GLFW_KEY_ESCAPE:
				glfwSetWindowShouldClose(window, 1);
				break;
			case GLFW_KEY_UP:
				g_upKeyCnt++;
				break;
			case GLFW_KEY_DOWN:
				g_upKeyCnt--;
				break;
			case GLFW_KEY_RIGHT:
				g_rightKeyCnt++;
				break;
			case GLFW_KEY_LEFT:
				g_rightKeyCnt--;
				break;
			case GLFW_KEY_W:
				g_forwardKeyCnt++;
				break;
			case GLFW_KEY_S:
				g_forwardKeyCnt--;
				break;
			case GLFW_KEY_A:
				g_yRotKeyCnt--;
				break;
			case GLFW_KEY_D:
				g_yRotKeyCnt++;
				break;
			case GLFW_KEY_Q:
				g_xRotKeyCnt++;
				break;
			case GLFW_KEY_E:
				g_xRotKeyCnt--;
				break;
			case GLFW_KEY_Z:
				g_zRotKeyCnt++;
				break;
			case GLFW_KEY_C:
				g_zRotKeyCnt--;
				break;
			case GLFW_KEY_I:
				g_upKeyCnt++;
				break;
			case GLFW_KEY_K:
				g_upKeyCnt--;
				break;
			case GLFW_KEY_J:
				g_rightKeyCnt--;
				break;
			case GLFW_KEY_L:
				g_rightKeyCnt++;
				break;
			case GLFW_KEY_0:
				g_forwardKeyCnt = 0;
				g_upKeyCnt = 0;
				g_rightKeyCnt = 0;
				g_xRotKeyCnt = 0;
				g_yRotKeyCnt = 0;
				g_zRotKeyCnt = 0;
				break;
			default:
				break;
		}
	}
}

void VisionProject::KeyInput(const char* action, int key){
	static int a = 0;
	if (strcmp(action, "KEYDOWN") == 0) {
		switch (key) {
			case 32: // space
				pause = !pause;
				break;
			case 65293: // enter
				//printf("%d, %d, %d / %d, %d, %d\n", g_rightKeyCnt, g_upKeyCnt, g_forwardKeyCnt, g_xRotKeyCnt, g_yRotKeyCnt, g_zRotKeyCnt);
				printf("save current status\n");
				{
					float mvArr[16];
					glGetFloatv(GL_MODELVIEW_MATRIX, mvArr);
					cv::Mat _tempPose = cv::Mat(4, 4, CV_32FC1, mvArr);
					_tempPose = _tempPose.clone().t();
					_tempPose.convertTo(_tempPose, CV_64F);
					//std::cout << _tempPose << std::endl;
					g_settings.SetInitPose(_tempPose);
					g_settings.SaveSetting("Setting.ini");
				}
				break;
			case 65289: // tab
				baryIdx = (baryIdx + 1) % refBarycentrics.size();
				break;
			case 65470: // F1 ~ F9
				reInit = true;
				baryIdx = 0;
				g_frameCnt = 0;
				break;
			case 65471:
				drawWireFrame = !drawWireFrame;
				break;
			case 65472:
				drawRays = !drawRays;
				break;
			case 65473:
				drawFeatures = !drawFeatures;
				break;
			case 65474:
				drawMotionVecs = !drawMotionVecs;
				break;
			case 65475:
				drawBarycentricRays = !drawBarycentricRays;
				break;
			case 65476:
				drawIntersectFaces = !drawIntersectFaces;
				break;
			case 65477:
				drawOnlyOne = !drawOnlyOne;
				break;
			case 65478:
				saveCurFrame = true;
				break;
			case 'o':
				displayColor = !displayColor;
				break;
			case GLFW_KEY_ESCAPE:
//				glfwSetWindowShouldClose(window, 1);
				break;
			case 65362:
			case 't':
				g_upKeyCnt++;
				break;
			case 65364:
			case 'g':
				g_upKeyCnt--;
				break;
			case 65363:
			case 'h':
				g_rightKeyCnt++;
				break;
			case 65361:
			case 'f':
				g_rightKeyCnt--;
				break;
			case 'w':
				g_forwardKeyCnt++;
				break;
			case 's':
				g_forwardKeyCnt--;
				break;
			case 'a':
				g_yRotKeyCnt--;
				break;
			case 'd':
				g_yRotKeyCnt++;
				break;
			case 'q':
				g_xRotKeyCnt++;
				break;
			case 'e':
				g_xRotKeyCnt--;
				break;
			case 'z':
				g_zRotKeyCnt++;
				break;
			case 'c':
				g_zRotKeyCnt--;
				break;
			case '0':
				g_forwardKeyCnt = 0;
				g_upKeyCnt = 0;
				g_rightKeyCnt = 0;
				g_xRotKeyCnt = 0;
				g_yRotKeyCnt = 0;
				g_zRotKeyCnt = 0;
				break;
			case 'p':
			{
				std::stringstream ss;
				ss << "Saved/!" << a;
				cv::imwrite(ss.str() + "sc." + g_extension, colorMat);
				cv::Mat depthMat8UC3 = cv::Mat::zeros(depthMat.rows, depthMat.cols, CV_8UC3);
				for(int i = 0; i < depthMat8UC3.rows; i++)
				{
					for(int j = 0; j < depthMat8UC3.cols; j++)
					{
						depthMat8UC3.at<cv::Vec3b>(i, j)[0] = depthMat.at<short>(i, j) / 256;
						depthMat8UC3.at<cv::Vec3b>(i, j)[1] = depthMat.at<short>(i, j) % 256;
					}
				}
				cv::imwrite(ss.str() + "sd." + g_extension, depthMat8UC3);
				a ++;
			}
			default:
				break;
		}
	}
}

void ErrorCallback(int error, const char *description) {
	std::cout << description << std::endl;
}

void MouseCallback(GLFWwindow *window, int button, int action, int mods) {
	if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
	}
	return;
}

void VisionProject::PerspectiveGL(GLdouble fovY, GLdouble aspect, GLdouble zNear, GLdouble zFar) {
	const GLdouble pi = 3.1415926535897932384626433832795;
	GLdouble fW, fH;

	//fH = tan( (fovY / 2) / 180 * pi ) * zNear;
	fH = tan(fovY / 360 * pi) * zNear;
	fW = fH * aspect;

	glFrustum(-fW, fW, -fH, fH, zNear, zFar);
}

float *GetGLProjectionMatrixFromRealIntrinsic(float fx, float fy, float cx, float cy, float _far, float _near, int width, int height) {
	float *glProjectionMat = new float[16];
	float zn = _near;
	float zf = _far;
	float scaleX = (1.0f / fx) * zn;
	float scaleY = (1.0f / fy) * zn;
	float l = -scaleX * cx;
	float r = scaleX * (width - cx);
	float b = -(height - cy) * scaleY;
	float t = scaleY * (cy);
	memset(glProjectionMat, 0, sizeof(float) * 16);
	glProjectionMat[0] = 2 * zn / (r - l);
	glProjectionMat[5] = 2 * zn / (t - b);
	glProjectionMat[8] = (l + r) / (r - l);
	glProjectionMat[9] = (t + b) / (t - b);
	//glProjectionMat[10] = zf / (zn - zf);
	glProjectionMat[10] = -(zf + zn) / (zf - zn);
	glProjectionMat[11] = -1;
	glProjectionMat[14] = 2 * zn * zf / (zn - zf);

	return glProjectionMat;
}

float *GetGLProjectionMatrixFromPseudoIntrinsic(float fx, float fy, float cx, float cy, float _far, float _near, int width, int height) {
	float *glProjectionMat = new float[16];
	memset(glProjectionMat, 0, sizeof(float) * 16);
	glProjectionMat[0] = fx / cx;
	glProjectionMat[5] = fy / cy;
	glProjectionMat[10] = -(_far + _near) / (_far - _near);
	glProjectionMat[11] = -1;
	glProjectionMat[14] = -2.0f * _far * _near / (_far - _near);
	return glProjectionMat;
}

void VisionProject::InitRealsense() {
#ifndef NO_REALSENSE
	rsConfig.enable_stream(rs2_stream::RS2_STREAM_COLOR, FRAME_WIDTH, FRAME_HEIGHT, rs2_format::RS2_FORMAT_BGR8, 6);
	rsConfig.enable_stream(rs2_stream::RS2_STREAM_DEPTH, FRAME_WIDTH, FRAME_HEIGHT, rs2_format::RS2_FORMAT_Z16, 6);
	rsPipelineProfile = rsPipeline.start(rsConfig);
#endif
}

GLFWwindow *InitGLFW(int ScreenWidth, int ScreenHeight, const char *winName, GLFWerrorfun errorFunc, GLFWmousebuttonfun mouseFunc, GLFWkeyfun keyFunc, cv::Mat intr) {
	GLFWwindow *window = nullptr;
//	glfwSetErrorCallback(errorFunc);
//	if (!glfwInit()) {
//		fprintf(stderr, "Failed to Init.\n");
//		return nullptr;
//	}
//
//	window = glfwCreateWindow(ScreenWidth, ScreenHeight, winName, NULL, NULL);
//	if (!window) {
//		glfwTerminate();
//		fprintf(stderr, "Failed to Create Window.\n");
//		return nullptr;
//	}
//
//	glfwMakeContextCurrent(window);
//	glfwSwapInterval(1);
//
//	glfwSetMouseButtonCallback(window, mouseFunc);
//	glfwSetKeyCallback(window, keyFunc);
	//std::cout <<intr<< std::endl;
	float fx = intr.at<float>(0, 0);
	float fy = intr.at<float>(1, 1);
	float cx = intr.at<float>(0, 2);
	float cy = intr.at<float>(1, 2);
	glProjectionMat = GetGLProjectionMatrixFromPseudoIntrinsic(fx, fy, cx, cy, 5000.0, 0.1, ScreenWidth, ScreenHeight);

	reCalcFx = glProjectionMat[0] * ScreenWidth / 2.0f;
	reCalcFy = glProjectionMat[5] * ScreenHeight / 2.0f;
	reCalcCx = (1.0f - glProjectionMat[8]) * ScreenWidth / 2.0f;
	reCalcCy = (glProjectionMat[9] + 1) * ScreenHeight / 2.0f;
	//std::cout << "Re-Calc. Intr.: " << reCalcFx << "," << reCalcFy << ", " << reCalcCx << "," << reCalcCy <<std::endl;
	return window;
}

void VisionProject::Idle() {
#ifndef NO_REALSENSE
	rs2::frameset _frameset = rsPipeline.wait_for_frames();
	rs2::align _align(rs2_stream::RS2_STREAM_COLOR);
	rs2::hole_filling_filter hff;
	rsAlignedFrameset = _align.process(_frameset);
	int colorWidth, colorHeight, depthWidth, depthHeight;
	if (!rsAlignedFrameset.size()) {
		colorMat = cv::Mat();
		depthMat = cv::Mat();
		return;
	}
	rsColorFrame = rsAlignedFrameset.get_color_frame();
	colorWidth = rsColorFrame.as<rs2::video_frame>().get_width();
	colorHeight = rsColorFrame.as<rs2::video_frame>().get_height();

	rsDepthFrame = rsAlignedFrameset.get_depth_frame();
	//rsDepthFrame = hff.process(rsDepthFrame);
	depthWidth = rsDepthFrame.as<rs2::video_frame>().get_width();
	depthHeight = rsDepthFrame.as<rs2::video_frame>().get_height();

	colorMat = cv::Mat(colorHeight, colorWidth, CV_8UC3, const_cast<void *>(rsColorFrame.get_data()));
	depthMat = cv::Mat(depthHeight, depthWidth, CV_16SC1, const_cast<void *>(rsDepthFrame.get_data()));

#endif
}

void VisionProject::ReadSavedFrame()
{
	std::stringstream imageCntSS;
	imageCntSS <<  "Saved/" << std::setfill('0') << std::setw(5) << g_frameCnt;
	colorMat = cv::imread(imageCntSS.str() + "c." + g_extension, CV_LOAD_IMAGE_COLOR);
	cv::Mat depthMat8UC3 = cv::imread(imageCntSS.str() + "d." + g_extension, CV_LOAD_IMAGE_COLOR);
	//std::cout << colorMat.size() << depthMat8UC3.size() << std::endl;
	depthMat = cv::Mat(depthMat8UC3.rows, depthMat8UC3.cols, CV_16SC1);
	// Convert 24bit image to 16bit image
	for(int i = 0; i < depthMat8UC3.rows; i++)
	{
		for(int j = 0; j < depthMat8UC3.cols; j++)
		{
			depthMat.at<short>(i, j) = depthMat8UC3.at<cv::Vec3b>(i, j)[0] * 256 + depthMat8UC3.at<cv::Vec3b>(i, j)[1];
		}
	}

}

void VisionProject::ReadSavedResults()
{
	std::stringstream fileCntSS;
	fileCntSS <<  "Saved/" << std::setfill('0') << std::setw(5) << g_frameCnt;
	std::ifstream ifs(fileCntSS.str() + "r.csv");
	curBarycentrics.clear();

	if(ifs.is_open())
	{
		std::string line;
		while(std::getline(ifs, line))
		{
			std::stringstream liness(line);
			std::string substr;

			space::Barycentric bary;

			getline(liness, substr, ',');
			bary.fIdx = std::stoi(substr);

			getline(liness, substr, ',');
			bary.idx.x = std::stoi(substr);

			getline(liness, substr, ',');
			bary.idx.y = std::stoi(substr);

			getline(liness, substr, ',');
			bary.idx.z = std::stoi(substr);

			for(int i = 0; i < 3; i++)
			{
				getline(liness, substr, ',');
				bary.v[i].x = std::stof(substr);
				getline(liness, substr, ',');
				bary.v[i].y = std::stof(substr);
				getline(liness, substr, ',');
				bary.v[i].z = std::stof(substr);
			}

			for(int i = 0; i < 3; i++)
			{
				getline(liness, substr, ',');
				bary.coeffs[i] = std::stof(substr);
			}

			getline(liness, substr, ',');
			bary.ray.dir.x = std::stof(substr);
			getline(liness, substr, ',');
			bary.ray.dir.y = std::stof(substr);
			getline(liness, substr, ',');
			bary.ray.dir.z = std::stof(substr);

			getline(liness, substr, ',');
			bary.ray.ori.x = std::stof(substr);
			getline(liness, substr, ',');
			bary.ray.ori.y = std::stof(substr);
			getline(liness, substr, ',');
			bary.ray.ori.z = std::stof(substr);

			getline(liness, substr, ',');
			bary.t = std::stof(substr);
			if(g_frameCnt == 0)
				refBarycentrics.push_back(bary);
			else
				curBarycentrics.push_back(bary);
		}

		if(g_frameCnt == 0)
		{
			ref3DCorners.clear();
			for(int i = 0; i < refBarycentrics.size(); i++)
			{
				space::Point a = refBarycentrics[i].ray.ori + (refBarycentrics[i].ray.dir * refBarycentrics[i].t);
				ref3DCorners.push_back(cv::Point3f(a.x, a.y, a.z));
			}
		}
		else
		{
			cur3DCorners.clear();
			for(int i = 0; i < curBarycentrics.size(); i++)
			{
				space::Point a = curBarycentrics[i].ray.ori + (curBarycentrics[i].ray.dir * refBarycentrics[i].t);
				cur3DCorners.push_back(cv::Point3f(a.x, a.y, a.z));
			}
		}
	}
}

void VisionProject::Render() {
	//glfwMakeContextCurrent(window);
	/*  rendering silhouette before AR scene rendering */
	glDisable(GL_LIGHTING);
	glEnable(GL_DEPTH_TEST);
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_CULL_FACE);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glLoadMatrixf(glProjectionMat);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	//glTranslatef(g_translateScale * g_rightKeyCnt, g_translateScale * g_upKeyCnt, -g_translateScale * g_forwardKeyCnt);

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
	rotMat.at<float>(2, 0) = 2 *(q1 * q3 - q0 * q2); rotMat.at<float>(2, 1)= 2*(q0*q1+q2*q3); rotMat.at<float>(2, 2) = 1-2*(q1*q1 + q2*q2);
	cv::Mat mvMat = cv::Mat::zeros(4, 4, CV_32FC1);
	mvMat(cv::Rect(0, 0, 3, 3)) += rotMat;
	mvMat.at<float>(0, 3) = g_translateScale * g_rightKeyCnt;
	mvMat.at<float>(1, 3) = g_translateScale * g_upKeyCnt;
	mvMat.at<float>(2, 3) = -g_translateScale * g_forwardKeyCnt;
	mvMat.at<float>(3, 3) = 1.0;

	//std::cout << "mvMat" << std::endl;
	//std::cout << mvMat << std::endl;
	//std::cout << "modelview" << std::endl;
	//std::cout << modelview << std::endl;

	mvMat = modelview * mvMat;
	mvMat= mvMat.t();
	//std::cout << "mvMatt" << std::endl;
	//std::cout << mvMat << std::endl;
	glLoadMatrixf((GLfloat*)mvMat.data);

	glPushMatrix();
	model->DrawSilhouette(1.0f, 1.0f, 1.0f);
	glPopMatrix();

	int vpArr[4];
	glGetIntegerv(GL_VIEWPORT, vpArr);
	glReadPixels(vpArr[0], vpArr[1], vpArr[2], vpArr[3], GL_LUMINANCE, GL_UNSIGNED_BYTE, silhouetteMat.data);
	glReadPixels(0, 0, vpArr[2], vpArr[3], GL_DEPTH_COMPONENT, GL_FLOAT, depthBuffer);
	glDisable(GL_DEPTH_TEST);

	cv::flip(silhouetteMat, silhouetteMat, 0);
	cv::Mat stElem(3, 3, CV_8U, cv::Scalar(1));
	cv::erode(silhouetteMat, silhouetteMat, stElem, cv::Point(-1, -1), 5);

	cv::Mat silhouetteBGR = cv::Mat(silhouetteMat.size(), CV_8UC3);
	cv::cvtColor(silhouetteMat, silhouetteBGR, CV_GRAY2BGR);

	glEnable(GL_DEPTH_TEST);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	cv::Mat mappedDepth;
	depthMat.convertTo(mappedDepth, CV_8U, -255.0 / 5000.0, 255.0);
	cv::applyColorMap(mappedDepth, mappedDepth, cv::COLORMAP_JET);
//SetImage함수 에그리 고싶 은mat 넣으 면
#if !defined(NO_REALSENSE) || defined(PLAYBACK_REALSENSE)
	if(displayColor)
		pImagePlane->SetImage((void *) colorMat.data);
	else
		pImagePlane->SetImage((void *) mappedDepth.data);
	// pImagePlane->SetImage((void *) silhouetteBGR.data);
	pImagePlane->Draw();
#endif

	glClear(GL_DEPTH_BUFFER_BIT);
	glEnable(GL_TEXTURE_2D);

	glEnable(GL_CULL_FACE);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	//glLoadMatrixf(ProjectionMatrix);
	//perspectiveGL(60, (double)FRAME_WIDTH / (double)FRAME_HEIGHT, 0.1, 1000);
	glLoadMatrixf(glProjectionMat);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	if(true) {
		glDisable(GL_TEXTURE_2D);
		//glPushMatrix();
		float lineWidth;
		glGetFloatv(GL_LINE_WIDTH, &lineWidth);
		glLineWidth(1.1f);
		glPointSize(5.0f);
		glColor3f(0.0f, 1.0f, 0.0f);
		for(int i =0; i<objPts.size(); i++){
			cv::Point3f p = objPts[i];
			cv::Point3f pp = objPts[(i+1)%objPts.size()];
			glBegin(GL_POINTS);

			glVertex3f(1.0*p.x, 1.0*p.y, 1.0*p.z);
			glEnd();

			glBegin(GL_LINES);
			//glVertex3f(0, 0, -0.20);
			glVertex3f(1.0*p.x, 1.0*p.y, 1.0*p.z);
			glVertex3f(1.0*pp.x, 1.0*pp.y, 1.0*pp.z);
			glEnd();
		}
		//glEnable(GL_CULL_FACE);
	}

	//glTranslatef(g_translateScale * g_rightKeyCnt, g_translateScale * g_upKeyCnt, -g_translateScale * g_forwardKeyCnt);
	glLoadMatrixf((GLfloat*)mvMat.data);
//    q.w() = cy * cp * cr + sy * sp * sr;
//    q.x() = cy * cp * sr - sy * sp * cr;
//    q.y() = sy * cp * sr + cy * sp * cr;
//    q.z() = sy * cp * cr - cy * sp * sr;

	//glRotatef(g_rotationScale * g_zRotKeyCnt, 0.0f, 0.0f, 1.0f);
	//glRotatef(g_rotationScale * g_yRotKeyCnt, 0.0f, 1.0f, 0.0f);
	//glRotatef(g_rotationScale * g_xRotKeyCnt, 1.0f, 0.0f, 0.0f);

	glPushMatrix();
	float lineWidth;
	glGetFloatv(GL_LINE_WIDTH, &lineWidth);
	glLineWidth(0.1f);
//	if (drawWireFrame)
//		//model->DrawWireframe(74.0f / 255.0f, 31.0f / 255.0f, 180.0f / 255.0f);
	//model->DrawWireframe(1.0f, 1.0f, 1.0f);
//	else
//		model->Draw();
	glLineWidth(lineWidth);
	glPopMatrix();
	glDisable(GL_TEXTURE_2D);

	if (drawFeatures) {
		float oldPointSize;
		glGetFloatv(GL_POINT_SIZE, &oldPointSize);
		glPointSize(5.0f);
		glColor3f(1.0f, 0.0f, 0.0f);
		for (int i = 0; i < ref3DCorners.size(); i++) {
			glBegin(GL_POINTS);
			glVertex3f(ref3DCorners[i].x, ref3DCorners[i].y, ref3DCorners[i].z);
			glEnd();
		}

		glColor3f(0.0f, 0.0f, 1.0f);
		for (int i = 0; i < cur3DCorners.size(); i++) {
			glBegin(GL_POINTS);
			glVertex3f(cur3DCorners[i].x, cur3DCorners[i].y, cur3DCorners[i].z);
			glEnd();
		}

		for(int i = 0; i < refBarycentrics.size(); i++)
		{
			bool isMatched = false;
			space::Barycentric refBary = refBarycentrics[i];
			space::Barycentric curBary;
			for(int j = 0; j < curBarycentrics.size(); j++)
			{
				if(curBarycentrics[j].fIdx == refBary.fIdx)
				{
					isMatched = true;
					curBary = curBarycentrics[j];
					break;
				}
			}
			if(isMatched)
			{
				space::Point a = refBary.ray.ori + refBary.ray.dir * refBary.t;
				space::Point b = curBary.ray.ori + curBary.ray.dir * curBary.t;
				glBegin(GL_LINES);
				glVertex3f(a.x, a.y, a.z);
				glVertex3f(b.x, b.y, b.z);
				glEnd();
			}
		}
		glPointSize(oldPointSize);
	}

	if (drawRays) {
		float oldLineWidth;
		glGetFloatv(GL_LINE_WIDTH, &oldLineWidth);
		glLineWidth(0.1f);
		glColor3f(1.0f, 0.0f, 0.0f);
		float t = 100000.0f;
		for (int i = 0; i < refRays.size(); i++) {
			space::Point origin(refRays[i].ori.x, refRays[i].ori.y, refRays[i].ori.z);
			space::Point end = origin + refRays[i].dir * t;
			glBegin(GL_LINES);
			glVertex3f(origin.x, origin.y, origin.z);
			glVertex3f(end.x, end.y, end.z);
			glEnd();
		}
		glLineWidth(oldLineWidth);
	}

	if (refBarycentrics.size() == 0)
		return;

	if (drawBarycentricRays) {
		float oldLineWidth;
		glGetFloatv(GL_LINE_WIDTH, &oldLineWidth);
		glLineWidth(0.1f);
		glColor3f(0.0f, 1.0f, 0.0f);
		if (drawOnlyOne) {
			space::Point origin(refBarycentrics[baryIdx].ray.ori.x, refBarycentrics[baryIdx].ray.ori.y, refBarycentrics[baryIdx].ray.ori.z);
			space::Point end = origin + refBarycentrics[baryIdx].ray.dir * refBarycentrics[baryIdx].t;
			glBegin(GL_LINES);
			glVertex3f(origin.x, origin.y, origin.z);
			glVertex3f(end.x, end.y, end.z);
			glEnd();

			origin = space::Point(curBarycentrics[baryIdx].ray.ori.x, curBarycentrics[baryIdx].ray.ori.y, curBarycentrics[baryIdx].ray.ori.z);
			end = origin + curBarycentrics[baryIdx].ray.dir * curBarycentrics[baryIdx].t;
			glBegin(GL_LINES);
			glVertex3f(origin.x, origin.y, origin.z);
			glVertex3f(end.x, end.y, end.z);
			glEnd();
		} else {
			for (int i = 0; i < refBarycentrics.size(); i++) {
				space::Point origin(refBarycentrics[i].ray.ori.x, refBarycentrics[i].ray.ori.y, refBarycentrics[i].ray.ori.z);
				space::Point end = origin + refBarycentrics[i].ray.dir * refBarycentrics[i].t;
				glBegin(GL_LINES);
				glVertex3f(origin.x, origin.y, origin.z);
				glVertex3f(end.x, end.y, end.z);
				glEnd();
			}
			glColor3f(1.0f, 0.0f, 0.0f);
			for (int i = 0; i < curBarycentrics.size(); i++) {
				space::Point origin(curBarycentrics[i].ray.ori.x, curBarycentrics[i].ray.ori.y, curBarycentrics[i].ray.ori.z);
				space::Point end = origin + curBarycentrics[i].ray.dir * curBarycentrics[i].t;
				glBegin(GL_LINES);
				glVertex3f(origin.x, origin.y, origin.z);
				glVertex3f(end.x, end.y, end.z);
				glEnd();
			}
		}
		glLineWidth(oldLineWidth);
	}

	if (drawIntersectFaces) {
		glColor3f(0.0f, 1.0f, 1.0f);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		if (drawOnlyOne) {
			space::Point v1, v2, v3;
			//v1 = refBarycentrics[baryIdx].v[0];
			//v2 = refBarycentrics[baryIdx].v[1];
			//v3 = refBarycentrics[baryIdx].v[2];
			std::vector<space::IPoint> vertexIndices = model->GetGeometries()[0].GetVertexIndices();
			std::vector<space::Point> vertices = model->GetVertices();
			v1 = vertices[refBarycentrics[baryIdx].idx.x];
			v2 = vertices[refBarycentrics[baryIdx].idx.y];
			v3 = vertices[refBarycentrics[baryIdx].idx.z];
			glBegin(GL_TRIANGLES);
			glVertex3f(v1.x, v1.y, v1.z);
			glVertex3f(v2.x, v2.y, v2.z);
			glVertex3f(v3.x, v3.y, v3.z);
			glEnd();
			glBegin(GL_TRIANGLES);
			glVertex3f(v3.x, v3.y, v3.z);
			glVertex3f(v2.x, v2.y, v2.z);
			glVertex3f(v1.x, v1.y, v1.z);
			glEnd();

			float oldPointSize;
			glGetFloatv(GL_POINT_SIZE, &oldPointSize);
			glPointSize(10.0f);
			glColor3f(1.0f, 0.0f, 0.0f);
			glBegin(GL_POINTS);
			float v1coeff = refBarycentrics[baryIdx].coeffs[0];
			float v2coeff = refBarycentrics[baryIdx].coeffs[1];
			float v3coeff = refBarycentrics[baryIdx].coeffs[2];
			float x = v1.x * v1coeff + v2.x * v2coeff + v3.x * v3coeff;
			float y = v1.y * v1coeff + v2.y * v2coeff + v3.y * v3coeff;
			float z = v1.z * v1coeff + v2.z * v2coeff + v3.z * v3coeff;
			glVertex3f(x, y, z);
			glEnd();
			glColor3f(1.0f, 0.0f, 1.0f);
			glBegin(GL_POINTS);
			cv::Point3f ref3DCorner = ref3DCorners[refBarycentrics[baryIdx].fIdx];
			glVertex3f(ref3DCorner.x, ref3DCorner.y, ref3DCorner.z);
			glEnd();
			glPointSize(oldPointSize);
		} else {
			for (int i = 0; i < refBarycentrics.size(); i++) {
				space::Point v1, v2, v3;
				v1 = refBarycentrics[i].v[0];
				v2 = refBarycentrics[i].v[1];
				v3 = refBarycentrics[i].v[2];
				glBegin(GL_TRIANGLES);
				glVertex3f(v1.x, v1.y, v1.z);
				glVertex3f(v2.x, v2.y, v2.z);
				glVertex3f(v3.x, v3.y, v3.z);
				glEnd();
				glBegin(GL_TRIANGLES);
				glVertex3f(v3.x, v3.y, v3.z);
				glVertex3f(v2.x, v2.y, v2.z);
				glVertex3f(v1.x, v1.y, v1.z);
				glEnd();
			}
		}
	}


}

void VisionProject::PostProcess() {
	static cv::Mat refGray;
	static cv::Mat refColor;
	static cv::Mat refDepth;
	static cv::Mat prevGray;
	static std::vector<cv::Point2f> prevCorners;
	static std::vector<cv::Mat> refGrayPyr;
	static cv::Ptr<cv::Feature2D> orb = cv::ORB::create(); // TEST
	static int baryIdx = 1;


	const static cv::Size subPixWinSize(10, 10), winSize(31, 31);
	const static cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);

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

	if ((refColor.empty() || reInit)) {
		//std::cout << "Init Reference" << std::endl;
		reInit = false;
		baryIdx = 1;

		refCorners.clear();
		curCorners.clear();
		ref3DCorners.clear();
		refRays.clear();
		refBarycentrics.clear();
		refZDRatio.clear();
		prevCorners.clear();

		colorMat.copyTo(refColor);
		depthMat.copyTo(refDepth);
		std::vector<cv::Point2f> wholeCorners;
		cv::cvtColor(refColor, refGray, CV_BGR2GRAY);
		prevGray = refGray.clone();
		cv::goodFeaturesToTrack(refGray, wholeCorners, 1000, 0.01, 10);//, silhouetteMat);
		//cv::cornerSubPix(refGray, wholeCorners, subPixWinSize, cv::Size(-1, -1), termcrit);
		//1cv::buildOpticalFlowPyramid(refGray, refGrayPyr, winSize, 3);

		cv::Mat curZBuffer(vpArr[3], vpArr[2], CV_32FC1, depthBuffer);
		cv::flip(curZBuffer, curZBuffer, 0);

		// Calculate Rays
		for (int i = 0; i < wholeCorners.size(); i++) {
			// Check out of images
			if (wholeCorners[i].x < 0 || wholeCorners[i].x >= silhouetteMat.cols - 1 || wholeCorners[i].y < 0 || wholeCorners[i].y >= silhouetteMat.rows - 1)
				continue;
			if (silhouetteMat.at<uchar>(wholeCorners[i]) != 255)
				continue;

			space::Line _ray;
			TrackerUtils::GetRay1(projArr, mvArr, vpArr, wholeCorners[i].x, wholeCorners[i].y, _ray.ori, _ray.dir);
			refRays.push_back(_ray);

			cv::Mat _3DCorner(4, 1, CV_32FC1);
			_3DCorner.at<float>(0, 0) = (2.0f * wholeCorners[i].x) / (float) vpArr[2] - 1.0f;
			_3DCorner.at<float>(1, 0) = 1.0f - (2.0f * wholeCorners[i].y) / (float) vpArr[3];
			_3DCorner.at<float>(2, 0) = 2.0f * curZBuffer.at<float>(wholeCorners[i]) - 1.0f;
			_3DCorner.at<float>(3, 0) = 1.0f;

			_3DCorner = _projMat.inv() * _3DCorner;
			_3DCorner /= _3DCorner.at<float>(3, 0);
			_3DCorner = _mvMat.inv() * _3DCorner;
			_3DCorner /= _3DCorner.at<float>(3, 0);

			//ref3DCorners.push_back(cv::Point3f(_3DCorner.at<float>(0, 0), _3DCorner.at<float>(1, 0), (2.0f * 0.1f * 5000.0f) / (5000.0f + 0.1f - _z * (5000.0f - 0.1f))));
			float A = ((float *) _projMat.data)[10];
			float B = ((float *) _projMat.data)[11];


			float _d = depthMat.at<short>(wholeCorners[i]);
			float wx = ((wholeCorners[i].x - reCalcCx) / reCalcFx) * _d;
			float wy = ((wholeCorners[i].y - reCalcCy) / reCalcFy) * _d;

			_d = sqrtf(wx * wx + wy * wy + _d * _d);
			cv::Mat w3d(4, 1, CV_32FC1);
			w3d.at<float>(0, 0) = wx;
			w3d.at<float>(1, 0) = wy;
			w3d.at<float>(2, 0) = _d;
			w3d.at<float>(3, 0) = 1.0f;
			w3d = _mvMat * w3d;
			w3d /= w3d.at<float>(3,0);
			float _z = curZBuffer.at<float>(wholeCorners[i]);
			float virtualZ = B / (2 * _z - 1.0 + A);
			float ratio = _d / virtualZ;
			//refZDRatio.push_back(ratio);

			float maxLen = -FLT_MAX;
			float maxT = 0.0f;
			float minLen = FLT_MAX;
			float minT = 0.0f;
			float t = 0;

			bool isIntersectF = false;
			bool isIntersectB = false;
			space::Barycentric baryF; // front
			space::Barycentric baryB; // back

			for (int gIdx = 0; gIdx < model->GetNumGeometries(); gIdx++) // gIdx: geometry index
			{
				std::vector<space::IPoint> vertexIndices = model->GetGeometries()[gIdx].GetVertexIndices();
				std::vector<space::Point> vertices = model->GetVertices();
				for (int fIdx = 0; fIdx < vertexIndices.size(); fIdx++) // fIdx: face index
				{
					space::Point v1 = vertices[vertexIndices[fIdx].x];
					space::Point v2 = vertices[vertexIndices[fIdx].y];
					space::Point v3 = vertices[vertexIndices[fIdx].z];
					if (TrackerUtils::IsIntersectsTriangle3(_ray, v1, v2, v3, &t)) {
						float curLen = space::Point::squared_distance(_ray.ori, _ray.ori + _ray.dir * t);
						if (minLen > curLen) { // in case of front
							isIntersectF = true;
							minLen = curLen;
							minT = t;
							baryF = TrackerUtils::GetBarycentric(_ray, t, vertexIndices[fIdx], v1, v2, v3);
						}
						else if(maxLen < curLen){
							isIntersectB = true;
							maxLen = curLen;
							maxT = t;
							baryB = TrackerUtils::GetBarycentric(_ray, t, vertexIndices[fIdx], v1, v2, v3);
						}
					}
				}
			}

			if (isIntersectF) {
				baryF.d = _d;
				baryF.fIdx = refCorners.size();
				refBarycentrics.push_back(baryF);
				refCorners.push_back(wholeCorners[i]);
				prevCorners.push_back(wholeCorners[i]);
				refZDRatio.push_back(_d / baryF.t);
				ref3DCorners.push_back(cv::Point3f(_3DCorner.at<float>(0, 0), _3DCorner.at<float>(1, 0), _3DCorner.at<float>(2, 0)));

			}
			if(isIntersectB) {
				baryB.fIdx = -baryIdx;
				refBarycentrics.push_back(baryB);
			}

			if(isIntersectF || isIntersectB)
				baryIdx ++;
		}

		if(!refZDRatio.empty()) {
			std::sort(refZDRatio.begin(), refZDRatio.end());
			refZDRatioMedian = refZDRatio[refZDRatio.size() / 2 + 1];
			for(int i = 0; i < refZDRatio.size(); i++)
			{
				//std::cout <<"zdRatio"<< refZDRatio[i] << std::endl;
				//
			}
		}
		//std::cout<<refBarycentrics.size() << std::endl;
	} else if (!refZDRatio.empty()) {
		curBarycentrics.clear();
		cur3DCorners.clear();
		mMotionVectors.clear();

		GetMarkerPose(60.0f);

		//oneshot = false;
		cv::Mat curGray;
		cv::cvtColor(colorMat, curGray, CV_BGR2GRAY);
		std::vector<cv::Point2f> curCorners;
		cv::goodFeaturesToTrack(curGray, curCorners, 1000, 0.01, 10);
		//cv::cornerSubPix(curGray, curCorners, subPixWinSize, cv::Size(-1, -1), termcrit);

		std::vector<uchar> status;
		std::vector<float> err;

		cv::calcOpticalFlowPyrLK(prevGray, curGray, prevCorners, curCorners, status, err, winSize, 2, termcrit, 0, 0.001);
		for(int i = 0; i < curCorners.size(); i++)
		{
			bool isFound = false;
			space::Barycentric bary;
			for(int j = 0; j < refBarycentrics.size(); j++)
			{
				if(refBarycentrics[j].fIdx == i)
				{
					isFound = true;
					bary = refBarycentrics[j];
				}
			}
			if(!isFound)
				continue;

			space::Line _ray;

			float A = ((float *) _projMat.data)[10];
			float B = ((float *) _projMat.data)[11];

			double _d = depthMat.at<short>(curCorners[i]);
			if(_d <= 20)
				continue;
			float wx = ((curCorners[i].x - reCalcCx) / reCalcFx) * _d;
			float wy = ((curCorners[i].y - reCalcCy) / reCalcFy) * _d;
			_d = sqrtf(wx * wx + wy * wy + _d * _d);
			double fragDepth = 0.5 * (-A * _d + B) / _d + 0.5;
			//double alignedFragDepth = fragDepth / refZDRatioMedian;
			//double virtualZ = B / (2 * alignedFragDepth - 1.0 + A);
			//TrackerUtils::GetRay2(projArr, mvArr, vpArr, curCorners[i].x, curCorners[i].y, alignedFragDepth, _ray.ori, _ray.dir, virtualZ);
			TrackerUtils::GetRay1(projArr, mvArr, vpArr, curCorners[i].x, curCorners[i].y, _ray.ori, _ray.dir);
//			std::cout << "cam_origin: (" << bary.ray.ori.x << ", " << bary.ray.ori.y << ", " << bary.ray.ori.z <<")" << std::endl;
//			std::cout << "original ray: (" << bary.ray.dir.x <<", " << bary.ray.dir.y << ", " << bary.ray.dir.z << ")/" << bary.t  << "/" << bary.d<< std::endl;
//			std::cout << "current ray: (" << _ray.dir.x <<", " << _ray.dir.y << ", " << _ray.dir.z << ")/" << _d / refZDRatioMedian<< "/" << _d << std::endl;
//			std::cout << "zdratio: " << refZDRatioMedian << std::endl;
			//std::cout << "_d" <<_d << "fragDepth" << fragDepth << "alignedFragDepth" << alignedFragDepth << std::endl;
			//bary.t = (virtualZ - _ray.ori.z) / _ray.dir.z ;
			//bary.t = _d;//virtualZ;
			bary.t = _d / refZDRatioMedian;
			bary.ray = _ray;

			if(protectionMask.rows*protectionMask.cols == 1){
				protectionMask = cv::Mat::zeros(colorMat.size(), CV_8U);
			}
			int maskThresh = 100;
			if(colorMat.at<cv::Vec3b>(refCorners[i])[0] > maskThresh &&colorMat.at<cv::Vec3b>(refCorners[i])[1] > maskThresh &&colorMat.at<cv::Vec3b>(refCorners[i])[2] > maskThresh ){
				if(refCorners[i].y < 300)
				protectionMask.at<uchar>(refCorners[i]) = 255;

			}
			else{
				protectionMask.at<uchar>(curCorners[i]) = 0;
			}
			if(protectionMask.at<uchar>(refCorners[i]) > 0){
				for(int j = 0; j < refBarycentrics.size(); j++)
				{
					if(refBarycentrics[j].fIdx == i)
					{
						isFound = true;
						bary = refBarycentrics[j];
					}
				}
			}

			curBarycentrics.push_back(bary);
			space::Point a = bary.ray.ori + bary.ray.dir * bary.t;
			cur3DCorners.push_back(cv::Point3f(a.x, a.y, a.z));
			MotionVector3D mv3D[3];

			mv3D[0].index = bary.idx.x;
			mv3D[1].index = bary.idx.y;
			mv3D[2].index = bary.idx.z;

			for (int j = 0; j < 3; j++) {
				mv3D[j].x = bary.v[j].x - refBarycentrics[i].v[j].x;
				mv3D[j].y = bary.v[j].y - refBarycentrics[i].v[j].y;
				mv3D[j].z = bary.v[j].z - refBarycentrics[i].v[j].z;
				mMotionVectors.push_back(mv3D[j]);
			}
		}
		prevCorners.clear();
		for(int i = 0; i < curCorners.size(); i++)
		{
			prevCorners.push_back(curCorners[i]);
		}
		prevGray = curGray.clone();
//		for (int i = 0; i < curCorners.size(); i++) {
//			if (status[i]) {
//				bool isFound = false;
//				space::Barycentric bary;
//				for(int j = 0; j < refBarycentrics.size(); j++)
//				{
//					if(refBarycentrics[j].fIdx == i)
//					{
//						isFound = true;
//						bary = refBarycentrics[j];
//					}
//				}
//				if(!isFound)
//					continue;
//
//				space::Line _ray;
//				TrackerUtils::GetRay1(projArr, mvArr, vpArr, curCorners[i].x, curCorners[i].y, _ray.ori, _ray.dir);
//				float A = ((float *) _projMat.data)[10];
//				float B = ((float *) _projMat.data)[11];
//				float _d = depthMat.at<short>(curCorners[i]);
//				float fragDepth = 0.5 * (-A * _d + B) / _d + 0.5;
//				float alignedFragDepth = fragDepth / refZDRatioMedian;
//				float virtualD = B / (2 * alignedFragDepth - 1 + A);
//
//				bary.t = virtualD;
//				bary.ray = _ray;
//				curBarycentrics.push_back(bary);
//				space::Point a = bary.ray.ori + bary.ray.dir * bary.t;
//				cur3DCorners.push_back(cv::Point3f(a.x, a.y, a.z));
//				MotionVector3D mv3D[3];
//
//				mv3D[0].index = bary.idx.x;
//				mv3D[1].index = bary.idx.y;
//				mv3D[2].index = bary.idx.z;
//
//				for (int j = 0; j < 3; j++) {
//					mv3D[j].x = bary.v[j].x - refBarycentrics[i].v[j].x;
//					mv3D[j].y = bary.v[j].y - refBarycentrics[i].v[j].y;
//					mv3D[j].z = bary.v[j].z - refBarycentrics[i].v[j].z;
//					mMotionVectors.push_back(mv3D[j]);
//				}
//			}
//		}
	}

	return;
}

void VisionProject::SaveFrameAndResults() {
	std::stringstream fileCntSS;
	fileCntSS << "Saved/" << std::setfill('0') << std::setw(5) << g_frameCnt;
	//std::cout << fileCntSS.str() << std::endl;
	cv::imwrite(fileCntSS.str() + "c." + g_extension, colorMat);
	cv::Mat depthMat8UC3 = cv::Mat::zeros(depthMat.rows, depthMat.cols, CV_8UC3);
	for(int i = 0; i < depthMat8UC3.rows; i++)
	{
		for(int j = 0; j < depthMat8UC3.cols; j++)
		{
			depthMat8UC3.at<cv::Vec3b>(i, j)[0] = depthMat.at<short>(i, j) / 256;
			depthMat8UC3.at<cv::Vec3b>(i, j)[1] = depthMat.at<short>(i, j) % 256;
		}
	}
	cv::imwrite(fileCntSS.str() + "d." + g_extension, depthMat8UC3);
	std::ofstream ofs(fileCntSS.str() + "r.csv");
	if(g_frameCnt == 0)
	{
		for(int i = 0; i < refBarycentrics.size(); i++)
		{
			ofs
			<< refBarycentrics[i].fIdx <<","
			<< refBarycentrics[i].idx.x << ","
			<< refBarycentrics[i].idx.y << ","
			<< refBarycentrics[i].idx.z << ","
			<< refBarycentrics[i].v[0].x <<","
			<< refBarycentrics[i].v[0].y <<","
			<< refBarycentrics[i].v[0].z <<","
			<< refBarycentrics[i].v[1].x <<","
			<< refBarycentrics[i].v[1].y <<","
			<< refBarycentrics[i].v[1].z <<","
			<< refBarycentrics[i].v[2].x <<","
			<< refBarycentrics[i].v[2].y <<","
			<< refBarycentrics[i].v[2].z <<","
			<< refBarycentrics[i].coeffs[0] << ","
			<< refBarycentrics[i].coeffs[1] << ","
			<< refBarycentrics[i].coeffs[2] << ","
			<< refBarycentrics[i].ray.dir.x << ","
			<< refBarycentrics[i].ray.dir.y << ","
			<< refBarycentrics[i].ray.dir.z << ","
			<< refBarycentrics[i].ray.ori.x << ","
			<< refBarycentrics[i].ray.ori.y << ","
			<< refBarycentrics[i].ray.ori.z << ","
			<< refBarycentrics[i].t << std::endl;
		}
	}
	else
	{
		for(int i = 0; i < curBarycentrics.size(); i++)
		{
			ofs
			<< curBarycentrics[i].fIdx <<","
			<< curBarycentrics[i].idx.x << ","
			<< curBarycentrics[i].idx.y << ","
			<< curBarycentrics[i].idx.z << ","
			<< curBarycentrics[i].v[0].x <<","
			<< curBarycentrics[i].v[0].y <<","
			<< curBarycentrics[i].v[0].z <<","
			<< curBarycentrics[i].v[1].x <<","
			<< curBarycentrics[i].v[1].y <<","
			<< curBarycentrics[i].v[1].z <<","
			<< curBarycentrics[i].v[2].x <<","
			<< curBarycentrics[i].v[2].y <<","
			<< curBarycentrics[i].v[2].z <<","
			<< curBarycentrics[i].coeffs[0] << ","
			<< curBarycentrics[i].coeffs[1] << ","
			<< curBarycentrics[i].coeffs[2] << ","
			<< curBarycentrics[i].ray.dir.x << ","
			<< curBarycentrics[i].ray.dir.y << ","
			<< curBarycentrics[i].ray.dir.z << ","
			<< curBarycentrics[i].ray.ori.x << ","
			<< curBarycentrics[i].ray.ori.y << ","
			<< curBarycentrics[i].ray.ori.z << ","
			<< curBarycentrics[i].t << std::endl;
		}
	}
}

void VisionProject::GetMarkerPose(float scale) {
	//test code for ArUco Marker
	cv::Mat curColor, curGray;
	cv::Mat pseudoProjMat = cv::Mat::eye(3, 3, CV_32F);
	pseudoProjMat.at<float>(0, 0) = reCalcFx;
	pseudoProjMat.at<float>(0, 2) = reCalcCx;
	pseudoProjMat.at<float>(1, 1) = reCalcFy;
	pseudoProjMat.at<float>(1, 2) = reCalcCy;

	cv::Ptr<cv::aruco::Dictionary> dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
	std::vector<std::vector<cv::Point2f>> corners;
	std::vector<int> markerIds;
	cv::Ptr<cv::aruco::DetectorParameters> params;

	colorMat.copyTo(curColor);

	{
		cv::Mat markerImage;
		cv::aruco::drawMarker(dict, 0, 100, markerImage, 1);
		cv::cvtColor(markerImage, markerImage, CV_GRAY2BGR);
		//curColor(cv::Rect(50, 50, 100, 100)) = markerImage;
		for(int i =0; i<markerImage.rows; i++){
			for(int j =0; j<markerImage.cols; j++){
				//curColor.at<cv::Vec3b>(i +50, j +50 +g_frameCnt*10) = markerImage.at<cv::Vec3b>(i, j);
			}
		}

	}

	cv::cvtColor(curColor, curGray, CV_BGR2GRAY);

	std::vector<cv::Vec3d> rvecs;
	std::vector<cv::Vec3d> tvecs;
	cv::aruco::detectMarkers(curGray, dict, corners, markerIds);
	cv::aruco::drawDetectedMarkers(curColor, corners, markerIds);

	float projArr[16], mvArr[16];
	glGetFloatv(GL_PROJECTION_MATRIX, projArr);
	glGetFloatv(GL_MODELVIEW_MATRIX, mvArr);

	int vpArr[4];
	glGetIntegerv(GL_VIEWPORT, vpArr);

	//cv::aruco::estimatePoseSingleMarkers(corners, scale, intrinsic, cv::Mat::zeros(5, 1, CV_32FC1), rvecs, tvecs);
	cv::aruco::estimatePoseSingleMarkers(corners, scale, pseudoProjMat, cv::Mat::zeros(5, 1, CV_32FC1), rvecs, tvecs);

	for(int i =0; i<markerIds.size(); i++){
		if(markerIds[i] != 0)
			continue;

		objPts.clear();
		//objPts.push_back(cv::Point3f(-0.5*scale, -0.5*scale, 0.f));
		//objPts.push_back(cv::Point3f(0.5*scale, -0.5*scale, 0.f));
		//objPts.push_back(cv::Point3f(0.5*scale, 0.5*scale, 0.f));
		//objPts.push_back(cv::Point3f(-0.5*scale, 0.5*scale, 0.f));

		objPts.push_back(cv::Point3f(-15.5*0.16666*scale, 5.0*0.16666*scale, 0.f));
		objPts.push_back(cv::Point3f(0.0*scale, 5.0*0.16666*scale, 0.f));
		objPts.push_back(cv::Point3f(0.0*scale, -4.5*0.16666*scale, 0.f));
		objPts.push_back(cv::Point3f(-15.5*0.16666*scale, -4.5*0.16666*scale, 0.f));

		cv::Mat R, R_inv;
		cv::Rodrigues(rvecs[i], R);
		R_inv = R.inv();

		std::vector<cv::Point3f> objPtsCV;
		if(corners.size()>0) {
			for (auto &p:objPts) {
				cv::Point3f newP;
				newP.x = (float) R.at<double>(0, 0) * p.x + -(float) R.at<double>(0, 1) * p.y +
						 -(float) R.at<double>(0, 2) * p.z + 1 * tvecs[i][0];
				newP.y = (float) R.at<double>(1, 0) * p.x + -(float) R.at<double>(1, 1) * p.y +
						 -(float) R.at<double>(1, 2) * p.z + 1 * tvecs[i][1];
				newP.z = (float) R.at<double>(2, 0) * p.x + -(float) R.at<double>(2, 1) * p.y +
						 -(float) R.at<double>(2, 2) * p.z + 1 * tvecs[i][2];
				p.x = newP.x;
				p.y = -newP.y;
				p.z = -newP.z;

				objPtsCV.push_back(newP);
			}
		}

		std::vector<cv::Point2f> imgPts;
		//cv::projectPoints(objPts, rvecs[i], tvecs[i], intrinsic, cv::Mat::zeros(5, 1, CV_32FC1), imgPts);
		if(corners.size()>0) {
			cv::projectPoints(objPtsCV, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), intrinsic,
							  cv::Mat::zeros(5, 1, CV_32FC1), imgPts);
		}

		if(markerIds[i] != 0)
			continue;



		markerBarycentrics.clear();
		for(int j =0; j<imgPts.size(); j++) {
			space::Line _ray;
			TrackerUtils::GetRay1(projArr, mvArr, vpArr, imgPts[j].x, imgPts[j].y, _ray.ori, _ray.dir);
			refRays.push_back(_ray);

			bool isIntersectF = false;
			bool isIntersectB = false;
			space::Barycentric baryF; // front
			space::Barycentric baryB; // back

			float maxLen = -FLT_MAX;
			float maxT = 0.0f;
			float minLen = FLT_MAX;
			float minT = 0.0f;
			float t = 0;

			for (int gIdx = 0; gIdx < model->GetNumGeometries(); gIdx++) // gIdx: geometry index
			{
				std::vector<space::IPoint> vertexIndices = model->GetGeometries()[gIdx].GetVertexIndices();
				std::vector<space::Point> vertices = model->GetVertices();
				for (int fIdx = 0; fIdx < vertexIndices.size(); fIdx++) // fIdx: face index
				{
					space::Point v1 = vertices[vertexIndices[fIdx].x];
					space::Point v2 = vertices[vertexIndices[fIdx].y];
					space::Point v3 = vertices[vertexIndices[fIdx].z];
					if (TrackerUtils::IsIntersectsTriangle3(_ray, v1, v2, v3, &t)) {
						float curLen = space::Point::squared_distance(_ray.ori, _ray.ori + _ray.dir * t);
						if (minLen > curLen) { // in case of front
							isIntersectF = true;
							minLen = curLen;
							minT = t;
							baryF = TrackerUtils::GetBarycentric(_ray, t, vertexIndices[fIdx], v1, v2, v3);
						}
						else if(maxLen < curLen){
							isIntersectB = true;
							maxLen = curLen;
							maxT = t;
							baryB = TrackerUtils::GetBarycentric(_ray, t, vertexIndices[fIdx], v1, v2, v3);
						}
					}
				}
			}

			if (isIntersectF) {
				//baryF.d = _d;
				baryF.fIdx = j;
				markerBarycentrics.push_back(baryF);

			}
			if(isIntersectB) {
				baryB.fIdx = j;
				markerBarycentrics.push_back(baryB);
			}
		}

		for(int j = 0; j<imgPts.size(); j++) {
			cv::circle(curColor, imgPts[j], 2, cv::Scalar(0, 50*j, 255), 3);
		}

		cv::aruco::drawAxis(curColor, intrinsic, cv::Mat::zeros(5, 1, CV_32FC1), rvecs[i], tvecs[i], 0.005f);
	}
	if(corners.size() == 0){
		objPts.clear();
	}
	//pImagePlane->SetImage((void *) curColor.data);
	//pImagePlane->Draw();
}

void VisionProject::InitializeGL() {
	g_settings.LoadSetting("../data/Setting.ini");
	g_extension = g_settings.GetImageExtension();
	FRAME_HEIGHT = g_settings.GetFrameHeight();
	FRAME_WIDTH = g_settings.GetFrameWidth();
	intrinsic = g_settings.GetIntrinsic();
	modelview = g_settings.GetInitPose();

	depthBuffer = new float[FRAME_WIDTH * FRAME_HEIGHT];
	silhouetteMat = cv::Mat(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC1);
	InitRealsense();
	InitGLFW(FRAME_WIDTH, FRAME_HEIGHT, "K-AR", ErrorCallback, MouseCallback, KeyCallback, intrinsic);
	pImagePlane = new ImagePlane(FRAME_WIDTH, FRAME_HEIGHT);
	model = new obj::ObjModel();
	model->Load(g_settings.GetModelPath(), g_settings.GetModelFile());
}

bool VisionProject::RenderingLoop() {
static bool skipped = false;
#if defined(PLAYBACK_REALSENSE)
	ReadSavedFrame();
#elif !defined(NO_REALSENSE) // Use Realsense
	if(skipped)
		Idle();
	else
	{
		for(int i  = 0; i < 10; i++)
			Idle();
		skipped = true;
	}
#endif

	Render(); // Always render
//
//#if !defined(NO_REALSENSE)
	PostProcess();
//#elif defined(PLAYBACK_REALSENSE)
	//ReadSavedResults();
//#endif

//#if defined(RECORD_REALSENSE)
	SaveFrameAndResults();
//#endif

	//std::cout << refBarycentrics.size() << " / " << curBarycentrics.size() << std::endl;
	//std::cout << g_frameCnt << " frames are loaded or saved." << std::endl;
	if(!pause)
		g_frameCnt ++;
	return true;
}

void VisionProject::TerminateGL() {
	glfwTerminate();
}


/* Main
int main(int argc, char *argv[])
{
	Settings settings;
	settings.LoadSetting("../data/Setting.ini");
	intrinsic = settings.GetIntrinsic();
	FRAME_WIDTH = settings.GetFrameWidth();
	FRAME_HEIGHT = settings.GetFrameHeight();
	depthBuffer = new float[FRAME_WIDTH * FRAME_HEIGHT];
	silhouetteMat = cv::Mat(FRAME_HEIGHT, FRAME_WIDTH, CV_8UC1);

	InitRealsense();
	GLFWwindow* window = InitGLFW(FRAME_WIDTH, FRAME_HEIGHT, "K-AR", ErrorCallback, MouseCallback, KeyCallback, intrinsic);
	pImagePlane = new ImagePlane(FRAME_WIDTH, FRAME_HEIGHT);

	model = new obj::ObjModel();
	model->Load(settings.GetModelPath(), settings.GetModelFile());
	while (!glfwWindowShouldClose(window))
	{
		Idle(window);
		Render();
		PostProcess();

		glfwSwapBuffers(window);
		glfwPollEvents();
	}
	glfwDestroyWindow(window);
	glfwTerminate();
	return 0;
}
 */
