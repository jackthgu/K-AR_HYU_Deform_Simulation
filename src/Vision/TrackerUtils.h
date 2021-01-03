#pragma once

#include "SpacePrimitives.h"
#include <opencv2/opencv.hpp>

namespace TrackerUtils {
	void GetRay1(float projArr[16], float mvArr[16], int vpArr[4], double screenX, double screenY, space::Point &origin, space::Vector &dir) {
		//x, y, width, height
		cv::Mat glprojMat(4, 4, CV_32FC1, projArr);
		cv::Mat glmvMat(4, 4, CV_32FC1, mvArr);
		cv::Mat projMat;
		cv::Mat mvMat;

		glprojMat.copyTo(projMat);
		glmvMat.copyTo(mvMat);

		projMat = projMat.t();
		mvMat = mvMat.t();
		cv::Mat invProjMat = projMat.inv();
		cv::Mat invMvMat = mvMat.inv();

		cv::Mat rayDir(4, 1, CV_32FC1);
		rayDir.at<float>(0, 0) = (2.0f * screenX) / (float) vpArr[2] - 1.0f;
		rayDir.at<float>(1, 0) = 1.0f - (2.0f * screenY) / (float) vpArr[3];
		rayDir.at<float>(2, 0) = -1.0f;
		rayDir.at<float>(3, 0) = 1.0f;

		rayDir = invProjMat * rayDir;
		rayDir.at<float>(2, 0) = -1.0f;
		rayDir.at<float>(3, 0) = 0.0f;

		rayDir = invMvMat * rayDir;
		rayDir /= cv::norm(rayDir);

		cv::Mat center = cv::Mat::zeros(4, 1, CV_32FC1);
		center.at<float>(3, 0) = 1.0f;
		center = invProjMat * center;
		center = invMvMat * center;
		center /= center.at<float>(3, 0);

		origin = space::Point(center.at<float>(0, 0), center.at<float>(1, 0), center.at<float>(2, 0));
		dir = space::Vector(rayDir.at<float>(0, 0), rayDir.at<float>(1, 0), rayDir.at<float>(2, 0));
	}
	void GetRay2(float projArr[16], float mvArr[16], int vpArr[4], double screenX, double screenY, double zb, space::Point &origin, space::Vector &dir, float &t) {
		//x, y, width, height
		cv::Mat glprojMat(4, 4, CV_32FC1, projArr);
		cv::Mat glmvMat(4, 4, CV_32FC1, mvArr);
		cv::Mat projMat;
		cv::Mat mvMat;

		glprojMat.copyTo(projMat);
		glmvMat.copyTo(mvMat);

		projMat = projMat.t();
		mvMat = mvMat.t();
		cv::Mat invProjMat = projMat.inv();
		cv::Mat invMvMat = mvMat.inv();

		cv::Mat rayDir(4, 1, CV_32FC1);
		rayDir.at<float>(0, 0) = (2.0f * screenX) / (float) vpArr[2] - 1.0f;
		rayDir.at<float>(1, 0) = 1.0f - (2.0f * screenY) / (float) vpArr[3];
		rayDir.at<float>(2, 0) = zb;
		rayDir.at<float>(3, 0) = 1.0f;

		rayDir = invProjMat * rayDir;
		//rayDir.at<float>(2, 0) = -1.0f;
		//rayDir.at<float>(3, 0) = 0.0f;

		rayDir = invMvMat * rayDir;
		t = cv::norm(rayDir) / rayDir.at<float>(3, 0);
		rayDir /= cv::norm(rayDir);

		cv::Mat center = cv::Mat::zeros(4, 1, CV_32FC1);
		center.at<float>(3, 0) = 1.0f;
		center = invProjMat * center;
		center = invMvMat * center;
		center /= center.at<float>(3, 0);

		origin = space::Point(center.at<float>(0, 0), center.at<float>(1, 0), center.at<float>(2, 0));
		dir = space::Vector(rayDir.at<float>(0, 0), rayDir.at<float>(1, 0), rayDir.at<float>(2, 0));
	}
	void GetRay4Camera(float projArr[16], float mvArr[16], int vpArr[4], double screenX, double screenY, space::Point &origin, space::Vector &dir) {
		cv::Mat projMat(4, 4, CV_32FC1, projArr);
		projMat = projMat.clone();
		cv::Mat mvMat(4, 4, CV_32FC1, mvArr);
		mvMat = mvMat.clone();
		projMat = projMat.t();
		mvMat = mvMat.t();
		cv::Mat invProjMat = projMat.inv();
		cv::Mat invMvMat = mvMat.inv();
		cv::Mat invMVP = /*invMvMat **/ invProjMat;

		//space::Point nearPt((2.0f * screenX) / (float)vpArr[2] - 1.0f, 1.0f - (2.0f * screenY) / (float)vpArr[3], 0.0f);
		space::Point nearPt(0.0f, 0.0f, 0.0f);
		space::Point unprojNearPt;
		unprojNearPt.x = invMVP.at<float>(0, 0) * nearPt.x + invMVP.at<float>(0, 1) * nearPt.y + invMVP.at<float>(0, 2) * nearPt.z + invMVP.at<float>(0, 3);
		unprojNearPt.y = invMVP.at<float>(1, 0) * nearPt.x + invMVP.at<float>(1, 1) * nearPt.y + invMVP.at<float>(1, 2) * nearPt.z + invMVP.at<float>(1, 3);
		unprojNearPt.z = invMVP.at<float>(2, 0) * nearPt.x + invMVP.at<float>(2, 1) * nearPt.y + invMVP.at<float>(2, 2) * nearPt.z + invMVP.at<float>(2, 3);
		float a = invMVP.at<float>(3, 0) * nearPt.x + invMVP.at<float>(3, 1) * nearPt.y + invMVP.at<float>(3, 2) * nearPt.z + invMVP.at<float>(3, 3);
		unprojNearPt.x /= a;
		unprojNearPt.y /= a;
		unprojNearPt.z /= a;

		space::Point farPt((2.0f * screenX) / (float) vpArr[2] - 1.0f, 1.0f - (2.0f * screenY) / (float) vpArr[3], 0.5f);
		space::Point unprojFarPt;
		unprojFarPt.x = invMVP.at<float>(0, 0) * farPt.x + invMVP.at<float>(0, 1) * farPt.y + invMVP.at<float>(0, 2) * farPt.z + invMVP.at<float>(0, 3);
		unprojFarPt.y = invMVP.at<float>(1, 0) * farPt.x + invMVP.at<float>(1, 1) * farPt.y + invMVP.at<float>(1, 2) * farPt.z + invMVP.at<float>(1, 3);
		unprojFarPt.z = invMVP.at<float>(2, 0) * farPt.x + invMVP.at<float>(2, 1) * farPt.y + invMVP.at<float>(2, 2) * farPt.z + invMVP.at<float>(2, 3);
		float b = invMVP.at<float>(3, 0) * farPt.x + invMVP.at<float>(3, 1) * farPt.y + invMVP.at<float>(3, 2) * farPt.z + invMVP.at<float>(3, 3);
		unprojFarPt.x /= b;
		unprojFarPt.y /= b;
		unprojFarPt.z /= b;

		dir = unprojFarPt - unprojNearPt;
		dir /= sqrtf(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);
		origin = unprojNearPt;
	}

	void GetRay4Model(float projArr[16], float mvArr[16], int vpArr[4], double screenX, double screenY, space::Point &origin, space::Vector &dir) {
		cv::Mat projMat(4, 4, CV_32FC1, projArr);
		projMat = projMat.clone();
		cv::Mat mvMat(4, 4, CV_32FC1, mvArr);
		mvMat = mvMat.clone();
		projMat = projMat.t();
		mvMat = mvMat.t();
		cv::Mat invProjMat = projMat.inv();
		cv::Mat invMvMat = mvMat.inv();
		cv::Mat invMVP = invMvMat * invProjMat;

		space::Point nearPt((2.0f * screenX) / (float) vpArr[2] - 1.0f, 1.0f - (2.0f * screenY) / (float) vpArr[3], 0.0f);
		//space::Point nearPt(0.0f, 0.0f, 0.0f);
		space::Point unprojNearPt;
		unprojNearPt.x = invMVP.at<float>(0, 0) * nearPt.x + invMVP.at<float>(0, 1) * nearPt.y + invMVP.at<float>(0, 2) * nearPt.z + invMVP.at<float>(0, 3);
		unprojNearPt.y = invMVP.at<float>(1, 0) * nearPt.x + invMVP.at<float>(1, 1) * nearPt.y + invMVP.at<float>(1, 2) * nearPt.z + invMVP.at<float>(1, 3);
		unprojNearPt.z = invMVP.at<float>(2, 0) * nearPt.x + invMVP.at<float>(2, 1) * nearPt.y + invMVP.at<float>(2, 2) * nearPt.z + invMVP.at<float>(2, 3);
		float a = invMVP.at<float>(3, 0) * nearPt.x + invMVP.at<float>(3, 1) * nearPt.y + invMVP.at<float>(3, 2) * nearPt.z + invMVP.at<float>(3, 3);
		unprojNearPt.x /= a;
		unprojNearPt.y /= a;
		unprojNearPt.z /= a;

		space::Point farPt((2.0f * screenX) / (float) vpArr[2] - 1.0f, 1.0f - (2.0f * screenY) / (float) vpArr[3], 0.5f);
		space::Point unprojFarPt;
		unprojFarPt.x = invMVP.at<float>(0, 0) * farPt.x + invMVP.at<float>(0, 1) * farPt.y + invMVP.at<float>(0, 2) * farPt.z + invMVP.at<float>(0, 3);
		unprojFarPt.y = invMVP.at<float>(1, 0) * farPt.x + invMVP.at<float>(1, 1) * farPt.y + invMVP.at<float>(1, 2) * farPt.z + invMVP.at<float>(1, 3);
		unprojFarPt.z = invMVP.at<float>(2, 0) * farPt.x + invMVP.at<float>(2, 1) * farPt.y + invMVP.at<float>(2, 2) * farPt.z + invMVP.at<float>(2, 3);
		float b = invMVP.at<float>(3, 0) * farPt.x + invMVP.at<float>(3, 1) * farPt.y + invMVP.at<float>(3, 2) * farPt.z + invMVP.at<float>(3, 3);
		unprojFarPt.x /= b;
		unprojFarPt.y /= b;
		unprojFarPt.z /= b;

		dir = unprojFarPt - unprojNearPt;
		dir /= sqrtf(dir.x * dir.x + dir.y * dir.y + dir.z * dir.z);
		origin = unprojNearPt;
	}

	bool IsIntersectsTriangle1(space::Line ray, space::Point v1, space::Point v2, space::Point v3, float *len) {
		static const float epsilon = FLT_MIN * 100;

		space::Vector normal = space::Vector::GetNormalFromPts(v1, v2, v3);

		/*float parallax = space::Vector::DotProduct(normal, ray.dir);
		if (parallax < epsilon && parallax > -epsilon)
			return false;*/

		space::Vector v1Vec(v1.x, v1.y, v1.z);
		space::Vector oVec(ray.ori.x, ray.ori.y, ray.ori.z);

		float t = (space::Vector::DotProduct(normal, v1Vec) - space::Vector::DotProduct(normal, oVec)) / space::Vector::DotProduct(normal, ray.dir);
		//float t = space::Vector::DotProduct(normal, v1Vec) / space::Vector::DotProduct(normal, ray);

		space::Point intersectPt = ray.ori + (ray.dir * t);
		space::Vector a(intersectPt.x - v1.x, intersectPt.y - v1.y, intersectPt.z - v1.z);
		space::Vector e1 = v2 - v1;
		space::Vector e2 = v3 - v1;

		space::Vector axe2 = space::Vector::CrossProduct(a, e2);
		float alpha = sqrtf(axe2.squared_length());
		space::Vector axe1 = space::Vector::CrossProduct(a, e1);
		float beta = sqrtf(axe1.squared_length());

		space::Vector e1xe2 = space::Vector::CrossProduct(e1, e2);

		alpha /= sqrtf(e1xe2.squared_length());
		beta /= sqrtf(e1xe2.squared_length());

		*len = t;
		return (alpha > 0 && beta > 0 && alpha + beta < 1);
	}

	bool IsIntersectsTriangle2(space::Line ray, space::Point v1, space::Point v2, space::Point v3, float *len) {
		static const float PI_2 = 6.28318530718;
		static const float angle_eps = 179.0 / 180.0;
		static const float epsilon = FLT_MIN * 10000;

		space::Vector normal = space::Vector::GetNormalFromPts(v1, v2, v3);

		float parallax = space::Vector::DotProduct(normal, ray.dir);
		if (parallax < epsilon && parallax > -epsilon)
			return false;

		space::Vector v1Vec(v1.x, v1.y, v1.z);
		space::Vector oVec(ray.ori.x, ray.ori.y, ray.ori.z);

		float t = (space::Vector::DotProduct(normal, v1Vec) - space::Vector::DotProduct(normal, oVec)) / space::Vector::DotProduct(normal, ray.dir);
		//float t = space::Vector::DotProduct(normal, v1Vec) / space::Vector::DotProduct(normal, ray);

		space::Point intersectPt = ray.ori + (ray.dir * t);
		*len = t;

		space::Vector L1 = v1 - intersectPt;
		space::Vector L2 = v2 - intersectPt;
		space::Vector L3 = v3 - intersectPt;

		L1 = L1.normalize();
		L2 = L2.normalize();
		L3 = L3.normalize();

		float dot1 = space::Vector::DotProduct(L1, L2);
		float dot2 = space::Vector::DotProduct(L2, L3);
		float dot3 = space::Vector::DotProduct(L3, L1);

		float angleSum = acos(dot1) + acos(dot2) + acos(dot3);

		if (angleSum < PI_2 * angle_eps)
			return false;
		else
			return true;
	}

	bool IsIntersectsTriangle3(space::Line ray, space::Point v1, space::Point v2, space::Point v3, float *len) {
		static const float epsilon = 0.1;

		space::Vector normal = space::Vector::GetNormalFromPts(v1, v2, v3);

		/*float parallax = space::Vector::DotProduct(normal, ray.dir);
		if (parallax < epsilon && parallax > -epsilon)
			return false;*/

		space::Vector v1Vec(v1.x, v1.y, v1.z);
		space::Vector oVec(ray.ori.x, ray.ori.y, ray.ori.z);

		float t = (space::Vector::DotProduct(normal, v1Vec) - space::Vector::DotProduct(normal, oVec)) / space::Vector::DotProduct(normal, ray.dir);
		//float t = space::Vector::DotProduct(normal, v1Vec) / space::Vector::DotProduct(normal, ray);

		space::Point intersectPt = ray.ori + (ray.dir * t);
		*len = t;

		space::Vector v2v1 = v2 - v1;
		space::Vector v3v1 = v3 - v1;

		space::Vector cross = space::Vector::CrossProduct(v2v1, v3v1);
		float faceArea = sqrtf(cross.squared_length());

		space::Vector v1IPt = v1 - intersectPt;
		space::Vector v2IPt = v2 - intersectPt;
		space::Vector v3IPt = v3 - intersectPt;

		space::Vector intersect23Cross = space::Vector::CrossProduct(v2IPt, v3IPt);
		float intersect23Area = sqrtf(intersect23Cross.squared_length());

		space::Vector intersect31Cross = space::Vector::CrossProduct(v3IPt, v1IPt);
		float intersect31Area = sqrtf(intersect31Cross.squared_length());

		space::Vector intersect12Cross = space::Vector::CrossProduct(v1IPt, v2IPt);
		float intersect12Area = sqrtf(intersect12Cross.squared_length());

		if (epsilon < abs((intersect23Area + intersect31Area + intersect12Area) - faceArea))
			return false;
		else
			return true;
	}

	bool IsIntersectTriangle(space::Point pt, space::Point v1, space::Point v2, space::Point v3) {
		space::Vector a(pt.x - v1.x, pt.y - v1.y, pt.z - v1.z);
		space::Vector e1 = v2 - v1;
		space::Vector e2 = v3 - v1;
		space::Vector axe2 = space::Vector::CrossProduct(a, e2);
		float alpha = sqrtf(axe2.squared_length());
		space::Vector axe1 = space::Vector::CrossProduct(a, e1);
		float beta = sqrtf(axe1.squared_length());

		space::Vector e1xe2 = space::Vector::CrossProduct(e1, e2);

		alpha /= sqrtf(e1xe2.squared_length());
		beta /= sqrtf(e1xe2.squared_length());
		return (alpha > 0 && beta > 0 && alpha + beta < 1);
	}

	space::Barycentric GetBarycentric(space::Line ray, float t, const space::IPoint vIdx, space::Point v1, space::Point v2, space::Point v3) {
		space::Point p = ray.ori + ray.dir * t;
		space::Vector v3v1 = v3 - v1;
		space::Vector v2v1 = v2 - v1;
		space::Vector pv1 = p - v1;

		float v123Area = sqrtf(space::Vector::CrossProduct(v2v1, v3v1).squared_length());
		float v12PArea = sqrtf(space::Vector::CrossProduct(pv1, v2v1).squared_length());
		float v13PArea = sqrtf(space::Vector::CrossProduct(pv1, v3v1).squared_length());

		space::Barycentric bary;
		bary.idx = vIdx;
		bary.v[0] = v1;
		bary.v[1] = v2;
		bary.v[2] = v3;

		float v3Coeff = v12PArea / v123Area;
		float v2Coeff = v13PArea / v123Area;
		float v1Coeff = 1 - v2Coeff - v3Coeff;

		bary.coeffs[0] = v1Coeff;
		bary.coeffs[1] = v2Coeff;
		bary.coeffs[2] = v3Coeff;
		bary.t = t;
		bary.ray = ray;
		return bary;
	}

};

