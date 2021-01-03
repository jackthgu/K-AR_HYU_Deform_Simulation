#pragma once

#include <opencv2/opencv.hpp>

#ifdef _WIN32
#include <windows.h>
#endif
#ifndef ANDROID

#include <GL/glew.h>

#else
#include <GLES/gl.h>
#endif

#include <vector>
#include <string>
#include <map>

#include "SpacePrimitives.h"

namespace obj {
	struct FeaturePoint {
		space::Point2D prevPt2D;
		space::Point prevPt3D;
		space::IPoint vertexIdx;
		//space::Line initialRay;
		space::Point2D currentPt2D;
		space::Point currentPt3D;
		space::Vector motionVector;
		space::Barycentric baryCoeffes;
	};

	class RGBA {
	public:
		RGBA() : r(1.0f), g(1.0f), b(1.0f), a(1.0f) {}

		RGBA(float _r, float _g, float _b, float _a) : r(_r), g(_g), b(_b), a(_a) {}

		float r, g, b, a;
	};

	class Material {
	public:
		Material();

		~Material();

	private:
		std::string texName;
		unsigned int tex;
		RGBA ambient;
		RGBA diffuse;
		RGBA specular;
		cv::Mat texImage;

		friend class ObjModel;
	};

	class Geometry {
	public:
		Geometry();

		~Geometry();

		std::vector<space::IPoint> GetVertexIndices() { return vertexIndices; }

	private:
		std::string name;
		std::vector<space::IPoint> vertexIndices;
		std::vector<space::IPoint> texcoordIndices;
		std::vector<space::IPoint> normalIndices;
		std::string materialName;
		float texcoordBound[4];

		friend class ObjModel;
	};

	class ObjModel {
	public:
		ObjModel();

		~ObjModel();

		bool Load(const std::string &dirpath, const std::string &filename);

		bool LoadMaterial(const std::string &dirpath, const std::string &filename);

		bool Save(const std::string &dirpath, const std::string &filename);

		bool SaveMaterial(const std::string &dirpath, const std::string &filename);

		void Draw();

		void DrawSilhouette(float r, float g, float b);

		void DrawWireframe(float r, float g, float b);

		void DrawTextureMap();

		void ClearTextureMap();

		void ComputeTexcoordBound();

		//void RemoveRedundantVertices();
		int GetNumGeometries();

		void ComputeNormalVectors();

		void ComputeBoundingBox();

		std::vector<space::Point> GetVertices();

		std::vector<space::IPoint> GetVertexIndices();

		std::vector<space::Point2D> GetTexcoord();

		std::vector<space::IPoint> GetTexcoordIndices();

		std::vector<Geometry> GetGeometries();

		void DeformUsingFeatures(std::vector<FeaturePoint> &features);

	private:
		std::vector<space::Point> vertices;
		std::vector<space::Point2D> texcoords;
		std::vector<space::Vector> normals;
		std::vector<Geometry> geometries;
		std::map<std::string, Material> materials;
		std::vector<Geometry>::iterator selectedGeometry;
		std::pair<space::Point, space::Point> boundingBox;
	};
};

