//
// Created by jin on 19. 1. 24.
//
#include <map>
#include "stdafx.h"
#include "MainLib/OgreFltk/RE.h"
#include <BaseLib/math/Metric.h>
#include <MainLib/OgreFltk/PointClouds.h>
#include "CompareHelper.h"

#define COMPARE_SCALE 1/7 // The smaller it is, the higher the similarity is.

std::tuple<int, int, int> tupleVector(vector3 v) {
	return std::make_tuple((int) (v.x * COMPARE_SCALE), (int) (v.y * COMPARE_SCALE), (int) (v.z * COMPARE_SCALE));
}

void CompareHelper::SetReferenceMesh(OBJloader::Mesh *ref) {
	int vCount = ref->numVertex();
	for (int i = 0; i < vCount; ++i) {
		// Multiply scale and make vertices coordinates to int so we can compare vertex coordinates like "Voxel".
		vector3 vertex = ref->getVertex(i);
		std::tuple<int, int, int> tv = tupleVector(vertex);
		mVoxelMap[tv] = 1;
	}
	std::cout << "Reference Mesh is saved. vCount : " << vCount << std::endl;
}

double CompareHelper::Compare(OBJloader::Mesh *mesh) {
	/**
 	 * check every vertex in mesh2 if it is already in map or not
 	 * count them and return the result
	 */
	int refVertexCount = static_cast<int>(mVoxelMap.size());
	int meshVertexCount = mesh->numVertex();
	int duplicatedVertexCount = 0;

	for (int i = 0; i < meshVertexCount; ++i) {
		vector3 vertex = mesh->getVertex(i);
		std::tuple<int, int, int> tv = tupleVector(vertex);

		if (mVoxelMap.count(tv)) {
			// Vertex is already exist
//			std::cout << "vertex exists : (" << std::get<0>(tv) << ", " << std::get<1>(tv) << ", " << std::get<2>(tv) << ")" << std::endl;
			duplicatedVertexCount++;
		}
	}

	// Return ratio of matching voxel.
	int bigger = refVertexCount > meshVertexCount ? refVertexCount : meshVertexCount;
	double result = (double) duplicatedVertexCount / (double) bigger;
//	std::cout << "refv : " <<refVertexCount << ", meshv : " << meshVertexCount << ", dupv : " << duplicatedVertexCount << std::endl;
	return result;
}

int CompareHelper::RMSE(OBJloader::Mesh *ref, OBJloader::Mesh *target, double *max, double *min, double *avg) {
	static const double SCALE = 1; // Means 1 unit is 10mm
	if (ref == nullptr || target == nullptr) {
		return -1;
	}
	int size = ref->numVertex();
	double maximum = -1;
	double totalDistanceSquare = 0;
	double minimum = 9999999;

	if (size != target->numVertex()) {
		std::cout << "Two mesh should have same index" << std::endl;
		return -1;
	}

	for (int i = 0; i < size; ++i) {
		double differenceX = ref->getVertex(i).x - target->getVertex(i).x;
		double differenceY = ref->getVertex(i).y - target->getVertex(i).y;
		double differenceZ = ref->getVertex(i).z - target->getVertex(i).z;
		double distanceSquare = differenceX * differenceX + differenceY * differenceY + differenceZ * differenceZ;
		if (distanceSquare != distanceSquare) {
			std::cout << "NAN ERROR!! - ref(" << i << ") : " << ref->getVertex(i).x << ", " <<ref->getVertex(i).y << ", " << ref->getVertex(i).z << std::endl;
			continue;
		}
		totalDistanceSquare += distanceSquare;

		if (maximum < distanceSquare) {
			maximum = distanceSquare;
		}
		if (minimum > distanceSquare) {
			minimum = distanceSquare;
		}
	}

	if (maximum == -1 || minimum == 9999999) {
		std::cout << "max, min calculation is wrong" << std::endl;
		return -1;
	}

	//printf("tot %f %d\n", totalDistanceSquare, size);
	// Set result
	*max = sqrt(maximum)*SCALE;
	*min = sqrt(minimum)*SCALE;
	*avg = sqrt(totalDistanceSquare / size)*SCALE; // RMSError

	return 0;
}

int CompareHelper::RMSE(OBJloader::Mesh *ref, vector3N targetVertices, double *max, double *min, double *avg) {
	if (ref == nullptr || targetVertices == nullptr || ref->numVertex() == 0 || targetVertices.size() == 0) {
		return -1;
	}
	int size = ref->numVertex();
	double maximum = -1;
	double totalDistanceSquare = 0;
	double minimum = 9999999;

	if (size != targetVertices.size()) {
		std::cout << "Two mesh should have same index; ref: " << size << ", target: " << targetVertices.size() << std::endl;
		return -1;
	}

	for (int i = 0; i < size; ++i) {
		double differenceX = ref->getVertex(i).x - targetVertices(i).x;
		double differenceY = ref->getVertex(i).y - targetVertices(i).y;
		double differenceZ = ref->getVertex(i).z - targetVertices(i).z;
		double distanceSquare = differenceX * differenceX + differenceY * differenceY + differenceZ * differenceZ;
		totalDistanceSquare += distanceSquare;

		if (maximum < distanceSquare) {
			maximum = distanceSquare;
		}
		if (minimum > distanceSquare) {
			minimum = distanceSquare;
		}
	}

	if (maximum == -1 || minimum == 9999999) {
		std::cout << "max, min calculation is wrong; max: " << maximum << ", min: " << minimum << "size: " << size << std::endl;
		return -1;
	}

	// Set result
	*max = maximum;
	*min = minimum;
	*avg = sqrt(totalDistanceSquare / size); // RMSError

	return 0;
}

int CompareHelper::PointCloudMetricError(OBJloader::Mesh *ref, OBJloader::Mesh *target, double *err) {
	PointCloudMetric m;
	if (ref->numVertex() < 1 || target->numVertex() < 1 || ref->numVertex() != target->numVertex()) {
		std::cout << "Vertex count is not same or 0" << std::endl;
		return -1;
	}

	// Eliminate nan vertices
	int size = ref->numVertex();
	int realSize = 0;
	for (int i = 0; i < size; ++i) {
		if (ref->getVertex(i).x == ref->getVertex(i).x) {
			realSize++;
		}
	}
	//std::cout << realSize << " out of " << size << "is not nan" << std::endl;

	vectorn a(realSize * 3);
	vectorn b(realSize * 3);

	int realIndex=0;
	for (int i = 0; i < ref->numVertex(); i++) {
		if (ref->getVertex(i).x == ref->getVertex(i).x) { // Check nan
			a.setVec3(realIndex * 3, ref->getVertex(i));
			b.setVec3(realIndex * 3, target->getVertex(i));
			realIndex++;
		}
	}

	*err = sqrt(m.CalcDistance(a, b)); // a: source positions, b: target positions. same size

	return 0;
}
