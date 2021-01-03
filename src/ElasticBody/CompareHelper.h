//
// Created by jin on 19. 1. 24.
//

#ifndef FLEXIBLEBODY_COMPAREHELPER_H
#define FLEXIBLEBODY_COMPAREHELPER_H

#include "BaseLib/baselib.h"
#include "MainLib/OgreFltk/Mesh.h"
#include <tuple>
#include "BaseLib/math/vector3N.h"

class CompareHelper {
public:
	/**
	 * Set reference mesh and save it to map
	 * @param Reference file (ex. ground truth)
	 */
	void SetReferenceMesh(OBJloader::Mesh *ref);

	/**
	 * Compare mesh to reference.
	 * @param Target mesh
	 * @return The ratio of similarity
	 */
	double Compare(OBJloader::Mesh *mesh);

	/**
	 * Compare mesh to reference.
	 * @param Two meshes, pointer for max, min, result
	 * @return 0 for success, -1 for error
	 */
	int RMSE(OBJloader::Mesh *ref, OBJloader::Mesh *target, double* max, double* min, double* avg);

	/**
	 * Compare mesh to reference.
	 * @param Ref mesh, target vertices, pointer for max, min, result
	 * @return 0 for success, -1 for error
	 */
	int RMSE(OBJloader::Mesh *ref, vector3N targetVertices, double* max, double* min, double* avg);

	/**
	 * Compare meshes with PointCloudMetric
	 * @param ref mesh
	 * @param target mesh
	 * @param err pointer to get result
	 * @return 0 for success, -1 for error
	 */
	int PointCloudMetricError(OBJloader::Mesh *ref, OBJloader::Mesh *target, double *err);

private:
	std::map<std::tuple<int, int, int>, int> mVoxelMap;
};


#endif //FLEXIBLEBODY_COMPAREHELPER_H
