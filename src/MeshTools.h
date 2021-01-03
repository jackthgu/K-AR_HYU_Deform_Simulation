
#ifndef MESH_TOOLS_H_
#define MESH_TOOLS_H_

#include "BaseLib/motion/Mesh.h"
namespace OBJloader
{
	namespace Triangulation
	{
		void delaunayTriangulation(OBJloader::Mesh& mesh);
	}
}

#endif
