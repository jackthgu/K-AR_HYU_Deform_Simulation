#include "stdafx.h"
#ifdef USE_CGAL
#include "CGAL/DelaunayTriangulation_P3_xy.h"
#endif
#include "BaseLib/motion/Mesh.h"
#include "BaseLib/math/Operator.h"
#include "Delaunay.h"
#include "MeshTools.h"

void OBJloader::Triangulation::delaunayTriangulation(OBJloader::Mesh& mesh)
{
	Delaunay_Triangulation::vertexSet vertices;
	
	for(int i=0; i<mesh.numVertex(); i++)
	{
		vertices.insert(Delaunay_Triangulation::vertex(mesh.getVertex(i).x,mesh.getVertex(i).y, i));
	}
	Delaunay_Triangulation::Delaunay d;
	Delaunay_Triangulation::triangleSet output;
	d.Triangulate(vertices, output);

	int faceCount=0;

	for(Delaunay_Triangulation::ctIterator i=output.begin(); i!=output.end(); ++i) faceCount++;

	mesh.resize(mesh.numVertex(), faceCount);
	
	faceCount=0;
	for(Delaunay_Triangulation::ctIterator i=output.begin(); i!=output.end(); ++i)
	{
		mesh.getFace(faceCount).vertexIndex(0)=i->GetVertex(0)->GetIndex();
		mesh.getFace(faceCount).vertexIndex(1)=i->GetVertex(1)->GetIndex();
		mesh.getFace(faceCount).vertexIndex(2)=i->GetVertex(2)->GetIndex();
		faceCount++;
	}

}


#ifdef USE_CGAL
void OBJloader::Triangulation::delaunayTriangulationCGAL(OBJloader::Mesh& mesh)
{
	std::vector<Point> vertices(mesh.m_arrayVertex.size());

	for(int i=0; i<mesh.numVertex(); i++)
	{
		vertices[i]=Point(mesh.m_arrayVertex[i].pos.x,mesh.m_arrayVertex[i].pos.z, i);
	}

	//Triangulation t;
	DelaunayTriangulation t;
	t.insert(vertices.begin(), vertices.end());

	
	mesh.isBoundaryVertex.resize(mesh.m_arrayVertex.size());
	mesh.isBoundaryVertex.clearAll();

	Vertex_circulator vc = t.incident_vertices(t.infinite_vertex()),
	done(vc);
	if (vc != 0) {
		do 
		{ 
			mesh.isBoundaryVertex.setAt(vc->point().z());
			//std::cout << vc->point() << std::endl;
		} while(++vc != done);
	}

	int faceCount=0;

	for(FaceIterator i=t.finite_faces_begin(); i!=t.finite_faces_end(); ++i) faceCount++;

	mesh.m_arrayFace.resize(faceCount);
	
	faceCount=0;
	for(FaceIterator i=t.finite_faces_begin(); i!=t.finite_faces_end(); ++i)
	{
		mesh.m_arrayFace[faceCount].vertexIndex[0]=i->vertex(0)->point().z();
		mesh.m_arrayFace[faceCount].vertexIndex[1]=i->vertex(1)->point().z();
		mesh.m_arrayFace[faceCount].vertexIndex[2]=i->vertex(2)->point().z();
		faceCount++;
	}
}

#endif
