#include "stdafx.h"

#include "EmbeddedSurface.h"
#include <fstream>
#include "sgeom.h"

using namespace std;
using namespace TetraMeshLoader;

EmbeddedSurface::NodeSurf::NodeSurf()
{
	for (int i=0; i<4; i++) {
		phi[i] = 0.0;
		tetraIndex[i] = -1;
	}
}

bool EmbeddedSurface::NodeSurf::findMapping(TetraMesh const& m)
{
	double b[4];

	tetraIndex[0] = tetraIndex[1] = tetraIndex[2] = tetraIndex[3] = -1;

	for (int i=0; i<m.getNumTetra(); i++) {

		_quadi ith_tetraIndex=m._vmesh_index[i];

		if ( ith_tetraIndex[0] == -1 || ith_tetraIndex[1] == -1 || ith_tetraIndex[2] == -1 || ith_tetraIndex[3] == -1 ) return false;

		vector3& x=_x();
		if ( sgeom_inside_tetrahedron(x, 
			m.nodePos(ith_tetraIndex[0]), 
			m.nodePos(ith_tetraIndex[1]), 
			m.nodePos(ith_tetraIndex[2]), 
			m.nodePos(ith_tetraIndex[3]), b) ) {
			phi[0] = b[0];
			phi[1] = b[1];
			phi[2] = b[2];
			phi[3] = b[3];
			tetraIndex[0] = ith_tetraIndex[0];
			tetraIndex[1] = ith_tetraIndex[1];
			tetraIndex[2] = ith_tetraIndex[2];
			tetraIndex[3] = ith_tetraIndex[3];
			break;
		}
	}

	if ( tetraIndex[0] == -1 || tetraIndex[1] == -1 || tetraIndex[2] == -1 || tetraIndex[3] == -1 ) {
		return false;
	}

	return true;
}

void EmbeddedSurface::NodeSurf::updatePosition(TetraMesh const& m)
{
	if ( !isDynamic()) return;

	vector3 x;
	x.setValue(0,0,0);
	for (int i=0; i<4; i++) {
		x += phi[i] * m.nodePos(tetraIndex[i]);
	}

	_x()=x;

}

//=============================================================
//                 EmbeddedSurface
//=============================================================
EmbeddedSurface::EmbeddedSurface() 
{ 
}

bool EmbeddedSurface::loadMesh(const char *file_name_)
{
	if(!_mesh.loadMesh(file_name_))
		return false;

	int n=_mesh.numVertex();
		
	_nodes.resize(n);
	for (int i=0; i<n; i++) {
		_nodes[i]._pMesh=this;
		_nodes[i]._vi=i;
	}

	if ( !_check_mesh_index() ) return false;

	return true;
}

bool EmbeddedSurface::loadExtraInfo(char *file_name_)
{
	string str;
	ifstream fin;

	strExtraInfo.clear();

	fin.open(file_name_);
	
	if ( !fin.is_open() ) return false;

	getline(fin,str);
	while ( fin ) {
		strExtraInfo.push_back(str);
		getline(fin,str);
	}

	return true;
}

void EmbeddedSurface::exportOBJ(char *file_name_)
{
	ofstream fout(file_name_);
	fout << "# OBJ exported by EmbeddedSurface::exportOBJ()" << endl;
	fout << "# " << string(file_name_) << endl;
	for (int i=0; i<_mesh.numVertex(); i++) {

		vector3 x=_mesh.getVertex(i).pos;
		fout << "v " << x[0] << " " << x[1] << " " << x[2] << endl;
	}

	for (int i=0; i<strExtraInfo.size(); i++) {
		fout << strExtraInfo[i] << endl;
	}
	fout.close();
}

/*
void EmbeddedSurface::addMayaScriptBMP(char *script_file_name_, double time_, char *obj_file_name_, char *bmp_file_name_, char *material_name_)
{
	ofstream fout;
	fout.open(script_file_name_, ios_base::app);

	// load a model from an obj-file
	fout << "currentTime " << time_ << ";" << endl;
	fout << "string $s11 = " << "\"" << obj_file_name_ << "\";" << endl;
	fout << "string $s1 = substituteAllString($s11, \"\\\\\", \"/\");" << endl;		// replace '\\' with '/'
	fout << "file -import -type \"OBJ\" -pr $s1;" << endl;
	fout << "select -r Mesh; " << endl;
	fout << "sets -forceElement " << material_name_ << ";" << endl;
	fout << "SmoothPolygon;" << endl;

	// render as a bmp-file
	fout << "string $s22 = " << "\"" << bmp_file_name_ << "\";" << endl;
	fout << "string $s3 = `render persp`;" << endl;
	fout << "string $s33 = substituteAllString($s3, \"/\", \"\\\\\");" << endl;
	fout << "system(\"move \\\"\" + $s33 + \"\\\" \\\"\" + $s22 + \"\\\"\");" << endl;	// will be printed as system("move  \"" + $s33 + "\" \"" + $s22 + "\"");

	// delete the model
	fout << "select -r Mesh;" << endl;
	fout << "doDelete;" << endl;
	fout << endl;

	fout.close();
}
*/
void EmbeddedSurface::updateNodePosition(TetraMesh const& m)
{
	for (int i=0; i<_nodes.size(); i++) {
		_nodes[i].updatePosition(m);
	}
}

bool EmbeddedSurface::setDynamicNodeMapping(TetraMesh const& m)
{

	for (int i=0; i<_nodes.size(); i++) {
		if ( !_nodes[i].findMapping(m) ) return false;
	}
	return true;
}

bool EmbeddedSurface::insideSurface(const vector3 &x)
{
	vector3  p1, p2, p3, n, m, xp;
	double d, d_face, d_node, d_edge;
	int idx_face, idx_node, idx_edge_start, idx_edge_end;

	d_face = d_node = d_edge = 1E10;
	idx_face = idx_node = idx_edge_start = idx_edge_end = -1;

	// find nearest projectable triangle
	for (int i=0; i<_mesh.numFace(); i++) {
		// (p1,p2,p3) = i-th surface mesh
		p1 = _mesh.getVertex(_mesh.getFace(i).vi(0)).pos;
		p2 = _mesh.getVertex(_mesh.getFace(i).vi(1)).pos;
		p3 = _mesh.getVertex(_mesh.getFace(i).vi(2)).pos;
		
		// n = normal vector of (p1,p2,p3)
		n .cross(p2-p1, p3-p1);
		n.normalize();
		// xp = projection of x onto the plane extended by the triangle
		d = (x-p1)% n;
		xp = x - d*n;

		if ( sgeom_inside_triangle(xp, p1, p2, p3) ) {
			if ( fabs(d) < fabs(d_face) ) { 
				d_face = d;			// nearest (signed) distance
				idx_face = i;
			}
		}
	}

	// find nearest node
	for (int i=0; i<getNumNode(); i++) {
		d = Norm(x - nodePos(i));
		if ( d < d_node ) {
			d_node = d;
			idx_node = i;
		}
	}

	// find nearest projectable edge
	for (int i=0; i<getNumFace(); i++) {
		// (p1,p2)
		p1 = nodePos(meshIndex(i)[0]);
		p2 = nodePos(meshIndex(i)[1]);
		// m = normalized direction vector
		m = p2-p1;
		m.normalize();
		// xp = projection of x onto the line extended from the segment (p1,p2)
		xp = p1 + Inner(x-p1, m)*m;
		d = Norm(x-xp);

		if ( sgeom_inside_line_segment(xp, p1, p2) ) {
			if ( d < d_edge ) {
				d_edge = d;			// nearest distance
				idx_edge_start = meshIndex(i)[0];
				idx_edge_end = meshIndex(i)[1];
			}
		}
	}
	for (int i=0; i<getNumFace(); i++) {
		// (p1,p2)
		p1 = nodePos(meshIndex(i)[1]);
		p2 = nodePos(meshIndex(i)[2]);
		// m = normalized direction vector
		m = p2-p1;
		m.normalize();
		// xp = projection of x onto the line extended from the segment (p1,p2)
		xp = p1 + Inner(x-p1, m)*m;
		d = Norm(x-xp);

		if ( sgeom_inside_line_segment(xp, p1, p2) ) {
			if ( d < d_edge ) {
				d_edge = d;
				idx_edge_start = meshIndex(i)[1];
				idx_edge_end = meshIndex(i)[2];
			}
		}
	}
	for (int i=0; i<getNumFace(); i++) {
		// (p1,p2)
		p1 = nodePos(meshIndex(i)[2]);
		p2 = nodePos(meshIndex(i)[0]);
		// m = normalized direction vector
		m = p2-p1;
		m.normalize();
		// xp = projection of x onto the line extended from the segment (p1,p2)
		xp = p1 + Inner(x-p1, m)*m;
		d = Norm(x-xp);

		if ( sgeom_inside_line_segment(xp, p1, p2) ) {
			if ( d < d_edge ) {
				d_edge = d;
				idx_edge_start = meshIndex(i)[2];
				idx_edge_end = meshIndex(i)[0];
			}
		}
	}

	// ** case of nearest triangle:
	// if the signed distance from the nearest triangle, on which x can be projected, is positive,
	// then x is outside of the surface.
	if ( idx_face >= 0 && fabs(d_face) <= d_node && fabs(d_face) <= d_edge ) {
		if ( d_face > 0 ) return false;
		return true;
	} 

	// ** case of nearest node:
	// if x is located in the positive side of all the neighbor faces of the nearest node,
	// then x is outside of the surface.
	if ( idx_node >= 0 && d_node <= fabs(d_face) && d_node <= d_edge ) {
		intvectorn  idx_face = _find_neighbor_faces(idx_node);
		for (int i=0; i<idx_face.size(); i++) {
			if ( sgeom_positive_side_of_triangle_plane(x,	
				nodePos(meshIndex(idx_face[i])[0]), 
				nodePos(meshIndex(idx_face[i])[1]), 
				nodePos(meshIndex(idx_face[i])[2])) ) return false;
		}
		return true;
	}

	// ** case of nearest edge:
	// if x is located in the positive side of all the neighbor faces of the nearest edge,
	// then x is outside of the surface.
	if ( idx_edge_start >= 0 && idx_edge_end >= 0 && d_edge <= fabs(d_face) && d_edge <= d_node ) {
		intvectorn  idx_face = _find_neighbor_faces(idx_edge_start, idx_edge_end);
		for (int i=0; i<idx_face.size(); i++) {
			if ( sgeom_positive_side_of_triangle_plane(x,	
				nodePos(meshIndex(idx_face[i])[0]), 
				nodePos(meshIndex(idx_face[i])[1]), 
				nodePos(meshIndex(idx_face[i])[2])) ) return false;
		}
		return true;
	}

	return false;
}

bool EmbeddedSurface::distanceFromSurface(const vector3 &x, double *pDist_)
{
	vector3 p1, p2, p3, n, m, xp;
	double d, d_face, d_node, d_edge;
	int idx_face, idx_node, idx_edge_start, idx_edge_end;

	d_face = d_node = d_edge = 1E10;
	idx_face = idx_node = idx_edge_start = idx_edge_end = -1;

	// find nearest projectable triangle
	for (int i=0; i<getNumFace(); i++) {
		// (p1,p2,p3) = i-th surface mesh
		p1 = nodePos(meshIndex(i)[0]);
		p2 = nodePos(meshIndex(i)[1]);
		p3 = nodePos(meshIndex(i)[2]);
		// n = normal vector of (p1,p2,p3)
		n = Cross(p2-p1, p3-p1);
		n.normalize();
		// xp = projection of x onto the plane extended by the triangle
		d = Inner(x-p1, n);
		xp = x - d*n;

		if ( sgeom_inside_triangle(xp, p1, p2, p3) ) {
			if ( fabs(d) < fabs(d_face) ) { 
				d_face = d;			// nearest (signed) distance
				idx_face = i;
			}
		}
	}

	// find nearest node
	for (int i=0; i<getNumNode(); i++) {
		d = Norm(x - nodePos(i));
		if ( d < d_node ) {
			d_node = d;
			idx_node = i;
		}
	}

	// find nearest projectable edge
	for (int i=0; i<getNumFace(); i++) {
		// (p1,p2)
		p1 = nodePos(meshIndex(i)[0]);
		p2 = nodePos(meshIndex(i)[1]);
		// m = normalized direction vector
		m = p2-p1;
		m.normalize();
		// xp = projection of x onto the line extended from the segment (p1,p2)
		xp = p1 + Inner(x-p1, m)*m;
		d = Norm(x-xp);

		if ( sgeom_inside_line_segment(xp, p1, p2) ) {
			if ( d < d_edge ) {
				d_edge = d;			// nearest distance
				idx_edge_start = meshIndex(i)[0];
				idx_edge_end = meshIndex(i)[1];
			}
		}
	}
	for (int i=0; i<getNumFace(); i++) {
		// (p1,p2)
		p1 = nodePos(meshIndex(i)[1]);
		p2 = nodePos(meshIndex(i)[2]);
		// m = normalized direction vector
		m = p2-p1;
		m.normalize();
		// xp = projection of x onto the line extended from the segment (p1,p2)
		xp = p1 + Inner(x-p1, m)*m;
		d = Norm(x-xp);

		if ( sgeom_inside_line_segment(xp, p1, p2) ) {
			if ( d < d_edge ) {
				d_edge = d;
				idx_edge_start = meshIndex(i)[1];
				idx_edge_end = meshIndex(i)[2];
			}
		}
	}
	for (int i=0; i<getNumFace(); i++) {
		// (p1,p2)
		p1 = nodePos(meshIndex(i)[2]);
		p2 = nodePos(meshIndex(i)[0]);
		// m = normalized direction vector
		m = p2-p1;
		m.normalize();
		// xp = projection of x onto the line extended from the segment (p1,p2)
		xp = p1 + Inner(x-p1, m)*m;
		d = Norm(x-xp);

		if ( sgeom_inside_line_segment(xp, p1, p2) ) {
			if ( d < d_edge ) {
				d_edge = d;
				idx_edge_start = meshIndex(i)[2];
				idx_edge_end = meshIndex(i)[0];
			}
		}
	}

	// ** case of nearest triangle:
	// if the signed distance from the nearest triangle, on which x can be projected, is positive,
	// then x is outside of the surface.
	if ( idx_face >= 0 && fabs(d_face) <= d_node && fabs(d_face) <= d_edge ) {
		if ( pDist_ != NULL ) { *pDist_ = d_face; }
		if ( d_face > 0 ) return false;
		return true;
	} 

	// ** case of nearest node:
	// if x is located in the positive side of all the neighbor faces of the nearest node,
	// then x is outside of the surface.
	if ( idx_node >= 0 && d_node <= fabs(d_face) && d_node <= d_edge ) {
		intvectorn  idx_face = _find_neighbor_faces(idx_node);
		bool b_inside = true;
		for (int i=0; i<idx_face.size(); i++) {
			if ( sgeom_positive_side_of_triangle_plane(x,	
				nodePos(meshIndex(idx_face[i])[0]), 
				nodePos(meshIndex(idx_face[i])[1]), 
				nodePos(meshIndex(idx_face[i])[2]),
				pDist_) ) b_inside = false;
		}
        return b_inside;
	}

	// ** case of nearest edge:
	// if x is located in the positive side of all the neighbor faces of the nearest edge,
	// then x is outside of the surface.
	if ( idx_edge_start >= 0 && idx_edge_end >= 0 && d_edge <= fabs(d_face) && d_edge <= d_node ) {
		intvectorn  idx_face = _find_neighbor_faces(idx_edge_start, idx_edge_end);
		bool b_inside = true;
		for (int i=0; i<idx_face.size(); i++) {
			if ( sgeom_positive_side_of_triangle_plane(x,	
				nodePos(meshIndex(idx_face[i])[0]), 
				nodePos(meshIndex(idx_face[i])[1]), 
				nodePos(meshIndex(idx_face[i])[2]),
				pDist_) ) b_inside = false;
		}
		return b_inside;
	}

	return false;
}

bool EmbeddedSurface::_check_mesh_index()
{
	// check mesh
	for (int i=0; i<getNumFace(); i++) {
		// index check
		for (int j=0; j<3; j++) {
			if ( meshIndex(i)[j] < 0 || meshIndex(i)[j] >= getNumNode() ) return false;
		}
		// node duplicity check
		if ( meshIndex(i)[0] == meshIndex(i)[1] || meshIndex(i)[0] == meshIndex(i)[2] || meshIndex(i)[1] == meshIndex(i)[2]  ) {
			return false;
		}
	}

	return true;
}

intvectorn EmbeddedSurface::_find_neighbor_faces(int idx_)
{
	intvectorn  re;
	for (int i=0; i<getNumFace(); i++) {
		for (int j=0; j<3; j++) {
			if ( meshIndex(i)[j] == idx_ ) re.push_back(i);
		}
	}
	return re;
}

intvectorn  EmbeddedSurface::_find_neighbor_faces(int idxs_, int idxe_)
{
	intvectorn  a, b, re;
	a = _find_neighbor_faces(idxs_);
	b = _find_neighbor_faces(idxe_);
	// find intersection of a and b
	for (int i=0; i<a.size(); i++) {
		if ( b.findFirstIndex(a[i]) >= 0 ) {
			re.push_back(a[i]);
		}
	}
	return re;
}

/*
void EmbeddedSurface::render()
{
	if ( !isVisible() ) return;

	vector3 n;
	GLdouble previous_color[4];
	glGetDoublev(GL_CURRENT_COLOR, previous_color);

	glPushName(ELM_SURF);

	if ( isVisibleNode() ) {
		for (int i=0; i<getNumNode(); i++) {
			Nodes[i].render();
		}
	}

	if ( isVisibleMesh() ) {
		glPushMatrix();
		for (int i=0; i<getNumFace(); i++) {
			if ( draw_type == GLSUB_SOLID ) {
				glBegin(GL_TRIANGLES);
				glColor4d(colorMesh[0], colorMesh[1], colorMesh[2], alpha);
			} else {
				glBegin(GL_LINE_LOOP);
				glColor4d(colorGrid[0], colorGrid[1], colorGrid[2], alpha);
			}
			vector3 x1 = Nodes[meshIndex(i)[0]].x;
			vector3 x2 = Nodes[meshIndex(i)[1]].x;
			vector3 x3 = Nodes[meshIndex(i)[2]].x;
			vector3 n = Cross(x2-x1, x3-x1);
			n.normalize();

			glNormal3d(n[0], n[1], n[2]);

			glVertex3dv(x1.GetArray());
			glVertex3dv(x2.GetArray());
			glVertex3dv(x3.GetArray());

			glEnd();
		}
		glPopMatrix();
	}

	glPopName();

	glColor4dv(previous_color);
}


*/