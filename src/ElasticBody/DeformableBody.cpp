#include "stdafx.h"

#include <fstream>
#include "sgeom.h"
#include "DeformableBody.h"

using namespace std;

DeformableBody::DeformableBody()
{
}

bool DeformableBody::loadVolumeMesh(const char *file_name_)
{
	if ( !ebody.loadMesh(file_name_) ) return false;
	return true;
}

bool DeformableBody::loadSurfaceMesh(const char *file_name_)
{
	if ( !surf_embedded.loadMesh(file_name_) ) return false;
	return true;
}

bool DeformableBody::syncSurface()
{

	if ( !surf_embedded.setDynamicNodeMapping(ebody) ) return false;

	return true;
}

/*
bool DeformableBody::loadMass(char *file_name_nv_, double density_)
{
	int n;
	double umass;
	ifstream fin;

	fin.open(file_name_nv_);

	if ( !fin.is_open() ) return false;

	fin >> n;

	if ( n != ebody.getNumNode() ) return false;

	for (int i=0; i<n; i++) {
		fin >> umass;
		ebody.Nodes[i].mass = umass * density_;
	}

	fin.close();

	return true;
}*/

/*
bool DeformableBody::loadMeshWeight(char *file_name_mw_)
{
	int m;
	ifstream fin;

	fin.open(file_name_mw_);

	if ( !fin.is_open() ) return false;

	fin >> m;

	if ( m != ebody.getNumTetra() ) return false;

	for (int i=0; i<m; i++) {
		fin >> ebody.elastic_force.m_mesh_weight[i];
	}

	fin.close();

	return true;
}
*/

/*void DeformableBody::calc_NodalVolume_MeshWeight(char *file_name_nv_, char *file_name_mw_, int n_)
{
	vector<double> nv(ebody.getNumNode());
	vector<double> mw(ebody.getNumTetra());

	for (int i=0; i<ebody.getNumNode(); i++) {
		nv[i] = 0;
	}
	for (int i=0; i<ebody.getNumTetra(); i++) {
		mw[i] = 0;
	}

	// finding the dimension of an enclosing hexahedron
	double xl, xu, yl, yu, zl, zu;
	xl = xu = ebody.Nodes[0].xg[0];
	yl = yu = ebody.Nodes[0].xg[1];
	zl = zu = ebody.Nodes[0].xg[2];
	for (int i=1; i<ebody.getNumNode(); i++) {
		if ( ebody.Nodes[i].xg[0] > xu ) xu = ebody.Nodes[i].xg[0];
		if ( ebody.Nodes[i].xg[0] < xl ) xl = ebody.Nodes[i].xg[0];
		if ( ebody.Nodes[i].xg[1] > yu ) yu = ebody.Nodes[i].xg[1];
		if ( ebody.Nodes[i].xg[1] < yl ) yl = ebody.Nodes[i].xg[1];
		if ( ebody.Nodes[i].xg[2] > zu ) zu = ebody.Nodes[i].xg[2];
		if ( ebody.Nodes[i].xg[2] < zl ) zl = ebody.Nodes[i].xg[2];
	}

	// uniform grid distance
	double d = xu-xl;
	if ( yu-yl < d ) d = yu-yl;
	if ( zu-zl < d ) d = zu-zl;
	d /= fabs(double(n_));

	// volumetric integration
	Vec3 p;
	double phi[4];
	double dV = d*d*d;
	int nx = ceil((xu-xl)/d);
	int ny = ceil((yu-yl)/d);
	int nz = ceil((zu-zl)/d);
	for (int i=0; i<=nx; i++) {
		for (int j=0; j<=ny; j++) {
			for (int k=0; k<=nz; k++) {
				p = Vec3(xl+double(i)*d, yl+double(j)*d, zl+double(k)*d);
				// when p is inside of the surface
				if ( surf_embedded.insideSurface(p) ) {
					for (int l=0; l<ebody.getNumTetra(); l++) {
						if ( sgeom_inside_tetrahedron(p, 
							ebody.Nodes[ebody._vmesh_index[l][0]].xg, 
							ebody.Nodes[ebody._vmesh_index[l][1]].xg, 
							ebody.Nodes[ebody._vmesh_index[l][2]].xg, 
							ebody.Nodes[ebody._vmesh_index[l][3]].xg, 
							phi) ) 
						{
							nv[ebody._vmesh_index[l][0]] += phi[0] * dV;
							nv[ebody._vmesh_index[l][1]] += phi[1] * dV;
							nv[ebody._vmesh_index[l][2]] += phi[2] * dV;
							nv[ebody._vmesh_index[l][3]] += phi[3] * dV;
							mw[l] += dV;
							break;
						}
					}
				}
			}
		}
	}

	for (int i=0; i<ebody.getNumTetra(); i++) {
		mw[i] /= ebody.elastic_force.m_Vol[i];
	}

	ofstream fout;
	fout.open(file_name_nv_);
	fout << ebody.getNumNode() << endl << endl;
	for (int i=0; i<ebody.getNumNode(); i++) {
		fout << nv[i] << endl;
	}
	fout.close();
	fout.open(file_name_mw_);
	fout << ebody.getNumTetra() << endl << endl;
	for (int i=0; i<ebody.getNumTetra(); i++) {
		fout << mw[i] << endl;
	}
	fout.close();
}*/

void DeformableBody::updateSurfaceNodePosition()
{
	surf_embedded.updateNodePosition(ebody);
}


void DeformableBody::fitVolumeMesh()
{
	vector<bool> bInNode(ebody.getNumNode());
	vector<bool> bOutMesh(ebody.getNumTetra());
	vector<bool> bUselessNode(ebody.getNumNode());

	for (int i=0; i<ebody.getNumNode(); i++) {
		if ( surf_embedded.insideSurface(ebody.nodePos(i)) ) {
			bInNode[i] = true;
		} else {
			bInNode[i] = false;
			//ebody.Nodes[i].setRadius(0.01);
			//ebody.Nodes[i].setColor(Vec3(1,0,0));
		}
	}

	for (int i=0; i<ebody.getNumTetra(); i++) {
		if ( !bInNode[ebody._vmesh_index[i][0]] && !bInNode[ebody._vmesh_index[i][1]] && !bInNode[ebody._vmesh_index[i][2]] && !bInNode[ebody._vmesh_index[i][3]] ) {
			bOutMesh[i] = true;
		} else {
			bOutMesh[i] = false;
		}
	}

	for (int i=0; i<ebody.getNumNode(); i++) {
		bUselessNode[i] = true;
	}

	for (int i=0; i<ebody.getNumTetra(); i++) {
		if ( !bOutMesh[i] ) {
			for (int j=0; j<4; j++) {
				bUselessNode[ebody._vmesh_index[i][j]] = false;
			}
		}
	}

	for (int i=0; i<ebody.getNumNode(); i++) {
		if ( bUselessNode[i] ) {
			//ebody.Nodes[i].setRadius(0.01);
			//ebody.Nodes[i].setColor(Vec3(1,0,0));
			//discardNode(i);
		}
	}

	ofstream fout("valuable_node.txt");
	for (int i=0; i<ebody.getNumNode(); i++) {
		if ( !bUselessNode[i] ) {
			for (int j=0; j<3; j++) {
				fout << ebody.nodePos(i)<< "  ";
			}
			fout << endl;
		}
	}
	fout.close();

}

void DeformableBody::discardNode(int idx_)
{
	if ( idx_ < 0 ) return;

	// remove mesh element containing the node
	for (int i=0; i<ebody.getNumTetra(); i++) {
		for (int j=0; j<4; j++) {
			if ( ebody._vmesh_index[i][j] == idx_ ) {
				ebody._vmesh_index.erase(ebody._vmesh_index.begin()+i);
			}
		}
	}

	// reindexing
	for (int i=0; i<ebody.getNumTetra(); i++) {
		for (int j=0; j<4; j++) {
			if ( ebody._vmesh_index[i][j] > idx_ ) {
				ebody._vmesh_index[i][j] -= 1;
			}
		}
	}
}