#include "stdafx.h"

#include "MainLib/OgreFltk/Mesh.h"
#include "BaseLib/math/Operator.h"
#include "MeanValueWeights.h"
#include <BaseLib/math/Operator.h>
#include <BaseLib/math/Operator_NR.h>

#include "ClassificationLib/math/Interpolation.h"
#include "sparseicp/ICP_wrap.h"

void MeanValueWeights(OBJloader::Mesh const& m, vectorn & w, vector3 const& x)
{
	// Refer to [Ju et al. 2005] Mean Value Coordinates for Closed Triangular Meshes. Figure 4.
	vectorn d;
	vector3N u;
	d.setSize(m.numVertex());
	w.setSize(m.numVertex());
	u.setSize(m.numVertex());
	w.setAllValue(0);
	const double epsilon=1.0e-14;
	for(int i=0; i<m.numVertex(); i++)
	{
		vector3 const& p=m.getVertex(i);
		d[i]=x.distance(p);
		if(d[i]<epsilon)
		{
			w[i]=1.0;
			return;
		}
		u[i]=(p-x)/d[i];
	}
	
	double l[3];
	double theta[3],h,c[3],s[3],det ;
	int vi[3];
	for(int f=0; f<m.numFace(); f++)
	{
		vi[0]=m.getFace(f).vertexIndex(0);
		vi[1]=m.getFace(f).vertexIndex(1);
		vi[2]=m.getFace(f).vertexIndex(2);

		l[0]=u[vi[1]].distance(u[vi[2]]);
		l[1]=u[vi[2]].distance(u[vi[0]]);
		l[2]=u[vi[0]].distance(u[vi[1]]);

		h=0;
		for(int i=0; i<3; i++)
		{
			theta[i]=2.0*asin(l[i]/2.0);
			h+=theta[i];
		}

		h*=0.5;
		if(M_PI-h < epsilon)
		{
//			printf("error1\n");
			// x lies on t, use 2D barycentric coordinates
			w.setAllValue(0);

			for(int i=0; i<3; i++)
			{
				w[vi[i]]+=sin(theta[i])*d[vi[(i+2)%3]]*d[vi[(i+1)%3]];
			}
			w/=w.sum();
			return;
		}

		const double u0x = u[vi[0]][0];
		const double u0y = u[vi[0]][1];
		const double u0z = u[vi[0]][2];
		const double u1x = u[vi[1]][0];
		const double u1y = u[vi[1]][1];
		const double u1z = u[vi[1]][2];
		const double u2x = u[vi[2]][0];
		const double u2y = u[vi[2]][1];
		const double u2z = u[vi[2]][2];
		double det = u0x*u1y*u2z - u0x*u1z*u2y + u0y*u1z*u2x - u0y*u1x*u2z + u0z*u1x*u2y - u0z*u1y*u2x;

		int i;
		for(i=0; i<3; i++)
		{
			int ip1=(i+1)%3;
			int im1=(i+2)%3;
			c[i]=(2.0*sin(h)*sin(h-theta[i]))/(sin(theta[ip1])*sin(theta[im1]))-1;

			s[i]=sqrt(1.0-SQR(c[i]));
			if(det<0)
				s[i]*=-1.0;
			
			if(ABS(s[i])<epsilon || s[i] != s[i]) // s[i] can't be zero
				break;
		}
		if(i!=3) continue;	// x lies outside t on the same plane, ignore t

		for(int i=0; i<3; i++)
		{
			int ip1=(i+1)%3;
			int im1=(i+2)%3;
			w[vi[i]]+=(theta[i]-c[ip1]*theta[im1]-c[im1]*theta[ip1])/(d[vi[i]]*sin(theta[ip1])*s[im1]);
		}

	}

	w/=w.sum();
}

void MeanValueWeights(OBJloader::Mesh const& m, int maxNumWeights, intvectorn& index, vectorn & weights, vector3 const& x)
{
	vectorn w;
	MeanValueWeights(m, w, x);

	intvectorn sorted;
	sorted.sortedOrder(w);

	
	const double epsilon=1.0e-14;

	index.reserve(maxNumWeights);
	weights.reserve(maxNumWeights);

	index.setSize(0);
	weights.setSize(0);

	for(int i=0; i<sorted.size(); i++)
	{
		if(ABS(w[sorted[i]])<epsilon)
			continue;

		index.push_back(sorted[i]);
		weights.pushBack(w[sorted[i]]);
		if(index.size()==maxNumWeights)
			break;

		
	}
	weights/=weights.sum();
}

LinearMeshDeformer::LinearMeshDeformer()
:control_mesh(NULL), embedded_mesh(NULL)
{
}

LinearMeshDeformer::~LinearMeshDeformer()
{
	
}

bool LinearMeshDeformer::saveCorrespondence(const char* filename)
{
	BinaryFile file;
	bool res=file.openWrite(filename);
	if(!res) return false;

	file.packInt(weights.size());
	for(int i=0; i<weights.size();i++)
		file.pack(weights[i]);
	file.close();
	return true;
}

bool LinearMeshDeformer::loadCorrespondence(const char* filename, OBJloader::Mesh & control, OBJloader::Mesh& embedded)
{
	control_mesh=&control;
	embedded_mesh=&embedded;

	BinaryFile file;
	bool res=file.openRead(filename);
	if(!res) return false;

	int n;
	file.unpackInt(n);

	if(n!=embedded.numVertex())
		return false;
	weights.resize(n);
	for(int i=0; i<weights.size();i++)
		file.unpack(weights[i]);
	file.close();
	return true;
}

void LinearMeshDeformer::transfer(OBJloader::Mesh const& control, double scale_factor)
{
	ASSERT(control.numVertex()==control_mesh->numVertex());

	//std::cout << "Transfer in linearMeshDeformer" << std::endl;
	double inv_scale_factor=1.0/scale_factor;
	for(int i=0; i<embedded_mesh->numVertex(); i++)
	{
		vector3& target=embedded_mesh->getVertex(i);
		target.setValue(0,0,0);

		for(int j=0; j<weights[i].size(); j++)
			target+=control.getVertex(j)*(inv_scale_factor*weights[i][j]);
	}
}

void LinearMeshDeformer::transferTo(OBJloader::Mesh const& control, vector3N& out)
{
	ASSERT(control.numVertex()==control_mesh->numVertex());

	out.setSize(embedded_mesh->numVertex());
	for(int i=0; i<embedded_mesh->numVertex(); i++)
	{
		vector3& target=out(i);
		target.setValue(0,0,0);

		for(int j=0; j<weights[i].size(); j++)
			target+=control.getVertex(j)*weights[i][j];
	}
}

class KNNIDWMeshCoords :public LinearMeshDeformer {
	InterpolationNormalize* reg;
	//KNearestInterpolationFast* reg;
	vector3N delta;
public:
	KNNIDWMeshCoords(int m_nK, float m_fK):LinearMeshDeformer() 
	{
		reg=new KNearestInterpolation(new L2Metric(), m_nK, 2.0, m_fK );
		//reg=new RBInterpolation();
		//reg=new KNearestInterpolationFast(m_nK, 2.0, m_fK);
	}
	~KNNIDWMeshCoords() {}

	virtual void calculateCorrespondence(OBJloader::Mesh & control, OBJloader::Mesh& embedded)
	{
		control_mesh=&control;
		embedded_mesh=&embedded;
		matrixn sources(control.numVertex(), 3);
		for(int i=0; i<control.numVertex(); i++)
			sources.row(i).setVec3(0, control.getVertex(i));

		reg->initialize(sources);

		weights.resize(embedded.numVertex());
		vectorn v(3);
		vectorn w;
		intvectorn id;
		for(int i=0; i<embedded.numVertex(); i++)
		{
			v.setVec3(0, embedded.getVertex(i));
			reg->calcWeight(v, id, w);
			auto& ww=weights[i];
			ww.setSize(control.numVertex());
			ww.setAllValue(0.0);
			for(int j=0; j<id.size(); j++)
				ww[id[j]]=w[j];
		}
		delta.setSize(embedded.numVertex());
		for (int i=0; i<embedded.numVertex(); i++)
		{
			vector3 target(0,0,0);

			for(int j=0; j<weights[i].size(); j++)
				target+=control.getVertex(j)*weights[i][j];
			delta(i)=embedded.getVertex(i)-target;
		}
	}
	virtual void transfer(OBJloader::Mesh const& control, double scale_factor)
	{
		ASSERT(control.numVertex()==control_mesh->numVertex());

		//std::cout << "Transfer in KNNMeshCoord" << std::endl;

		double inv_scale_factor=1.0/scale_factor;
		for(int i=0; i<embedded_mesh->numVertex(); i++)
		{
			vector3& target=embedded_mesh->getVertex(i);
			target.setValue(0,0,0);

			for(int j=0; j<weights[i].size(); j++)
				target+=control.getVertex(j)*(inv_scale_factor*weights[i][j]);

			target+=delta(i);
		}
	}
};
MeshDeformer* createKNNIDWMeshDeformer(int m_nK, float m_fK)
{
	return new KNNIDWMeshCoords(m_nK, m_fK);
}

LinearDeformer::LinearDeformer()
	:LinearMeshDeformer(),
	embedded_vertices(NULL)
{
}

LinearDeformer::~LinearDeformer()
{
	
}
bool LinearDeformer::loadCorrespondence(const char* filename, OBJloader::Mesh & control, vector3N& embedded)
{
	control_mesh=&control;
	embedded_vertices=&embedded;

	BinaryFile file;
	bool res=file.openRead(filename);
	if(!res) return false;

	int n;
	file.unpackInt(n);

	if(n!=embedded.size())
		return false;
	weights.resize(n);
	for(int i=0; i<weights.size();i++)
		file.unpack(weights[i]);
	file.close();
	return true;
}
void LinearDeformer::transfer(OBJloader::Mesh const& control, double scale_factor)
{
	ASSERT(control.numVertex()==control_mesh->numVertex());
	ASSERT(embedded_mesh==NULL);

	//std::cout << "Transfer in linear Work" << std::endl;

	double inv_scale_factor=1.0/scale_factor;
	for(int i=0; i<embedded_vertices->size(); i++)
	{
		vector3& target=(*embedded_vertices)(i);
		target.setValue(0,0,0);

		for(int j=0; j<weights[i].size(); j++)
			target+=control.getVertex(j)*(inv_scale_factor*weights[i][j]);
	}
}


class MeanValueMeshCoords :public LinearMeshDeformer {
public:
	MeanValueMeshCoords():LinearMeshDeformer() {}
	~MeanValueMeshCoords() {}

	virtual void calculateCorrespondence(OBJloader::Mesh & control, OBJloader::Mesh & embedded);
};

MeshDeformer* createMeanValueMeshDeformer()
{
	return new MeanValueMeshCoords();
}

void MeanValueMeshCoords::calculateCorrespondence(OBJloader::Mesh & control, OBJloader::Mesh & embedded)
{
	int max_num_weights=100;
	control_mesh=&control;
	embedded_mesh=&embedded;

	
	weights.resize(embedded.numVertex());

#ifdef USE_MAX_NUM_WEIGHTS
	indexes.resize(embedded.numVertex());
	for(int i=0; i<embedded.numVertex(); i++)
	{
		MeanValueWeights(control, max_num_weights, indexes[i], weights[i], embedded.getVertex(i));
	}
#else
	for(int i=0; i<embedded.numVertex(); i++)
		MeanValueWeights(control, weights[i], embedded.getVertex(i));
#endif
}

/*void MeanValueMeshCoords::transfer(OBJloader::Mesh const& control)
{
	ASSERT(control.numVertex()==control_mesh->numVertex());
#ifdef USE_MAX_NUM_WEIGHTS
	for(int i=0; i<embedded_mesh->numVertex(); i++)
	{
		vector3& target=embedded_mesh->getVertex(i);
		target.setValue(0,0,0);

		for(int j=0; j<weights[i].size(); j++)
			target+=control.getVertex(indexes[i][j])*weights[i][j];
	}
#else
	for(int i=0; i<embedded_mesh->numVertex(); i++)
	{
		vector3& target=embedded_mesh->getVertex(i);
		target.setValue(0,0,0);

		for(int j=0; j<weights[i].size(); j++)
			target+=control.getVertex(j)*weights[i][j];
	}
#endif
}
*/


class MeanValueCoords :public LinearDeformer {
public:
	MeanValueCoords():LinearDeformer() {}
	~MeanValueCoords() {}

	virtual void calculateCorrespondence(OBJloader::Mesh & control, vector3N& embedded);
};



LinearDeformer* createMeanValueDeformer()
{
	return new MeanValueCoords();
}

void MeanValueCoords::calculateCorrespondence(OBJloader::Mesh & control, vector3N & embedded)
{
	int max_num_weights=100;
	control_mesh=&control;
	embedded_vertices=&embedded;

	
	weights.resize(embedded.size());

#ifdef USE_MAX_NUM_WEIGHTS
	indexes.resize(embedded.numVertex());
	for(int i=0; i<embedded.numVertex(); i++)
	{
		MeanValueWeights(control, max_num_weights, indexes[i], weights[i], embedded.getVertex(i));
	}
#else
	for(int i=0; i<embedded.size(); i++)
		MeanValueWeights(control, weights[i], embedded(i));
#endif
}
class IdentityTF :public LinearDeformer {
public:
	IdentityTF():LinearDeformer() 
	{
	}
	~IdentityTF() {}

	virtual void calculateCorrespondence(OBJloader::Mesh & control, vector3N& embedded)
	{
		ASSERT(control.numVertex()==embedded.size());
	}
	void transfer(OBJloader::Mesh const& control, double scale_factor)
	{
		for(int i=0; i<embedded_vertices->size(); i++)
		{
			vector3& target=(*embedded_vertices)(i);
			target=control.getVertex(i);
		}
	}
};
static bool initialFaceDic = false;
class KNNIDWCoords :public LinearDeformer {
	InterpolationNormalize* reg;
	//KNearestInterpolation* reg;
	//KNearestInterpolationFast* reg;
	vector3N delta;
	vector3N beforeEmbedded;
	intvectorn adjacentFace;

public:
	KNNIDWCoords(int m_nK, float m_fK, float m_NW):LinearDeformer() 
	{
		reg=new KNearestInterpolation(new L2Metric(), m_nK, m_fK , m_NW);
		//reg=new RBInterpolation();
	}
	KNNIDWCoords(int m_nK, float m_NW):LinearDeformer() 
	{
		reg=new KNearestInterpolation(new L2Metric(), m_nK, 2.0, m_NW );
		//reg=new KNearestInterpolationFast(m_nK, 2.0, m_fK);
	}
	~KNNIDWCoords() {}

	virtual void calculateCorrespondence(OBJloader::Mesh & control, vector3N& embedded)
	{
		control_mesh=&control;
		embedded_vertices=&embedded;

		matrixn sources(control.numVertex(), 3);
		for(int i=0; i<control.numVertex(); i++)
			sources.row(i).setVec3(0, control.getVertex(i));

		reg->initialize(sources);

		weights.resize(embedded.size());
		vectorn v(3);
		vectorn w;
		intvectorn id;
		for(int i=0; i<embedded.size(); i++)
		{
			v.setVec3(0, embedded(i));
			reg->calcWeight(v, id, w);
			auto& ww=weights[i];
			ww.setSize(control.numVertex());
			ww.setAllValue(0.0);
			for(int j=0; j<id.size(); j++)
				ww[id[j]]=w[j];
		}

		// save entered mFeatureMesh
		beforeEmbedded.setSize(embedded.size());
		delta.setSize(embedded.size());
		for (int i=0; i<embedded.size(); i++)
		{
			vector3 target(0,0,0);

			for(int j=0; j<weights[i].size(); j++)
				target+=control.getVertex(j)*weights[i][j];
			delta(i)=embedded(i)-target;
			beforeEmbedded(i) = target;
			//delta(i)=vector3(0,0,0);
		}
	}
	void transfer(OBJloader::Mesh const& control, double scale_factor)
	{
		//std::cout << "Transfer in KNNIDWCoord" << std::endl;
		LinearDeformer::transfer(control, scale_factor);

		// TODO
		// 비교대상 선정 => beforeEmbed, target(Delta 적용이전의 tet_to_obj)
		// Tet_to_obj is Mesh, so i can get face_info from tet_to_obj
		matrixn source, output;
		source.setSize(3,3);
		output.setSize(3,3);

		matrix4 R;
		vector3N deltaTemp = delta;
#if 0
		// Initialize Adjacent Face
		if(!initialFaceDic) {

			adjacentFace.setSize(embedded_vertices->size()*3);

			for(int i=0; i<embedded_vertices->size(); ++i) {

				for(int j=0; j<embedded_mesh->numFace(); ++j) {

					if(embedded_mesh->getFace(j).isVertex(i)) {

						int temp=embedded_mesh->getFace(j).vertexIndex(0);
						adjacentFace.set(3*i+0,temp);
						temp=embedded_mesh->getFace(j).vertexIndex(1);
						adjacentFace.set(3*i+1,temp);
						temp=embedded_mesh->getFace(j).vertexIndex(2);
						adjacentFace.set(3*i+2,temp);
						break;
					}
				}
			}

			initialFaceDic = true;
		}

		for (int i=0; i< embedded_vertices->size(); ++i)
		{
			// 2) set R
			for(int j=0; j<3; j++)
			{
				source.setColumn(j, vectorn(beforeEmbedded(adjacentFace[3*i+j])) );
				output.setColumn(j, vectorn((*embedded_vertices)(adjacentFace[3*i+j])) );
			}
			std::cout << std::endl;

			point_to_point(source, output, R);

			// Translation을 제외
			for(int j=0; j<3; j++)
				R.m[j][3] = 0;

			for(int j=0; j<3; j++) {
				for(int k=0; k<3; k++) {
					std::cout << R.m[j][k] << " ";
				}
				std::cout << std::endl;
			}

			deltaTemp(i) = R * delta(i);
		}

		for(int i=0; i<embedded_vertices->size(); i++) {
			vector3 &target = (*embedded_vertices)(i);

			target+=deltaTemp(i);
		}


#elif 0
		// For Check Point to Point func work well
		for (int i=0; i< beforeEmbedded.size()/3; ++i) {

			// 2) set R
			std::cout<<std::endl;
			std::cout << "Testing"<<std::endl;
			for(int j=0; j < 3; ++j) {
				source.setColumn(j, vectorn(beforeEmbedded(3*i + j)));
				output.setColumn(j, vectorn((*embedded_vertices)(3*i + j)));
				std::cout << vectorn((*embedded_vertices)(3*i + j)) << " : " << vectorn(beforeEmbedded(3*i + j)) << std::endl;
			}

			point_to_point(source, output, R);

			// For check R*source -> Output
			for(int j=0; j < 3; ++j) {
				vector3 &target = (*embedded_vertices)(3*i + j);
				vector3 &input = beforeEmbedded(3*i + j);

				auto t = vector3(input[0], input[1], input[2]);
				std::cout << "output : " << target << " , source : " << t << std::endl;
				std::cout << "Result: " << R * t << std::endl;
			}
			std::cout<< std::endl;

			// Translation을 제외
			for(int j=0; j<3; j++)
				R.m[j][3] = 0;

			for(int j=0; j<3; j++)
			{
				vector3& target=(*embedded_vertices)(3*i + j);

				// todo: rotate delta according to local deformation. e.g. quater R=estimateLocalDeform(i); target+=R*delta(i);
				target +=  R * delta(3*i + j);
			}
		}

#else
		for(int i=0; i<embedded_vertices->size(); i++)
		{
			vector3& target=(*embedded_vertices)(i);
			// todo: rotate delta according to local deformation. e.g. quater R=estimateLocalDeform(i); target+=R*delta(i);
			target+=delta(i);
		}

#endif
	}
};
LinearDeformer* createIdentityDeformer()
{
	return new IdentityTF();
}
LinearDeformer* createKNNIDWDeformer(int m_nK, float m_fK, float m_NW)
{
	return new KNNIDWCoords(m_nK, m_fK, m_NW);
}

LinearDeformer* createKNNIDWDeformer(int m_nK, float m_NW)
{
	return new KNNIDWCoords(m_nK, m_NW);
}

InverseDeformer::InverseDeformer(LinearMeshDeformer* deformer, vector3N const& embedded_orig)
{
	_embedded_orig=embedded_orig;
	_deformer=deformer;
	// w11 w12 w13 ...
	// w21 w22 w23 ...
	// ...
	int nv=((LinearMeshDeformer *) deformer)->numVerts();
	int ntv=((LinearMeshDeformer *) deformer)->getWeights(0).size();

	matrixn W(nv * 3, ntv * 3);
	W.setAllValue(0.0);
	for (int i = 0; i < nv; i++) {
		vectorn &w = ((LinearMeshDeformer *) deformer)->getWeights(i);
		for (int j = 0; j < ntv; j++) {
			double wj = w(j);
			W(i * 3, j * 3) = wj;
			W(i * 3 + 1, j * 3 + 1) = wj;
			W(i * 3 + 2, j * 3 + 2) = wj;
		}
	}

	m::pseudoInverse(invW, W);
}

void InverseDeformer::invDeform(vector3N const& embedded, OBJloader::Mesh const& origControl, OBJloader::Mesh & currControl)
{
	const vector3N * mFeaturePoints=&embedded;
	const vector3N * mFeaturePoints_originalShape=&_embedded_orig;
	// start_optimize
	vectorn targetDeform(mFeaturePoints->size() * 3);
	for (int i = 0; i < mFeaturePoints->size(); i++)
		targetDeform.setVec3(i * 3, (*mFeaturePoints)(i) - (*mFeaturePoints_originalShape)(i));
	vectorn controlDeform(invW.rows());

	Msg::verify(targetDeform.size() == invW.cols(), "a");
	Msg::verify(controlDeform.size() == invW.rows(), "b");
	controlDeform.column().mult(invW, targetDeform.column());

	int surface_vtx_num = currControl.numVertex();
	for (int i = 0; i < surface_vtx_num; i++) {

		double gain = 1;
		currControl.getVertex(i)= origControl.getVertex(i) + gain * controlDeform.toVector3(i * 3);
	}
}
