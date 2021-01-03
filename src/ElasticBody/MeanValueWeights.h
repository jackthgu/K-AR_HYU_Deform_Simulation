#pragma once

//! compute mvmcs

class MeshDeformer
{
public:
	MeshDeformer(){}
	virtual ~MeshDeformer(){}

	virtual void calculateCorrespondence(OBJloader::Mesh & control, OBJloader::Mesh & embedded)=0;
	virtual void transfer(OBJloader::Mesh const& control, double scale_factor)=0;


	virtual bool saveCorrespondence(const char* filename){Msg::msgBox("saveCorrespondnece");return false;}
	virtual bool loadCorrespondence(const char* filename, OBJloader::Mesh & control, OBJloader::Mesh& embedded)	{Msg::msgBox("loadCorrespondnece");return false;}
};

class LinearMeshDeformer: public MeshDeformer
{
protected:
	OBJloader::Mesh* control_mesh;
	OBJloader::Mesh* embedded_mesh;
	std::vector<vectorn> weights;
public:
	LinearMeshDeformer();
	~LinearMeshDeformer();
	vectorn& getWeights(int ivert) { return weights[ivert];}
	int numVerts() { return weights.size();}
	void setEmbeddedMesh(OBJloader::Mesh* mesh) {embedded_mesh = mesh;}
	virtual bool saveCorrespondence(const char* filename);
	virtual bool loadCorrespondence(const char* filename, OBJloader::Mesh & control, OBJloader::Mesh& embedded);
	virtual void transfer(OBJloader::Mesh const& control, double scale_factor);
	virtual void transferTo(OBJloader::Mesh const& control, vector3N& out);

};

class LinearDeformer: public LinearMeshDeformer
{
protected:
	vector3N* embedded_vertices;
	virtual void calculateCorrespondence(OBJloader::Mesh & control, OBJloader::Mesh & embedded){ Msg::error("do not use this in this class");}
public:
	LinearDeformer();
	~LinearDeformer();
	virtual void calculateCorrespondence(OBJloader::Mesh & control, vector3N& embedded)=0;
	virtual bool loadCorrespondence(const char* filename, OBJloader::Mesh & control, vector3N& embedded);
	virtual void transfer(OBJloader::Mesh const& control, double scale_factor);
};

class InverseDeformer
{
	public:
	matrixn invW;
	LinearMeshDeformer* _deformer;
	vector3N _embedded_orig;

	InverseDeformer(LinearMeshDeformer* deformer, vector3N const& embedded_orig);
	void invDeform(vector3N const& embedded, OBJloader::Mesh const& origcontrol, OBJloader::Mesh & control);
};

MeshDeformer* createMeanValueMeshDeformer();
MeshDeformer* createHarmonicMeshDeformer();
MeshDeformer* createKNNIDWMeshDeformer(int m_nK, float m_fNW);
LinearDeformer* createMeanValueDeformer();
LinearDeformer* createHarmonicDeformer();
LinearDeformer* createKNNIDWDeformer(int m_nK, float m_fNW);
LinearDeformer* createKNNIDWDeformer(int m_nK, float m_fK, float m_NW);
