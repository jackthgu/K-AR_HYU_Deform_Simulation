#include "stdafx.h"
#include "MainLib/OgreFltk/Mesh.h"
#include "opennl/extern/ONL_opennl.h"
/************************** Harmonic Coordinates ****************************/
/* From "Harmonic Coordinates for Character Articulation",
	Pushkar Joshi, Mark Meyer, Tony DeRose, Brian Green and Tom Sanocki,
	SIGGRAPH 2007. */

#define EPSILON 0.0001f

#define MESHDEFORM_TAG_UNTYPED  0
#define MESHDEFORM_TAG_BOUNDARY 1
#define MESHDEFORM_TAG_INTERIOR 2
#define MESHDEFORM_TAG_EXTERIOR 3

#define MESHDEFORM_LEN_THRESHOLD 1e-6

#define MESHDEFORM_MIN_INFLUENCE 0.0005

static int MESHDEFORM_OFFSET[7][3] =
		{{0,0,0}, {1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1}};

typedef struct MDefBoundIsect {
	vector3 co;
	double uvw[4];
	int nvert, v[4], facing;
	double len;
} MDefBoundIsect;

/*
typedef struct MFace {
	unsigned int v1, v2, v3, v4;
	char pad, mat_nr;
	char edcode, flag;	// we keep edcode, for conversion to edges draw flags in old files 
} MFace;*/

/* struct for intersection data */
typedef struct Isect {
	vector3 start;			/* start+vec = end, in ray_tree_intersect */
	vector3 vec;
	vector3 end;			

	double labda, u, v;		/* distance to hitpoint, uv weights */

	OBJloader::Face* face;	/* face is where to intersect with */
//	RayFace *face;			/* face is where to intersect with */
//	int ob;
//	RayFace *faceorig;		/* start face */
//	int oborig;
//	RayFace *face_last;		/* for shadow optimize, last intersected face */
//	int ob_last;

	short isect;			/* which half of quad */
	enum {RE_RAY_SHADOW, RE_RAY_MIRROR, RE_RAY_SHADOW_TRA };
	short mode;				/* RE_RAY_SHADOW, RE_RAY_MIRROR, RE_RAY_SHADOW_TRA */
	int lay;				/* -1 default, set for layer lamps */

	/* only used externally */
	double col[4];			/* RGBA for shadow_tra */

	/* octree only */
//	RayFace *facecontr;
	int obcontr;
	double ddalabda;
	short faceisect;		/* flag if facecontr was done or not */

	/* custom pointer to be used in the RayCheckFunc */
	void *userdata;
} Isect;

typedef struct MDefBindInfluence {
	struct MDefBindInfluence *next;
	double weight;
	int vertex;
} MDefBindInfluence;

typedef struct MDefInfluence {
	int vertex;
	double weight;
} MDefInfluence;
#ifdef USE_DYNBIND
typedef struct MDefCell {
	int offset;
	int totinfluence;
} MDefCell;
#endif

typedef struct MeshDeformModifierData {
	//ModifierData modifier;

	//struct Object *object;			/* mesh object */
	OBJloader::Mesh* object;
	char defgrp_name[32];			/* optional vertexgroup name */

	short gridsize, needbind;
	short flag, pad;

	/* variables filled in when bound */
	vectorn bindweights;/* computed binding weights */
	vector3 *bindcos;	
	int totvert, totcagevert;		/* total vertices in mesh and cage */
#ifdef USE_DYNBIND
	MDefCell *dyngrid;				/* grid with dynamic binding cell points */
	MDefInfluence *dyninfluences;	/* dynamic binding vertex influences */
	int *dynverts, *pad2;			/* is this vertex bound or not? */
	int dyngridsize;				/* size of the dynamic bind grid */
	int totinfluence;				/* total number of vertex influences */
	double dyncellmin[3];			/* offset of the dynamic bind grid */
	double dyncellwidth;				/* width of dynamic bind cell */

#endif
	//matrix4 bindmat;			/* matrix of cage at binding time */
} MeshDeformModifierData;
struct MDefBoundIsectArray
{
	MDefBoundIsect *__array[6];

	MDefBoundIsectArray()
	{
		for(int i=0; i<6; i++)
			__array[i]=NULL;
	}

	MDefBoundIsect *& operator[](int n)
	{
		return __array[n];
	}
};
typedef struct MeshDeformBind {
	
	MeshDeformBind ()
	{
		width[0]=width[1]=width[2]=0;
		halfwidth[0]=halfwidth[1]=halfwidth[2]=0;
		size=0; size3=0;
		cagedm=NULL;
		cagecos=NULL;
		vertexcos=NULL;
		totvert=0, totcagevert=0;
		dyngrid=NULL;
		varidx=NULL;
	}
	/* grid dimensions */
	vector3 min, max;
	double width[3], halfwidth[3];
	int size, size3;

	/* meshes */
	OBJloader::Mesh *cagedm;
	vector3* cagecos;
	vector3* vertexcos;
	int totvert, totcagevert;

	/* grids */
	std::vector<MDefBoundIsectArray> boundisect;
	intvectorn semibound;
	intvectorn tag;
	vectorn phi;
	vectorn totalphi;

	/* mesh stuff */
	vectorn inside;
	vectorn weights;
	MDefBindInfluence **dyngrid;
	matrix4 cagemat;

	/* direct solver */
	int *varidx;

#if 0
	/* raytrace */
	RayTree *raytree;
#endif
} MeshDeformBind;

inline void INIT_MINMAX(vector3& min, vector3& max)
{
	min.setValue(FLT_MAX, FLT_MAX,FLT_MAX);
	max.setValue(-1.0*FLT_MAX, -1.0*FLT_MAX, -1.0*FLT_MAX);
}

inline void DO_MINMAX(vector3& newv, vector3& min, vector3& max)
{
	min.x=MIN(min.x, newv.x);
	min.y=MIN(min.y, newv.y);
	min.z=MIN(min.z, newv.z);

	max.x=MAX(max.x, newv.x);
	max.y=MAX(max.y, newv.y);
	max.z=MAX(max.z, newv.z);
}


inline void VECSUB(vector3& c, vector3& a, vector3& b)
{
	c.sub(a,b);
}
inline void Mat4MulVecfl(matrix4& mat, vector3& vec)
{
	vec.leftMult(mat);
}

inline void VECADD(vector3& c, vector3& a, vector3& b)
{
	c.add(a,b);
}

inline double VecLenf(vector3& a, vector3& b)
{
	return a.distance(b);
}

inline void Crossf(vector3& c, vector3& a, vector3& b)
{
	c.cross(a,b);
}

inline double INPR(vector3& a, vector3& b)
{
	return a%b;
}

inline void VECCOPY(vector3& a, vector3& b)
{
	a=b;
}

inline void CalcNormdouble( vector3&v1, vector3&v2, vector3&v3, vector3& n)
{
	vector3 n1, n2;

	n1[0]= v1[0]-v2[0];
	n2[0]= v2[0]-v3[0];
	n1[1]= v1[1]-v2[1];
	n2[1]= v2[1]-v3[1];
	n1[2]= v1[2]-v2[2];
	n2[2]= v2[2]-v3[2];
	n[0]= n1[1]*n2[2]-n1[2]*n2[1];
	n[1]= n1[2]*n2[0]-n1[0]*n2[2];
	n[2]= n1[0]*n2[1]-n1[1]*n2[0];
	n.normalize();
}

/* ray intersection */

/* our own triangle intersection, so we can fully control the epsilons and
 * prevent corner case from going wrong*/
static int meshdeform_tri_intersect(vector3& orig, vector3& end, vector3& vert0,
    vector3& vert1, vector3& vert2, vector3& isectco, double *uvw)
{
	double det,inv_det, u, v;

	vector3 tvec,pvec, qvec, isectdir, dir,edge1, edge2;

	VECSUB(dir, end, orig);

	/* find vectors for two edges sharing vert0 */
	VECSUB(edge1, vert1, vert0);
	VECSUB(edge2, vert2, vert0);

	/* begin calculating determinant - also used to calculate U parameter */
	Crossf(pvec, dir, edge2);

	/* if determinant is near zero, ray lies in plane of triangle */
	det = INPR(edge1, pvec);

	if (det == 0.0f)
	  return 0;
	inv_det = 1.0f / det;

	/* calculate distance from vert0 to ray origin */
	VECSUB(tvec, orig, vert0);

	/* calculate U parameter and test bounds */
	u = INPR(tvec, pvec) * inv_det;
	if (u < -EPSILON || u > 1.0f+EPSILON)
	  return 0;

	/* prepare to test V parameter */
	Crossf(qvec, tvec, edge1);

	/* calculate V parameter and test bounds */
	v = INPR(dir, qvec) * inv_det;
	if (v < -EPSILON || u + v > 1.0f+EPSILON)
	  return 0;

	isectco[0]= (1.0f - u - v)*vert0[0] + u*vert1[0] + v*vert2[0];
	isectco[1]= (1.0f - u - v)*vert0[1] + u*vert1[1] + v*vert2[1];
	isectco[2]= (1.0f - u - v)*vert0[2] + u*vert1[2] + v*vert2[2];

	uvw[0]= 1.0 - u - v;
	uvw[1]= u;
	uvw[2]= v;

	/* check if it is within the length of the line segment */
	VECSUB(isectdir, isectco, orig);

	if(INPR(dir, isectdir) < -EPSILON)
		return 0;
	
	if(INPR(dir, dir) + EPSILON < INPR(isectdir, isectdir))
		return 0;

	return 1;
}

/* blender's raytracer is not use now, even though it is much faster. it can
 * give problems with rays falling through, so we use our own intersection 
 * function above with tweaked epsilons */

#if 0
static MeshDeformBind *MESHDEFORM_BIND = NULL;

static void meshdeform_ray_coords_func(RayFace *face, double **v1, double **v2, double **v3, double **v4)
{
	MFace *mface= (MFace*)face;
	double (*cagecos)[3]= MESHDEFORM_BIND->cagecos;

	*v1= cagecos[mface->v1];
	*v2= cagecos[mface->v2];
	*v3= cagecos[mface->v3];
	*v4= (mface->v4)? cagecos[mface->v4]: NULL;
}

static int meshdeform_ray_check_func(Isect *is, RayFace *face)
{
	return 1;
}

static void meshdeform_ray_tree_create(MeshDeformBind *mdb)
{
	MFace *mface;
	double min[3], max[3];
	int a, totface;

	/* create a raytrace tree from the mesh */
	INIT_MINMAX(min, max);

	for(a=0; a<mdb->totcagevert; a++)
		DO_MINMAX(mdb->cagecos[a], min, max)

	MESHDEFORM_BIND= mdb;

	mface= mdb->cagedm->getFaceArray(mdb->cagedm);
	totface= mdb->cagedm->getNumFaces(mdb->cagedm);

	mdb->raytree= RE_ray_tree_create(64, totface, min, max,
		meshdeform_ray_coords_func, meshdeform_ray_check_func);

	for(a=0; a<totface; a++, mface++)
		RE_ray_tree_add_face(mdb->raytree, mface);

	RE_ray_tree_done(mdb->raytree);
}

static void meshdeform_ray_tree_free(MeshDeformBind *mdb)
{
	MESHDEFORM_BIND= NULL;
	RE_ray_tree_free(mdb->raytree);
}
#endif

static int meshdeform_intersect(MeshDeformBind *mdb, Isect *isec)
{
	OBJloader::Face* mface;
	vector3 face[4], co, nor;
	double uvw[3], len;
	int f, hit, is= 0, totface;

	isec->labda= 1e10;

	totface= mdb->cagedm->numFace();

	for(f=0; f<totface; f++) {
		mface=&mdb->cagedm->getFace(f);
		VECCOPY(face[0], mdb->cagecos[mface->vi(0)]);
		VECCOPY(face[1], mdb->cagecos[mface->vi(1)]);
		VECCOPY(face[2], mdb->cagecos[mface->vi(2)]);

		hit= meshdeform_tri_intersect(isec->start, isec->end, face[0], face[1], face[2], co, uvw);
		CalcNormdouble(face[0], face[1], face[2], nor);

		if(hit) {
			len= VecLenf(isec->start, co)/VecLenf(isec->start, isec->end);
			if(len < isec->labda) {
				isec->labda= len;
				isec->face= mface;
				isec->isect= (INPR(isec->vec, nor) <= 0.0f);
				is= 1;
			}
		}
	}

	return is;
}

/* Mean value weights - smooth interpolation weights for polygons with
 * more than 3 vertices */
double MeanValueHalfTan(vector3& v1, vector3& v2, vector3& v3)
{
	vector3 d3, d2, cross;
	double area, dot, len;

	VECSUB(d2, v2, v1);
	VECSUB(d3, v3, v1);
	Crossf(cross, d2, d3);

	area= cross.length();
	dot= d2%d3;
	len= d2.length()*d3.length();

	if(area == 0.0f)
		return 0.0f;
	else
		return (len - dot)/area;
}


void MeanValueWeights(vector3* v, int n, vector3& co, double *w)
{
	double totweight, t1, t2, len;
	vector3 vmid, vprev, vnext;
	int i;

	totweight= 0.0f;

	for(i=0; i<n; i++) {
		vmid= v[i];
		vprev= (i == 0)? v[n-1]: v[i-1];
		vnext= (i == n-1)? v[0]: v[i+1];

		t1= MeanValueHalfTan(co, vprev, vmid);
		t2= MeanValueHalfTan(co, vmid, vnext);

		len= VecLenf(co, vmid);
		w[i]= (t1+t2)/len;
		totweight += w[i];
	}

	if(totweight != 0.0f)
		for(i=0; i<n; i++)
			w[i] /= totweight;
}



static MDefBoundIsect *meshdeform_ray_tree_intersect(MeshDeformBind *mdb, vector3& co1, vector3& co2)
{
	MDefBoundIsect *isect;
	Isect isec;
	vector3* cagecos;
	OBJloader::Face* mface;
	vector3 vert[4];
	double len;
	static vector3 epsilon= vector3(0, 0, 0); //1e-4, 1e-4, 1e-4};

	/* setup isec */
	memset(&isec, 0, sizeof(isec));
	isec.mode= Isect::RE_RAY_MIRROR; /* we want the closest intersection */
	isec.lay= -1;
//	isec.face_last= NULL;
//	isec.faceorig= NULL;
	isec.labda= 1e10f;

	VECADD(isec.start, co1, epsilon);
	VECADD(isec.end, co2, epsilon);
	VECSUB(isec.vec, isec.end, isec.start);

#if 0
	/*if(RE_ray_tree_intersect(mdb->raytree, &isec)) {*/
#endif

	if(meshdeform_intersect(mdb, &isec)) {
		len= isec.labda;
		mface= isec.face;

		/* create MDefBoundIsect */
		//isect= BLI_memarena_alloc(mdb->memarena, sizeof(*isect));
		isect=new MDefBoundIsect ();

		/* compute intersection coordinate */
		isect->co[0]= co1[0] + isec.vec[0]*len;
		isect->co[1]= co1[1] + isec.vec[1]*len;
		isect->co[2]= co1[2] + isec.vec[2]*len;

		isect->len= VecLenf(co1, isect->co);
		if(isect->len < MESHDEFORM_LEN_THRESHOLD)
			isect->len= MESHDEFORM_LEN_THRESHOLD;

		isect->v[0]= mface->vi(0);
		isect->v[1]= mface->vi(1);
		isect->v[2]= mface->vi(2);
		
		isect->nvert= 3;

		isect->facing= isec.isect;

		/* compute mean value coordinates for interpolation */
		cagecos= mdb->cagecos;
		VECCOPY(vert[0], cagecos[mface->vi(0)]);
		VECCOPY(vert[1], cagecos[mface->vi(1)]);
		VECCOPY(vert[2], cagecos[mface->vi(2)]);
		MeanValueWeights(vert, isect->nvert, isect->co, isect->uvw);

		return isect;
	}

	return NULL;
}

static int meshdeform_inside_cage(MeshDeformBind *mdb, vector3& co)
{
	MDefBoundIsect *isect;
	vector3 outside, start, dir;
	int i, counter;

	for(i=1; i<=6; i++) {
		counter = 0;

		outside[0] = co[0] + (mdb->max[0] - mdb->min[0] + 1.0f)*MESHDEFORM_OFFSET[i][0];
		outside[1] = co[1] + (mdb->max[1] - mdb->min[1] + 1.0f)*MESHDEFORM_OFFSET[i][1];
		outside[2] = co[2] + (mdb->max[2] - mdb->min[2] + 1.0f)*MESHDEFORM_OFFSET[i][2];

		VECCOPY(start, co);
		VECSUB(dir, outside, start);
		dir.normalize();
		
		isect = meshdeform_ray_tree_intersect(mdb, start, outside);
		if(isect && !isect->facing)
			return 1;
	}

	return 0;
}

/* solving */

static int meshdeform_index(MeshDeformBind *mdb, int x, int y, int z, int n)
{
	int size= mdb->size;
	
	x += MESHDEFORM_OFFSET[n][0];
	y += MESHDEFORM_OFFSET[n][1];
	z += MESHDEFORM_OFFSET[n][2];

	if(x < 0 || x >= mdb->size)
		return -1;
	if(y < 0 || y >= mdb->size)
		return -1;
	if(z < 0 || z >= mdb->size)
		return -1;

	return x + y*size + z*size*size;
}

static void meshdeform_cell_center(MeshDeformBind *mdb, int x, int y, int z, int n, double *center)
{
	x += MESHDEFORM_OFFSET[n][0];
	y += MESHDEFORM_OFFSET[n][1];
	z += MESHDEFORM_OFFSET[n][2];

	center[0]= mdb->min[0] + x*mdb->width[0] + mdb->halfwidth[0];
	center[1]= mdb->min[1] + y*mdb->width[1] + mdb->halfwidth[1];
	center[2]= mdb->min[2] + z*mdb->width[2] + mdb->halfwidth[2];
}

static void meshdeform_add_intersections(MeshDeformBind *mdb, int x, int y, int z)
{
	MDefBoundIsect *isect;
	vector3 center, ncenter;
	int i, a;

	a= meshdeform_index(mdb, x, y, z, 0);
	meshdeform_cell_center(mdb, x, y, z, 0, center);

	/* check each outgoing edge for intersection */
	for(i=1; i<=6; i++) {
		if(meshdeform_index(mdb, x, y, z, i) == -1)
			continue;

		meshdeform_cell_center(mdb, x, y, z, i, ncenter);

		isect= meshdeform_ray_tree_intersect(mdb, center, ncenter);
		if(isect) {
			mdb->boundisect[a][i-1]= isect;
			mdb->tag[a]= MESHDEFORM_TAG_BOUNDARY;
		}
	}
}

static void meshdeform_bind_floodfill(MeshDeformBind *mdb)
{
	int *stack;
	intvectorn& tag=mdb->tag;
	int a, b, i, xyz[3], stacksize, size= mdb->size;

	stack= new int[mdb->size3];//, "MeshDeformBindStack");

	/* we know lower left corner is EXTERIOR because of padding */
	tag[0]= MESHDEFORM_TAG_EXTERIOR;
	stack[0]= 0;
	stacksize= 1;

	/* floodfill exterior tag */
	while(stacksize > 0) {
		a= stack[--stacksize];

		xyz[2]= a/(size*size);
		xyz[1]= (a - xyz[2]*size*size)/size;
		xyz[0]= a - xyz[1]*size - xyz[2]*size*size;

		for(i=1; i<=6; i++) {
			b= meshdeform_index(mdb, xyz[0], xyz[1], xyz[2], i);

			if(b != -1) {
				if(tag[b] == MESHDEFORM_TAG_UNTYPED ||
				   (tag[b] == MESHDEFORM_TAG_BOUNDARY && !mdb->boundisect[a][i-1])) {
					tag[b]= MESHDEFORM_TAG_EXTERIOR;
					stack[stacksize++]= b;
				}
			}
		}
	}

	/* other cells are interior */
	for(a=0; a<size*size*size; a++)
		if(tag[a]==MESHDEFORM_TAG_UNTYPED)
			tag[a]= MESHDEFORM_TAG_INTERIOR;

#if 0
	{
		int tb, ti, te, ts;
		tb= ti= te= ts= 0;
		for(a=0; a<size*size*size; a++)
			if(tag[a]==MESHDEFORM_TAG_BOUNDARY)
				tb++;
			else if(tag[a]==MESHDEFORM_TAG_INTERIOR)
				ti++;
			else if(tag[a]==MESHDEFORM_TAG_EXTERIOR) {
				te++;

				if(mdb->semibound[a])
					ts++;
			}
		
		printf("interior %d exterior %d boundary %d semi-boundary %d\n", ti, te, tb, ts);
	}
#endif

	delete[] stack;
}

static double meshdeform_boundary_phi(MeshDeformBind *mdb, MDefBoundIsect *isect, int cagevert)
{
	int a;

	for(a=0; a<isect->nvert; a++)
		if(isect->v[a] == cagevert)
			return isect->uvw[a];
	
	return 0.0f;
}

static double meshdeform_interp_w(MeshDeformBind *mdb, double *gridvec, double *vec, int cagevert)
{
	double dvec[3], ivec[3], wx, wy, wz, result=0.0f;
	double weight, totweight= 0.0f;
	int i, a, x, y, z;

	for(i=0; i<3; i++) {
		ivec[i]= (int)gridvec[i];
		dvec[i]= gridvec[i] - ivec[i];
	}

	for(i=0; i<8; i++) {
		if(i & 1) { x= ivec[0]+1; wx= dvec[0]; }
		else { x= ivec[0]; wx= 1.0f-dvec[0]; } 

		if(i & 2) { y= ivec[1]+1; wy= dvec[1]; }
		else { y= ivec[1]; wy= 1.0f-dvec[1]; } 

		if(i & 4) { z= ivec[2]+1; wz= dvec[2]; }
		else { z= ivec[2]; wz= 1.0f-dvec[2]; } 

		CLAMP(x, 0, mdb->size-1);
		CLAMP(y, 0, mdb->size-1);
		CLAMP(z, 0, mdb->size-1);

		a= meshdeform_index(mdb, x, y, z, 0);
		weight= wx*wy*wz;
		result += weight*mdb->phi[a];
		totweight += weight;
	}

	if(totweight > 0.0f)
		result /= totweight;

	return result;
}

static void meshdeform_check_semibound(MeshDeformBind *mdb, int x, int y, int z)
{
	int i, a;

	a= meshdeform_index(mdb, x, y, z, 0);
	if(mdb->tag[a] != MESHDEFORM_TAG_EXTERIOR)
		return;

	for(i=1; i<=6; i++)
		if(mdb->boundisect[a][i-1]) 
			mdb->semibound[a]= 1;
}

static double meshdeform_boundary_total_weight(MeshDeformBind *mdb, int x, int y, int z)
{
	double weight, totweight= 0.0f;
	int i, a;

	a= meshdeform_index(mdb, x, y, z, 0);

	/* count weight for neighbour cells */
	for(i=1; i<=6; i++) {
		if(meshdeform_index(mdb, x, y, z, i) == -1)
			continue;

		if(mdb->boundisect[a][i-1])
			weight= 1.0f/mdb->boundisect[a][i-1]->len;
		else if(!mdb->semibound[a])
			weight= 1.0f/mdb->width[0];
		else
			weight= 0.0f;

		totweight += weight;
	}

	return totweight;
}

static void meshdeform_matrix_add_cell(MeshDeformBind *mdb, int x, int y, int z)
{
	MDefBoundIsect *isect;
	double weight, totweight;
	int i, a, acenter;

	acenter= meshdeform_index(mdb, x, y, z, 0);
	if(mdb->tag[acenter] == MESHDEFORM_TAG_EXTERIOR)
		return;

	nlMatrixAdd(mdb->varidx[acenter], mdb->varidx[acenter], 1.0f);
	
	totweight= meshdeform_boundary_total_weight(mdb, x, y, z);
	for(i=1; i<=6; i++) {
		a= meshdeform_index(mdb, x, y, z, i);
		if(a == -1 || mdb->tag[a] == MESHDEFORM_TAG_EXTERIOR)
			continue;

		isect= mdb->boundisect[acenter][i-1];
		if (!isect) {
			weight= (1.0f/mdb->width[0])/totweight;
			nlMatrixAdd(mdb->varidx[acenter], mdb->varidx[a], -weight);
		}
	}
}

static void meshdeform_matrix_add_rhs(MeshDeformBind *mdb, int x, int y, int z, int cagevert)
{
	MDefBoundIsect *isect;
	double rhs, weight, totweight;
	int i, a, acenter;

	acenter= meshdeform_index(mdb, x, y, z, 0);
	if(mdb->tag[acenter] == MESHDEFORM_TAG_EXTERIOR)
		return;

	totweight= meshdeform_boundary_total_weight(mdb, x, y, z);
	for(i=1; i<=6; i++) {
		a= meshdeform_index(mdb, x, y, z, i);
		if(a == -1)
			continue;

		isect= mdb->boundisect[acenter][i-1];

		if (isect) {
			weight= (1.0f/isect->len)/totweight;
			rhs= weight*meshdeform_boundary_phi(mdb, isect, cagevert);
			nlRightHandSideAdd(0, mdb->varidx[acenter], rhs);
		}
	}
}

static void meshdeform_matrix_add_semibound_phi(MeshDeformBind *mdb, int x, int y, int z, int cagevert)
{
	MDefBoundIsect *isect;
	double rhs, weight, totweight;
	int i, a;

	a= meshdeform_index(mdb, x, y, z, 0);
	if(!mdb->semibound[a])
		return;
	
	mdb->phi[a]= 0.0f;

	totweight= meshdeform_boundary_total_weight(mdb, x, y, z);
	for(i=1; i<=6; i++) {
		isect= mdb->boundisect[a][i-1];

		if (isect) {
			weight= (1.0f/isect->len)/totweight;
			rhs= weight*meshdeform_boundary_phi(mdb, isect, cagevert);
			mdb->phi[a] += rhs;
		}
	}
}

static void meshdeform_matrix_add_exterior_phi(MeshDeformBind *mdb, int x, int y, int z, int cagevert)
{
	double phi, totweight;
	int i, a, acenter;

	acenter= meshdeform_index(mdb, x, y, z, 0);
	if(mdb->tag[acenter] != MESHDEFORM_TAG_EXTERIOR || mdb->semibound[acenter])
		return;

	phi= 0.0f;
	totweight= 0.0f;
	for(i=1; i<=6; i++) {
		a= meshdeform_index(mdb, x, y, z, i);

		if(a != -1 && mdb->semibound[a]) {
			phi += mdb->phi[a];
			totweight += 1.0f;
		}
	}

	if(totweight != 0.0f)
		mdb->phi[acenter]= phi/totweight;
}

static void meshdeform_matrix_solve(MeshDeformBind *mdb)
{
	NLContext context;
	vector3 vec, gridvec;
	int a, b, x, y, z, totvar;
	char message[1024];

	/* setup variable indices */
	mdb->varidx= new int [mdb->size3];//, "MeshDeformDSvaridx");
	for(a=0, totvar=0; a<mdb->size3; a++)
		mdb->varidx[a]= (mdb->tag[a] == MESHDEFORM_TAG_EXTERIOR)? -1: totvar++;

	if(totvar == 0) {
		delete [] mdb->varidx;//MEM_freeN(
		return;
	}

	//progress_bar(0, "Starting mesh deform solve");

	/* setup opennl solver */
	nlNewContext();
	context= nlGetCurrent();

	nlSolverParameteri(NL_NB_VARIABLES, totvar);
	nlSolverParameteri(NL_NB_ROWS, totvar);
	nlSolverParameteri(NL_NB_RIGHT_HAND_SIDES, 1);

	nlBegin(NL_SYSTEM);
	nlBegin(NL_MATRIX);

	/* build matrix */
	for(z=0; z<mdb->size; z++)
		for(y=0; y<mdb->size; y++)
			for(x=0; x<mdb->size; x++)
				meshdeform_matrix_add_cell(mdb, x, y, z);

	/* solve for each cage vert */
	for(a=0; a<mdb->totcagevert; a++) {
		if(a != 0) {
			nlBegin(NL_SYSTEM);
			nlBegin(NL_MATRIX);
		}

		/* fill in right hand side and solve */
		for(z=0; z<mdb->size; z++)
			for(y=0; y<mdb->size; y++)
				for(x=0; x<mdb->size; x++)
					meshdeform_matrix_add_rhs(mdb, x, y, z, a);

		nlEnd(NL_MATRIX);
		nlEnd(NL_SYSTEM);

#if 0
		nlPrintMatrix();
#endif

		if(nlSolveAdvanced(NULL, NL_TRUE)) {
			for(z=0; z<mdb->size; z++)
				for(y=0; y<mdb->size; y++)
					for(x=0; x<mdb->size; x++)
						meshdeform_matrix_add_semibound_phi(mdb, x, y, z, a);

			for(z=0; z<mdb->size; z++)
				for(y=0; y<mdb->size; y++)
					for(x=0; x<mdb->size; x++)
						meshdeform_matrix_add_exterior_phi(mdb, x, y, z, a);

			for(b=0; b<mdb->size3; b++) {
				if(mdb->tag[b] != MESHDEFORM_TAG_EXTERIOR)
					mdb->phi[b]= nlGetVariable(0, mdb->varidx[b]);
				mdb->totalphi[b] += mdb->phi[b];
			}

			if(mdb->weights.size()) {
				/* static bind : compute weights for each vertex */
				for(b=0; b<mdb->totvert; b++) {
					if(mdb->inside[b]) {
						VECCOPY(vec, mdb->vertexcos[b]);

						Mat4MulVecfl(mdb->cagemat, vec);
						gridvec[0]= (vec[0] - mdb->min[0] - mdb->halfwidth[0])/mdb->width[0];
						gridvec[1]= (vec[1] - mdb->min[1] - mdb->halfwidth[1])/mdb->width[1];
						gridvec[2]= (vec[2] - mdb->min[2] - mdb->halfwidth[2])/mdb->width[2];

						mdb->weights[b*mdb->totcagevert + a]= meshdeform_interp_w(mdb, gridvec, vec, a);
					}
				}
			}
			else {
				MDefBindInfluence *inf;

				/* dynamic bind */
				for(b=0; b<mdb->size3; b++) {
					if(mdb->phi[b] >= MESHDEFORM_MIN_INFLUENCE) {
						inf= new MDefBindInfluence();
						inf->vertex= a;
						inf->weight= mdb->phi[b];
						inf->next= mdb->dyngrid[b];
						mdb->dyngrid[b]= inf;
					}
				}
			}
		}
		else {
			Msg::error("Mesh Deform: failed to find solution.");
			break;
		}

		Msg::print2("Mesh deform solve %d / %d", a+1, mdb->totcagevert);
//		progress_bar((double)(a+1)/(double)(mdb->totcagevert), message);
		
	}

#if 0
	/* sanity check */
	for(b=0; b<mdb->size3; b++)
		if(mdb->tag[b] != MESHDEFORM_TAG_EXTERIOR)
			if(fabs(mdb->totalphi[b] - 1.0f) > 1e-4)
				printf("totalphi deficiency [%s|%d] %d: %.10f\n",
					(mdb->tag[b] == MESHDEFORM_TAG_INTERIOR)? "interior": "boundary", mdb->semibound[b], mdb->varidx[b], mdb->totalphi[b]);
#endif
	
	/* free */
	delete[]mdb->varidx;
	//MEM_freeN();

	nlDeleteContext(context);
}

void harmonic_coordinates_bind(MeshDeformModifierData *mmd, vector3* vertexcos, int totvert, matrix4& cagemat)
{
	MeshDeformBind mdb;
	MDefBindInfluence *inf;
	MDefInfluence *mdinf;
#ifdef USE_DYNBIND
	MDefCell *cell;
#endif
	vector3 center, vec;
	double maxwidth, totweight;
	int a, b, x, y, z, totinside, offset;

//	waitcursor(1);
//	start_progress_bar();

	//memset(&mdb, 0, sizeof(MeshDeformBind));

	/* get mesh and cage mesh */
	mdb.vertexcos= vertexcos;
	mdb.totvert= totvert;
	
	mdb.cagedm=mmd->object;
	mdb.totcagevert= mdb.cagedm->numVertex();
	mdb.cagecos= new vector3[mdb.totcagevert];//, "MeshDeformBindCos");
	mdb.cagemat= cagemat;

	for(a=0; a<mdb.totcagevert; a++)
		VECCOPY(mdb.cagecos[a], mdb.cagedm->getVertex(a));

	/* compute bounding box of the cage mesh */
	INIT_MINMAX(mdb.min, mdb.max);

	for(a=0; a<mdb.totcagevert; a++)
		DO_MINMAX(mdb.cagecos[a], mdb.min, mdb.max);

	/* allocate memory */
	mdb.size= (2<<(mmd->gridsize-1)) + 2;
	mdb.size3= mdb.size*mdb.size*mdb.size;
	mdb.tag.setSize(mdb.size3);//, "MeshDeformBindTag");
	mdb.tag.setAllValue(0);
	mdb.phi.setSize(mdb.size3);//, "MeshDeformBindPhi");
	mdb.phi.setAllValue(0);
	mdb.totalphi.setSize(mdb.size3);//, "MeshDeformBindTotalPhi");
	mdb.totalphi.setAllValue(0);
	mdb.boundisect.resize(mdb.size3);//, "MDefBoundIsect");
	mdb.semibound.setSize(mdb.size3);//, "MDefSemiBound");
	mdb.semibound.setAllValue(0);

	mdb.inside.setSize(mdb.totvert);//, "MDefInside");
	mdb.inside.setAllValue(0);
#ifdef USE_DYNBIND
	if(mmd->flag & MOD_MDEF_DYNAMIC_BIND)
		mdb.dyngrid= MEM_callocN(sizeof(MDefBindInfluence*)*mdb.size3, "MDefDynGrid");
	else
#endif
	mdb.weights.setSize(mdb.totvert*mdb.totcagevert);//, "MDefWeights");
	mdb.weights.setAllValue(0);

	//mdb.memarena= BLI_memarena_new(BLI_MEMARENA_STD_BUFSIZE);
	//BLI_memarena_use_calloc(mdb.memarena);

	/* make bounding box equal size in all directions, add padding, and compute
	 * width of the cells */
	maxwidth = -1.0f;
	for(a=0; a<3; a++)
		if(mdb.max[a]-mdb.min[a] > maxwidth)
			maxwidth= mdb.max[a]-mdb.min[a];

	for(a=0; a<3; a++) {
		center[a]= (mdb.min[a]+mdb.max[a])*0.5f;
		mdb.min[a]= center[a] - maxwidth*0.5f;
		mdb.max[a]= center[a] + maxwidth*0.5f;

		mdb.width[a]= (mdb.max[a]-mdb.min[a])/(mdb.size-4);
		mdb.min[a] -= 2.1f*mdb.width[a];
		mdb.max[a] += 2.1f*mdb.width[a];

		mdb.width[a]= (mdb.max[a]-mdb.min[a])/mdb.size;
		mdb.halfwidth[a]= mdb.width[a]*0.5f;
	}

	//progress_bar(0, "Setting up mesh deform system");

#if 0
	/* create ray tree */
	meshdeform_ray_tree_create(&mdb);
#endif

	totinside= 0;
	for(a=0; a<mdb.totvert; a++) {
		VECCOPY(vec, mdb.vertexcos[a]);
		Mat4MulVecfl(mdb.cagemat, vec);
		mdb.inside[a]= meshdeform_inside_cage(&mdb, vec);
		if(mdb.inside[a])
			totinside++;
	}

	/* free temporary MDefBoundIsects */
	//BLI_memarena_free(mdb.memarena);
	//mdb.memarena= BLI_memarena_new(BLI_MEMARENA_STD_BUFSIZE);

	/* start with all cells untyped */
	for(a=0; a<mdb.size3; a++)
		mdb.tag[a]= MESHDEFORM_TAG_UNTYPED;
	
	/* detect intersections and tag boundary cells */
	for(z=0; z<mdb.size; z++) {
		std::cout << z << "/" << mdb.size << " finished\r"; // Takes too long for huge model.
		std::cout.flush();
		for (y = 0; y < mdb.size; y++)
			for (x = 0; x < mdb.size; x++)
				meshdeform_add_intersections(&mdb, x, y, z);
	}

#if 0
	/* free ray tree */
	meshdeform_ray_tree_free(&mdb);
#endif

	/* compute exterior and interior tags */
	meshdeform_bind_floodfill(&mdb);

	for(z=0; z<mdb.size; z++)
		for(y=0; y<mdb.size; y++)
			for(x=0; x<mdb.size; x++)
				meshdeform_check_semibound(&mdb, x, y, z);

	/* solve */
	meshdeform_matrix_solve(&mdb);

	/* assign results */
	mmd->bindcos= mdb.cagecos;
	mmd->totvert= mdb.totvert;
	mmd->totcagevert= mdb.totcagevert;
	//Mat4CpyMat4(mmd->bindmat, mmd->object->obmat);

#ifdef USE_DYNBIND
	if(mmd->flag & MOD_MDEF_DYNAMIC_BIND) {
		mmd->totinfluence= 0;
		for(a=0; a<mdb.size3; a++)
			for(inf=mdb.dyngrid[a]; inf; inf=inf->next)
				mmd->totinfluence++;

		/* convert MDefBindInfluences to smaller MDefInfluences */
		mmd->dyngrid= MEM_callocN(sizeof(MDefCell)*mdb.size3, "MDefDynGrid");
		mmd->dyninfluences= MEM_callocN(sizeof(MDefInfluence)*mmd->totinfluence, "MDefInfluence");
		offset= 0;
		for(a=0; a<mdb.size3; a++) {
			cell= &mmd->dyngrid[a];
			cell->offset= offset;

			to
				tweight= 0.0f;
			mdinf= mmd->dyninfluences + cell->offset;
			for(inf=mdb.dyngrid[a]; inf; inf=inf->next, mdinf++) {
				mdinf->weight= inf->weight;
				mdinf->vertex= inf->vertex;
				totweight += mdinf->weight;
				cell->totinfluence++;
			}

			if(totweight > 0.0f) {
				mdinf= mmd->dyninfluences + cell->offset;
				for(b=0; b<cell->totinfluence; b++, mdinf++)
					mdinf->weight /= totweight;
			}

			offset += cell->totinfluence;
		}

		mmd->dynverts= mdb.inside;
		mmd->dyngridsize= mdb.size;
		VECCOPY(mmd->dyncellmin, mdb.min);
		mmd->dyncellwidth= mdb.width[0];
		MEM_freeN(mdb.dyngrid);
	}
	else 
#endif
	{

		mmd->bindweights= mdb.weights;
//		delete[] mdb.inside;
	}

	/*
	// transform bindcos to world space 
	for(a=0; a<mdb.totcagevert; a++)
		Mat4MulVecfl(mmd->object->obmat, mmd->bindcos+a*3);
	*/
	
	/* free */
//	mdb.cagedm->release(mdb.cagedm);
//	delete[]mdb.tag;
//	delete[]mdb.phi;
//	delete[]mdb.totalphi;
//	delete[]mdb.boundisect;
//	delete[]mdb.semibound;
	//BLI_memarena_free(mdb.memarena);

	//end_progress_bar();
	//waitcursor(0);
}

#include "MeanValueWeights.h"
class HarmonicCoordMeshDeformer: public LinearMeshDeformer
{
	MeshDeformModifierData mmd;
	matrix4 cagemat;
	vector3N vertexcos;
public:
	HarmonicCoordMeshDeformer():LinearMeshDeformer(){}
	virtual void calculateCorrespondence(OBJloader::Mesh & control, OBJloader::Mesh & embedded)
	{
		control_mesh=&control;
		embedded_mesh=&embedded;
		mmd.gridsize=7;
		mmd.object=&control;
		cagemat.identity();
		
		vertexcos.setSize(embedded.numVertex());
		for(int i=0; i<vertexcos.size(); i++)
			vertexcos[i]=embedded.getVertex(i);

		std::cout << "binding" << std::endl;
		harmonic_coordinates_bind(&mmd, vertexcos, vertexcos.size(), cagemat);
		std::cout << "finish" << std::endl;

		weights.resize(embedded.numVertex());
		for(int i=0; i<embedded.numVertex(); i++)
		{
			weights[i].resize(control.numVertex());

			Msg::verify(mmd.totcagevert==control.numVertex(), "error1");
			Msg::verify(mmd.totvert==embedded.numVertex(), "error2");
			for(int j=0; j<control.numVertex(); j++)
				weights[i][j]=mmd.bindweights[i*mmd.totcagevert+j];
		}

	}	
	virtual void calculateCorresForTetra(OBJloader::Mesh & control, OBJloader::Mesh & embedded)
	{

	}
};

class HarmonicCoords :public LinearDeformer {
	MeshDeformModifierData mmd;
	matrix4 cagemat;
	vector3N vertexcos;
public:
	HarmonicCoords():LinearDeformer() {}
	~HarmonicCoords() {}

	virtual void calculateCorrespondence(OBJloader::Mesh & control, vector3N& embedded)
	{
		control_mesh=&control;
		embedded_vertices=&embedded;
		mmd.gridsize=7;
		mmd.object=&control;
		cagemat.identity();
		
		vertexcos.setSize(embedded.size());
		for(int i=0; i<vertexcos.size(); i++)
			vertexcos[i]=embedded(i);

		std::cout << "binding" << std::endl;
		harmonic_coordinates_bind(&mmd, vertexcos, vertexcos.size(), cagemat);
		std::cout << "finish" << std::endl;

		weights.resize(embedded.size());
		for(int i=0; i<embedded.size(); i++)
		{
			weights[i].resize(control.numVertex());

			Msg::verify(mmd.totcagevert==control.numVertex(), "error1");
			Msg::verify(mmd.totvert==embedded.size(), "error2");
			for(int j=0; j<control.numVertex(); j++)
				weights[i][j]=mmd.bindweights[i*mmd.totcagevert+j];
		}
	}
};

LinearDeformer* createHarmonicDeformer()
{
	return new HarmonicCoords();
}
MeshDeformer* createHarmonicMeshDeformer()
{
	return new HarmonicCoordMeshDeformer();
}
