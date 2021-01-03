/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
///btSoftBody2 implementation by Taesoo Kwon, based on the btSoftBody implementation by Nathanael Presson.
#include "softbody2/Physics.h"

#include "btSoftBody2.h"
#if	1
#include <stdio.h>
#define	DOTRACE
#endif
#include <string.h>
#include "LinearMath/btQuickprof.h"
#include "BulletCollision/NarrowPhaseCollision/btGjkEpa2.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseInterface.h"
#include "BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include <iostream>


//
// Collision shape
//

inline btVector3 ToBullet(vector3 const& v)
{
	return btVector3(v.x, v.y, v.z);
}

void btSoftBody2::addExternalForce(intvectorn const& indices, vector3N const& forces)
{
	_indices=&indices;
	_forces=&forces;
}
btScalar btSoftBody2::Node::im() const
{
	btScalar m=mpSystem->GetMass(m_index).x;
	if(m>0) return 1.0/m;
	return 0.0;
}
btVector3 btSoftBody2::Node::x() const
{
	return ToBullet(mpSystem->Position(m_index));
}
vector3& btSoftBody2::Node::x_ref()
{
	return mpSystem->Position(m_index);
}
btVector3 btSoftBody2::Node::v() const
{
	return ToBullet(mpSystem->Velocity(m_index));

}
vector3& btSoftBody2::Node::v_ref()
{
	return mpSystem->Velocity(m_index);
}

///btSoftBody2CollisionShape is work-in-progress collision shape for softbodies
class btSoftBody2CollisionShape : public btConcaveShape
{
public:
	btSoftBody2*						m_body;
	btSoftBody2::tNodeArray			m_nodes;		// Nodes
	btSoftBody2::tLinkArray			m_links;		// Links
	btSoftBody2::tFaceArray			m_faces;		// Faces

	btSoftBody2CollisionShape(btSoftBody2* backptr);

	virtual ~btSoftBody2CollisionShape();

	virtual void	processAllTriangles(btTriangleCallback* callback,const btVector3& aabbMin,const btVector3& aabbMax) const;

	///getAabb returns the axis aligned bounding box in the coordinate frame of the given transform t.
	virtual void getAabb(const btTransform& t,btVector3& aabbMin,btVector3& aabbMax) const
	{
	/* t should be identity, but better be safe than...fast? */ 
	const btVector3	mins=m_body->m_bounds[0];
	const btVector3	maxs=m_body->m_bounds[1];
	const btVector3	crns[]={t*btVector3(mins.x(),mins.y(),mins.z()),
							t*btVector3(maxs.x(),mins.y(),mins.z()),
							t*btVector3(maxs.x(),maxs.y(),mins.z()),
							t*btVector3(mins.x(),maxs.y(),mins.z()),
							t*btVector3(mins.x(),mins.y(),maxs.z()),
							t*btVector3(maxs.x(),mins.y(),maxs.z()),
							t*btVector3(maxs.x(),maxs.y(),maxs.z()),
							t*btVector3(mins.x(),maxs.y(),maxs.z())};
	aabbMin=aabbMax=crns[0];
	for(int i=1;i<8;++i)
		{
		aabbMin.setMin(crns[i]);
		aabbMax.setMax(crns[i]);
		}
	}

	virtual int		getShapeType() const
	{
		return SOFTBODY_SHAPE_PROXYTYPE;
	}
	virtual void	setLocalScaling(const btVector3& /*scaling*/)
	{		
		///na
		btAssert(0);
	}
	virtual const btVector3& getLocalScaling() const
	{
	static const btVector3 dummy(1,1,1);
	return dummy;
	}
	virtual void	calculateLocalInertia(btScalar /*mass*/,btVector3& /*inertia*/) const
	{
		///not yet
		btAssert(0);
	}
	virtual const char*	getName()const
	{
		return "SoftBody";
	}
};

btSoftBody2CollisionShape::btSoftBody2CollisionShape(btSoftBody2* backptr)
{
m_body=backptr;
}

btSoftBody2CollisionShape::~btSoftBody2CollisionShape()
{

}

void	btSoftBody2CollisionShape::processAllTriangles(btTriangleCallback* /*callback*/,const btVector3& /*aabbMin*/,const btVector3& /*aabbMax*/) const
{
	//not yet
	btAssert(0);
}

//
// Helpers
//

//
#ifdef DOTRACE
static inline void			Trace(const btMatrix3x3& m,const char* name)
{
	printf("%s[0]: %.2f,\t%.2f,\t%.2f\r\n",name,m[0].x(),m[0].y(),m[0].z());
	printf("%s[1]: %.2f,\t%.2f,\t%.2f\r\n",name,m[1].x(),m[1].y(),m[1].z());
	printf("%s[2]: %.2f,\t%.2f,\t%.2f\r\n",name,m[2].x(),m[2].y(),m[2].z());
	printf("\r\n");
}
#else
static inline void			Trace(const btMatrix3x3&,const char*) {}
#endif

//
template <typename T>
static inline T				Lerp(const T& a,const T& b,btScalar t)
{ return(a+(b-a)*t); }
//
static inline btMatrix3x3	Lerp(const btMatrix3x3& a, const btMatrix3x3& b, btScalar t)
{
btMatrix3x3	r;
r[0]=Lerp(a[0],b[0],t);
r[1]=Lerp(a[1],b[1],t);
r[2]=Lerp(a[2],b[2],t);
return(r);
}
//
template <typename T>
static inline T				Clamp(const T& x,const T& l,const T& h)
{ return(x<l?l:x>h?h:x); }
//
template <typename T>
static inline T				Sq(const T& x)
{ return(x*x); }
//
template <typename T>
static inline T				Cube(const T& x)
{ return(x*x*x); }
//
template <typename T>
static inline T				Sign(const T& x)
{ return((T)(x<0?-1:+1)); }
//
static inline btMatrix3x3	ScaleAlongAxis(const btVector3& a,btScalar s)
{
	const btScalar	xx=a.x()*a.x();
	const btScalar	yy=a.y()*a.y();
	const btScalar	zz=a.z()*a.z();
	const btScalar	xy=a.x()*a.y();
	const btScalar	yz=a.y()*a.z();
	const btScalar	zx=a.z()*a.x();
	btMatrix3x3		m;
	m[0]=btVector3(1-xx+xx*s,xy*s-xy,zx*s-zx);
	m[1]=btVector3(xy*s-xy,1-yy+yy*s,yz*s-yz);
	m[2]=btVector3(zx*s-zx,yz*s-yz,1-zz+zz*s);
	return(m);
}
//
static inline btMatrix3x3	Cross(const btVector3& v)
{
	btMatrix3x3	m;
	m[0]=btVector3(0,-v.z(),+v.y());
	m[1]=btVector3(+v.z(),0,-v.x());
	m[2]=btVector3(-v.y(),+v.x(),0);
	return(m);
}
//
static inline btMatrix3x3	Diagonal(btScalar x)
{
	btMatrix3x3	m;
	m[0]=btVector3(x,0,0);
	m[1]=btVector3(0,x,0);
	m[2]=btVector3(0,0,x);
	return(m);
}
//
static inline btMatrix3x3	Add(const btMatrix3x3& a, const btMatrix3x3& b)
{
	btMatrix3x3	r;
	for(int i=0;i<3;++i) r[i]=a[i]+b[i];
	return(r);
}
//
static inline btMatrix3x3	Sub(const btMatrix3x3& a, const btMatrix3x3& b)
{
	btMatrix3x3	r;
	for(int i=0;i<3;++i) r[i]=a[i]-b[i];
	return(r);
}
//
static inline btMatrix3x3	Mul(const btMatrix3x3& a, btScalar b)
{
	btMatrix3x3	r;
	for(int i=0;i<3;++i) r[i]=a[i]*b;
	return(r);
}
//
static inline btMatrix3x3	MassMatrix(btScalar im, const btMatrix3x3& iwi, const btVector3& r)
{
	const btMatrix3x3	cr=Cross(r);
	return(Sub(Diagonal(im),cr*iwi*cr));
}
//
static inline btMatrix3x3	ImpulseMatrix(btScalar dt, btScalar ima, btScalar imb, const btMatrix3x3& iwi, const btVector3& r)
{
	return(	Diagonal(1/dt)*
		Add(Diagonal(ima),MassMatrix(imb,iwi,r)).inverse());
}
//
static inline void			PolarDecompose(const btMatrix3x3& m, btMatrix3x3& q, btMatrix3x3& s)
{
	static const btScalar	half=(btScalar)0.5;
	static const btScalar	accuracy=(btScalar)0.00001;
	static const int		maxiterations=64;
	btScalar				det=m.determinant();
	if(!btFuzzyZero(det))
	{
		q=m;
		for(int i=0;i<maxiterations;++i)
		{
			q=Mul(Add(q,Mul(q.adjoint(),1/det).transpose()),half);
			const btScalar	ndet=q.determinant();
			if(Sq(ndet-det)>accuracy) det=ndet; else break;
		}
		/* Final orthogonalization	*/ 
		q[0]=q[0].normalized();
		q[1]=q[1].normalized();
		q[2]=cross(q[0],q[1]).normalized();
		/* Compute 'S'				*/ 
		s=q.transpose()*m;
	}
	else
	{
		q.setIdentity();
		s.setIdentity();
	}
}
//
static inline btVector3		ProjectOnAxis(const btVector3& v, const btVector3& a)
{
	return(a*dot(v,a));
}
//
static inline btVector3		ProjectOnPlane(const btVector3& v, const btVector3& a)
{
	return(v-ProjectOnAxis(v,a));
}
//
static inline void			ProjectOrigin(const btVector3& a, const btVector3& b, btVector3& prj, btScalar& sqd)
{
const btVector3	d=b-a;
const btScalar	m2=d.length2();
if(m2>SIMD_EPSILON)
	{	
	const btScalar	t=Clamp<btScalar>(-dot(a,d)/m2,0,1);
	const btVector3	p=a+d*t;
	const btScalar	l2=p.length2();
	if(l2<sqd)
		{
		prj=p;
		sqd=l2;
		}
	}
}
//
static inline void			ProjectOrigin(const btVector3& a, const btVector3& b, const btVector3& c, btVector3& prj, btScalar& sqd)
{
const btVector3&	q=cross(b-a,c-a);
const btScalar		m2=q.length2();
if(m2>SIMD_EPSILON)
	{
	const btVector3	n=q/btSqrt(m2);
	const btScalar	k=dot(a,n);
	const btScalar	k2=k*k;
	if(k2<sqd)
		{
		const btVector3	p=n*k;
		if(	(dot(cross(a-p,b-p),q)>0)&&
			(dot(cross(b-p,c-p),q)>0)&&
			(dot(cross(c-p,a-p),q)>0))
			{			
			prj=p;
			sqd=k2;
			}
			else
			{
			ProjectOrigin(a,b,prj,sqd);
			ProjectOrigin(b,c,prj,sqd);
			ProjectOrigin(c,a,prj,sqd);
			}
		}
	}
}
//
template <typename T>
static inline T				BaryEval(const T& a, const T& b, const T& c, const btVector3& coord)
{
	return(a*coord.x()+b*coord.y()+c*coord.z());
}
//
static inline btVector3		BaryCoord(const btVector3& a, const btVector3& b, const btVector3& c, const btVector3& p)
{
const btScalar	w[]={	cross(a-p,b-p).length(),
						cross(b-p,c-p).length(),
						cross(c-p,a-p).length()};
const btScalar	isum=1/(w[0]+w[1]+w[2]);
return(btVector3(w[1]*isum,w[2]*isum,w[0]*isum));
}

//
static inline btVector3		NormalizeAny(const btVector3& v)
{
	const btScalar l=v.length();
	if(l>SIMD_EPSILON)
		return(v/l);
	else
		return(btVector3(0,0,0));
}

//
static inline btDbvt::Aabb	BoxOf(const btSoftBody2::Face& f, btScalar margin)
{
	btVector3 p1=f.m_n[0]->x();
	btVector3 p2=f.m_n[1]->x();
	btVector3 p3=f.m_n[2]->x();

const btVector3*	pts[]={	&p1,
							&p2,
							&p3};
btDbvt::Aabb		aabb=btDbvt::Aabb::FromPoints(pts,3);
aabb.Expand(btVector3(margin,margin,margin));
return(aabb);
}

//
static inline btScalar		AreaOf(const btVector3& x0, const btVector3& x1, const btVector3& x2)
{
	const btVector3	a=x1-x0;
	const btVector3	b=x2-x0;
	const btVector3	cr=cross(a,b);
	const btScalar	area=cr.length();
	return(area);
}

//
static inline btScalar		VolumeOf(const btVector3& x0, const btVector3& x1, const btVector3& x2, const btVector3& x3)
{
	const btVector3	a=x1-x0;
	const btVector3	b=x2-x0;
	const btVector3	c=x3-x0;
	return(dot(a,cross(b,c)));
}

//
static inline btScalar		RayTriangle(const btVector3& org, const btVector3& dir, const btVector3& a, const btVector3& b, const btVector3& c, btScalar maxt=SIMD_INFINITY)
{
	static const btScalar	ceps=-SIMD_EPSILON*10;
	static const btScalar	teps=SIMD_EPSILON*10;
	const btVector3			n=cross(b-a,c-a);
	const btScalar			d=dot(a,n);
	const btScalar			den=dot(dir,n);
	if(!btFuzzyZero(den))
	{
		const btScalar		num=dot(org,n)-d;
		const btScalar		t=-num/den;
		if((t>teps)&&(t<maxt))
		{
			const btVector3	hit=org+dir*t;
			if(	(dot(n,cross(a-hit,b-hit))>ceps)	&&			
				(dot(n,cross(b-hit,c-hit))>ceps)	&&
				(dot(n,cross(c-hit,a-hit))>ceps))
			{
				return(t);
			}
		}
	}
	return(-1);
}

//
// Private implementation
//

struct	RayCaster : public btDbvt::ICollide
	{
	btVector3			o;
	btVector3			d;
	btVector3			nd;
	btScalar			mint;
	btSoftBody2::Face*	face;
	int					tests;
			RayCaster(const btVector3& org,const btVector3& dir,btScalar mxt)
		{
		o		=	org;
		d		=	dir;
		nd		=	dir.normalized();	
		mint	=	mxt;
		face	=	0;
		tests	=	0;
		}
	void	Process(const btDbvt::Node* leaf)
		{
		btSoftBody2::Face&	f=*(btSoftBody2::Face*)leaf->data;
		const btScalar		t=RayTriangle(	o,d,
											f.m_n[0]->x(),
											f.m_n[1]->x(),
											f.m_n[2]->x(),
											mint);
		if((t>0)&&(t<mint))
			{
			mint=t;
			face=&f;
			}
		++tests;
		}
	bool	Descent(const btDbvt::Node* node)
		{
		const btVector3	ctr=node->box.Center()-o;		
		const btScalar	sqr=node->box.Lengths().length2()/4;
		const btScalar	prj=dot(ctr,nd);
		return((ctr-(nd*prj)).length2()<=sqr);
		}
	};


//
static int		RaycastInternal(const btSoftBody2* psb, const btVector3& org, const btVector3& dir, btScalar& mint,	int& face, bool bcountonly)
{
	int	cnt=0;
	if(bcountonly||psb->m_fdbvt.empty())
		{/* Full search	*/ 
		for(int i=0,ni=psb->getFaces().size();i<ni;++i)
			{
			const btSoftBody2::Face&	f=psb->getFaces()[i];
			const btScalar			t=RayTriangle(	org,dir,
													f.m_n[0]->x(),
													f.m_n[1]->x(),
													f.m_n[2]->x(),
													mint);
			if(t>0)
				{
				++cnt;if(!bcountonly) { face=i;mint=t; }
				}
			}
		}
		else
		{/* Use dbvt	*/ 
		RayCaster	collider(org,dir,mint);
		psb->m_fdbvt.collideGeneric(collider);
		if(collider.face)
			{
			mint=collider.mint;
			face=(int)(collider.face-&psb->getFaces()[0]);
			cnt=1;
			}
		}
	return(cnt);
}

//
static void		InitializeFaceTree(btSoftBody2* psb)
{
psb->m_fdbvt.clear();
for(int i=0;i<psb->getFaces().size();++i)
	{
	btSoftBody2::Face&	f=psb->getFaces()[i];
	f.m_leaf=psb->m_fdbvt.insert(BoxOf(f,0),&f);
	}
psb->m_fdbvt.optimizeTopDown();
}

//
static btVector3	EvaluateCom(btSoftBody2* psb)
{
	btVector3	com(0,0,0);
	return(com);
}

extern bool			CheckContactTaesoo(btSoftBody2* psb, btRigidBody* prb, btSoftBody2::Node& n, btScalar margin, btSoftBody2::sCti& cti);

//
static bool			CheckContact(btSoftBody2* psb, btRigidBody* prb, btSoftBody2::Node& n, btScalar margin, btSoftBody2::sCti& cti)
{
	return CheckContactTaesoo(psb, prb, n, margin, cti);

	/*
	btVector3			nrm;
	btCollisionShape*	shp=prb->getCollisionShape();
	if(shp->isConvex())
	{
		btConvexShape*		csh=static_cast<btConvexShape*>(shp);
		const btTransform&	wtr=prb->getInterpolationWorldTransform();
		btScalar			dst=psb->m_worldInfo->m_sparsesdf.Evaluate(	wtr.invXform(x),
																		csh,
																		nrm,
																		margin);
		if(dst<0)
		{
			cti.m_body		=	prb;
			cti.m_normal	=	wtr.getBasis()*nrm;
			cti.m_offset	=	-dot(	cti.m_normal,
										x-cti.m_normal*dst);
			return(true);
		}
		return(false);
	}
	else
		//........ 아직 안구현됨.
		return false;
		*/
}

//
static void			UpdateNormals(btSoftBody2* psb)
{
	const btVector3	zv(0,0,0);
	for(int i=0,ni=psb->getNodes().size();i<ni;++i)
	{
		psb->getNodes()[i].m_n=zv;
	}
	for(int i=0,ni=psb->getFaces().size();i<ni;++i)
	{
		btSoftBody2::Face&	f=psb->getFaces()[i];
		const btVector3		n=cross(f.m_n[1]->x()-f.m_n[0]->x(),
			f.m_n[2]->x()-f.m_n[0]->x());
		f.m_normal=n.normalized();
		f.m_n[0]->m_n+=n;
		f.m_n[1]->m_n+=n;
		f.m_n[2]->m_n+=n;
	}
	for(int i=0,ni=psb->getNodes().size();i<ni;++i)
	{
		psb->getNodes()[i].m_n.normalize();
	}
}

//
static void			UpdateBounds(btSoftBody2* psb)
{
	if(psb->m_ndbvt.m_root)
		{
		const btVector3&	mins=psb->m_ndbvt.m_root->box.Mins();
		const btVector3&	maxs=psb->m_ndbvt.m_root->box.Maxs();
		const btScalar		csm=psb->getCollisionShape()->getMargin();
		const btVector3		mrg=btVector3(	csm,
											csm,
											csm)*1; // ??? to investigate...
		psb->m_bounds[0]=mins-mrg;
		psb->m_bounds[1]=maxs+mrg;
			if(0!=psb->getBroadphaseHandle())
			{					
				psb->m_worldInfo->m_broadphase->setAabb(psb->getBroadphaseHandle(),
														psb->m_bounds[0],
														psb->m_bounds[1],
														psb->m_worldInfo->m_dispatcher);
			}
		}
		else
		{
		psb->m_bounds[0]=
		psb->m_bounds[1]=btVector3(0,0,0);
		}		
}


//static void			UpdateConstantsTaesoo(btSoftBody2* psb);
//
static void			UpdateConstants(btSoftBody2* psb)
{
	//UpdateConstantsTaesoo(psb);
	//return;
	/* Links		*/ 
	for(int i=0,ni=psb->getLinks().size();i<ni;++i)
	{
		btSoftBody2::Link&	l=psb->getLinks()[i];
		l.m_pSpring->m_RestDistance=	(l.m_n[0]->x()-l.m_n[1]->x()).length();
		l.m_c0	=	l.m_n[0]->im()+l.m_n[1]->im();
		l.m_c1	=	l.m_pSpring->m_RestDistance*l.m_pSpring->m_RestDistance;
	}
	/* Faces		*/ 
	for(int i=0,ni=psb->getFaces().size();i<ni;++i)
	{
		btSoftBody2::Face&	f=psb->getFaces()[i];
		f.m_ra	=	AreaOf(f.m_n[0]->x(),f.m_n[1]->x(),f.m_n[2]->x());
	}
	/* Area's		*/ 
	btAlignedObjectArray<int>	counts;
	counts.resize(psb->getNodes().size(),0);
	for(int i=0,ni=psb->getNodes().size();i<ni;++i)
	{
		psb->getNodes()[i].m_area	=	0;
	}
	for(int i=0,ni=psb->getFaces().size();i<ni;++i)
	{
		btSoftBody2::Face&	f=psb->getFaces()[i];
		for(int j=0;j<3;++j)
		{
			const int index=(int)(f.m_n[j]-&psb->getNodes()[0]);
			counts[index]++;
			f.m_n[j]->m_area+=btFabs(f.m_ra);
		}
	}
	for(int i=0,ni=psb->getNodes().size();i<ni;++i)
	{
		if(counts[i]>0)
			psb->getNodes()[i].m_area/=(btScalar)counts[i];
		else
			psb->getNodes()[i].m_area=0;
	}
}

inline vector3 ToBase(btVector3 const& a)
{
	return vector3(a.x(), a.y(), a.z());
}
//
static void			PSolve_Anchors(btSoftBody2* psb,btScalar sdt)
{
	const btScalar	kAHR=psb->m_cfg.kAHR;
	for(int i=0,ni=psb->m_anchors.size();i<ni;++i)
	{
	ASSERT(0);

		const btSoftBody2::Anchor&	a=psb->m_anchors[i];
		const btTransform&			t=a.m_body->getInterpolationWorldTransform();
		btSoftBody2::Node&			n=*a.m_node;
		const btVector3				wa=t*a.m_local;
		const btVector3				va=a.m_body->getVelocityInLocalPoint(a.m_c1)*sdt;
		const btVector3				vb=n.v()*sdt;
		const btVector3				vr=(va-vb)+(wa-n.x())*kAHR;
		const btVector3				impulse=a.m_c0*vr;
		n.x_ref()+=ToBase(impulse*a.m_c2);
		a.m_body->applyImpulse(-impulse,a.m_c1);
	}
}




// taesoo modified - handle soft - rigid collision.
static void			PSolve_RContacts(btSoftBody2* psb,btScalar sdt)
{
	if(psb->m_pSystem->m_cfg.contactMethod!=Physics_ParticleSystem::Config::BARAFF98)
	{
		for(int i=0,ni=psb->m_rcontacts.size();i<ni;++i)
		{
			const btSoftBody2::RContact&	c=psb->m_rcontacts[i];
			const btSoftBody2::sCti&		cti=c.m_cti;	
			const btVector3		va=cti.m_body->getVelocityInLocalPoint(c.m_c1);
			const btVector3		vb=c.m_node->v();
			psb->m_pSystem->mContacts_peneltyMethod.add(ToBase(va), ToBase(vb), ToBase(cti.m_normal), cti.m_dp2, c.m_node->m_index);
		}
	}
	else
	{
		static std::vector<bool> contacts;
		contacts.resize(psb->m_pSystem->numParticle());

		for(int i=0, ni=psb->m_pSystem->numParticle(); i<ni; i++)
			contacts[i]=false;

		for(int i=0,ni=psb->m_rcontacts.size();i<ni;++i)
		{
			const btSoftBody2::RContact&	c=psb->m_rcontacts[i];
			const btSoftBody2::sCti&		cti=c.m_cti;	
			const btVector3		va=cti.m_body->getVelocityInLocalPoint(c.m_c1);
			const btVector3		vb=c.m_node->v();
			const btVector3		vr=vb-va;

			// 수직방향 속도.
			btScalar		dn=dot(vr,cti.m_normal);		
			if(dn<=SIMD_EPSILON)
			{
				contacts[c.m_node->m_index]=true;

				// penetrating depth
				const btScalar		dp=cti.m_dp;

				// 수평방향 속도.
				const btVector3		fv=vr-cti.m_normal*dn;

				// 즉 vr= fv+ cti.m_normal*dn;

				// 꺼내기.
				//const btVector3		new_vr=fv*c.m_c3-cti.m_normal*(dp*c.m_c4);			
				//const btVector3		new_vr=fv*c.m_c3-cti.m_normal*(dn*c.m_c4);
				//const btVector3		new_vr=fv*c.m_c3;
				const btVector3		new_vr=fv;
				// todo: 바꿔야 할 듯.

				////////////////////////////////////////
				// update particle 

				// apply impulses
				btVector3 desiredDeltaVelocity=new_vr-vr;
				//btVector3 correction=-1*impulse*c.m_c2;
				btVector3 correction=-1.01*cti.m_normal*dp;

				psb->m_pSystem->mContacts.setContacts(c.m_node->m_index, 
					//desiredDeltaVelocity.x(), desiredDeltaVelocity.y(), desiredDeltaVelocity.z(),	
					vr.x(), vr.y(), vr.z(), 
					correction.x(), correction.y(), correction.z(), 
					cti.m_normal.x(), cti.m_normal.y(), cti.m_normal.z(), NULL);

				// setImpulse는 아래 내용을 대신한다.
				// c.m_node->m_v+=desiredDeltaVelocity;
				// c.m_node->m_x+=correction; //-> position correction requires modification of the linear system.			

				////////////////////////////////////////
				// update rigidbody
				const btVector3		impulse=c.m_c0*(vr-new_vr);
				c.m_cti.m_body->applyImpulse(impulse,c.m_c1);


			}
		}


		for(int i=0, ni=psb->m_pSystem->numParticle(); i<ni; i++)
		{
			if(!contacts[i])
			{
				psb->m_pSystem->mContacts.setNoContact(i);
			}
		}

		// 이제 모든 노드에 대해서 이전 프레임에 constraint이 있었지만, 이번 프레임에 contact로 detect되지 않은 것을 찾아낸다.
		// 
		/*

		Physics_ParticleSystem  & system=*psb->m_pSystem;
		for(int i=0, ni=system.numParticle(); i<ni; i++)
		{
		if(system.mContacts.prevContactStatus[i]!=Physics_Contacts::no_contact)
		{
		if(system.mContacts.contactStatus[i]!=Physics_Contacts::no_contact)
		printf("o");
		else
		printf("x");
		}
		}
		printf("\n");


		*/
	}

}

//
static void			PSolve_SContacts(btSoftBody2* psb, btScalar sdt)
{
for(int i=0,ni=psb->m_scontacts.size();i<ni;++i)
	{
	const btSoftBody2::SContact&	c=psb->m_scontacts[i];
	const btVector3&	nr=c.m_normal;
	btSoftBody2::Node&	n=*c.m_node;
	btSoftBody2::Face&	f=*c.m_face;
	const btVector3		p=BaryEval(	f.m_n[0]->x(),
									f.m_n[1]->x(),
									f.m_n[2]->x(),
									c.m_weights);


	const btVector3		vv=BaryEval(	f.m_n[0]->v(),
									f.m_n[1]->v(),
									f.m_n[2]->v(),
									c.m_weights);
	const btVector3		q=p-vv*sdt;


	const btVector3		vr=n.v()*sdt-(p-q);
	btVector3			corr(0,0,0);
	if(dot(vr,nr)<0)
		{
		const btScalar	j=c.m_margin-(dot(nr,n.x())-dot(nr,p));
		corr+=c.m_normal*j;
		}
	corr			-=	ProjectOnPlane(vr,nr)*c.m_friction;
	/*n.m_x			+=	corr*c.m_cfm[0];
	f.m_n[0]->m_x	-=	corr*(c.m_cfm[1]*c.m_weights.x());
	f.m_n[1]->m_x	-=	corr*(c.m_cfm[1]*c.m_weights.y());
	f.m_n[2]->m_x	-=	corr*(c.m_cfm[1]*c.m_weights.z());
*/
	assert(0);
	// velocity를 다시 계산해주어야함.
	}
}


//
// btSoftBody2
//

//
btSoftBody2::btSoftBody2(btSoftBody2::btSoftBody2WorldInfo*	worldInfo,int node_count,  const btVector3* x,  const btScalar* m, double kSpringDamp, int integrateMethod)
:m_worldInfo(worldInfo)
{	
	_indices=NULL;
	_forces=NULL;
	m_pSystem=NULL;
	
	/* Init		*/ 
	m_internalType		=	CO_SOFT_BODY2;
	m_cfg.kSpringDamp	=	kSpringDamp;
	m_cfg.kSHR		=	(btScalar)1.0;
	m_cfg.kAHR		=	(btScalar)0.7;
	m_cfg.timescale		=	1;
	m_cfg.collisions	=	fCollision::Default;
	m_tag			=	0;
	m_timeacc		=	0;
	m_bUpdateRtCst		=	true;
	m_bounds[0]		=	btVector3(0,0,0);
	m_bounds[1]		=	btVector3(0,0,0);
	m_worldTransform.setIdentity();
	///for now, create a collision shape internally
	setCollisionShape(new btSoftBody2CollisionShape(this));	
	m_collisionShape->setMargin(0.25);

	m_pSystem=new Physics_ParticleSystem(node_count);

	Physics_ParticleSystem* system=m_pSystem;
	system->m_iInverseIterations = 10;
	system->m_iIntegrationMethod = integrateMethod;

	system->Reset();

	/* Nodes	*/ 
	const btScalar		margin=getCollisionShape()->getMargin();
	getNodes().resize(node_count);
	for(int i=0,ni=node_count;i<ni;++i)
	{	
		Node&	n=getNodes()[i];
		memset(&n,0,sizeof(n));
		// taesoo
		n.m_index=i;
		n.mpSystem=system;

		n.x_ref()=ToBase(x?*x++:btVector3(0,0,0));
		n.v_ref()=ToBase(btVector3(0,0,0));

		system->SetMass(i, m?*m++:1);
		n.m_leaf	=	m_ndbvt.insert(btDbvt::Aabb::FromCR(n.x(),margin),&n);
	}

	m_ndbvt.optimizeTopDown();
	UpdateBounds(this);	

	//printf("create btSoftbody2");
}

//
btSoftBody2::~btSoftBody2()
{
	//printf("discard btSoftbody2");
	//for now, delete the internal shape
	delete m_collisionShape;	
		delete m_pSystem;

}

//
bool			btSoftBody2::checkLink(int node0,int node1) const
{
	return(checkLink(&getNodes()[node0],&getNodes()[node1]));
}

//
bool			btSoftBody2::checkLink(const Node* node0,const Node* node1) const
{
	const Node*	n[]={node0,node1};
	for(int i=0,ni=getLinks().size();i<ni;++i)
	{
		const Link&	l=getLinks()[i];
		if(	(l.m_n[0]==n[0]&&l.m_n[1]==n[1])||
			(l.m_n[0]==n[1]&&l.m_n[1]==n[0]))
		{
			return(true);
		}
	}
	return(false);
}

//
bool			btSoftBody2::checkFace(int node0,int node1,int node2) const
{
	const Node*	n[]={	&getNodes()[node0],
		&getNodes()[node1],
		&getNodes()[node2]};
	for(int i=0,ni=getFaces().size();i<ni;++i)
	{
		const Face&	f=getFaces()[i];
		int			c=0;
		for(int j=0;j<3;++j)
		{
			if(	(f.m_n[j]==n[0])||
				(f.m_n[j]==n[1])||
				(f.m_n[j]==n[2])) c|=1<<j; else break;
		}
		if(c==7) return(true);
	}
	return(false);
}

//
void			btSoftBody2::appendLink(int node0, int node1, btScalar kST, eLType::_ type, bool bcheckexist)
{
	appendLink(&getNodes()[node0],&getNodes()[node1],kST,type,bcheckexist);
}

//
void			btSoftBody2::appendLink(Node* node0, Node* node1, btScalar kST, eLType::_ type, bool bcheckexist)
{
	if((!bcheckexist)||(!checkLink(node0,node1)))
	{
		Physics_SpringForce* pSpring = new Physics_SpringForce();
		pSpring->m_iParticle[0] = node0->m_index;
		pSpring->m_iParticle[1] = node1->m_index;
		pSpring->m_RestDistance = (node0->x()-node1->x()).length();
		pSpring->m_MaxDistance = pSpring->m_RestDistance* 1.1;
		pSpring->m_bFixup = true;
		pSpring->m_kSpring = kST;
		pSpring->m_kSpringDamp = m_cfg.kSpringDamp;
		m_pSystem->AddForce( *pSpring );

		Link	l;
		l.m_pSpring=pSpring;
		l.m_n[0]	=	node0;
		l.m_n[1]	=	node1;
		l.m_c0		=	0;
		l.m_c1		=	0;
		l.m_type	=	type;
		l.m_tag		=	0;
		getLinks().push_back(l);


		m_bUpdateRtCst=true;
	}
}

//
void			btSoftBody2::appendFace(int node0,int node1,int node2)
{
	Face	f;
	f.m_n[0]	=	&getNodes()[node0];
	f.m_n[1]	=	&getNodes()[node1];
	f.m_n[2]	=	&getNodes()[node2];
	f.m_ra		=	AreaOf(	f.m_n[0]->x(),
		f.m_n[1]->x(),
		f.m_n[2]->x());
	f.m_tag		=	0;
	getFaces().push_back(f);
	m_bUpdateRtCst=true;
}

//
void			btSoftBody2::appendAnchor(int node,btRigidBody* body)
{
	Anchor	a;
	a.m_node			=	&getNodes()[node];
	a.m_body			=	body;
	a.m_local			=	body->getInterpolationWorldTransform().inverse()*a.m_node->x();
	a.m_node->m_battach	=	1;
	m_anchors.push_back(a);
}


//
void			btSoftBody2::setMass(int node,btScalar mass)
{
	m_pSystem->SetMass(node, mass);
	m_bUpdateRtCst=true;
}

//
btScalar		btSoftBody2::getMass(int node) const
{
	return m_pSystem->GetMass(node).x;
}

//
btScalar		btSoftBody2::getTotalMass() const
{
	btScalar	mass=0;
	for(int i=0;i<getNodes().size();++i)
	{
		mass+=getMass(i);
	}
	return(mass);
}

//
void			btSoftBody2::setTotalMass(btScalar mass,bool fromfaces)
{
	if(fromfaces)
	{
		vectorn masses(getNodes().size());
		masses.setAllValue(0.0);
		
		for(int i=0;i<getFaces().size();++i)
		{
			const Face&		f=getFaces()[i];
			const btScalar	twicearea=AreaOf(	f.m_n[0]->x(),
				f.m_n[1]->x(),
				f.m_n[2]->x());
			for(int j=0;j<3;++j)
			{
				masses(f.m_n[j]->m_index)+=twicearea;
			}
		}

		masses*=mass/masses.sum();

		for(int i=0;i<getNodes().size();++i)
		{
			setMass(i, masses[i]);
		}
	}
	else
	{
		btScalar itm=mass/getNodes().size();
		for(int i=0;i<getNodes().size();++i)
			setMass(i, itm);
	}

	m_bUpdateRtCst=true;
}

//
void			btSoftBody2::setTotalDensity(btScalar density)
{
	setTotalMass(getVolume()*density,true);
}






//
btScalar		btSoftBody2::getVolume() const
{
	btScalar	vol=0;
	if(getNodes().size()>0)
	{
		const btVector3	org=getNodes()[0].x();
		for(int i=0,ni=getFaces().size();i<ni;++i)
		{
			const Face&	f=getFaces()[i];
			vol+=dot(f.m_n[0]->x()-org,cross(f.m_n[1]->x()-org,f.m_n[2]->x()-org));
		}
		vol/=(btScalar)6;
	}
	return(vol);
}

//
int				btSoftBody2::generateBendingConstraints(int distance, btScalar stiffness)
{
	if(distance>1)
	{
		/* Build graph	*/ 
		const int		n=getNodes().size();
		const unsigned	inf=(~(unsigned)0)>>1;
		unsigned*		adj=new unsigned[n*n];
#define IDX(_x_,_y_)	((_y_)*n+(_x_))
		for(int j=0;j<n;++j)
		{
			for(int i=0;i<n;++i)
			{
				if(i!=j)	adj[IDX(i,j)]=adj[IDX(j,i)]=inf;
				else
					adj[IDX(i,j)]=adj[IDX(j,i)]=0;
			}
		}
		for(int i=0;i<getLinks().size();++i)
		{
			if(getLinks()[i].m_type==eLType::Structural)
			{
				const int	ia=(int)(getLinks()[i].m_n[0]-&getNodes()[0]);
				const int	ib=(int)(getLinks()[i].m_n[1]-&getNodes()[0]);
				adj[IDX(ia,ib)]=1;
				adj[IDX(ib,ia)]=1;
			}
		}
		for(int k=0;k<n;++k)
		{
			for(int j=0;j<n;++j)
			{
				for(int i=j+1;i<n;++i)
				{
					const unsigned	sum=adj[IDX(i,k)]+adj[IDX(k,j)];
					if(adj[IDX(i,j)]>sum)
					{
						adj[IDX(i,j)]=adj[IDX(j,i)]=sum;
					}
				}
			}
		}
		/* Build links	*/ 
		int	nlinks=0;
		for(int j=0;j<n;++j)
		{
			for(int i=j+1;i<n;++i)
			{
				if(adj[IDX(i,j)]==(unsigned)distance)
				{
					appendLink(i,j,stiffness,eLType::Bending);
					++nlinks;
				}
			}
		}
		delete[] adj;
		return(nlinks);
	}
	return(0);
}

//
void			btSoftBody2::randomizeConstraints()
{
	for(int i=0,ni=getLinks().size();i<ni;++i)
	{
		btSwap(getLinks()[i],getLinks()[rand()%ni]);
	}
	for(int i=0,ni=getFaces().size();i<ni;++i)
	{
		btSwap(getFaces()[i],getFaces()[rand()%ni]);
	}
}

//
bool			btSoftBody2::rayCast(const btVector3& org, const btVector3& dir, sRayCast& results,	btScalar maxtime)
{
	if(getFaces().size())
		{
		if(m_fdbvt.empty()) InitializeFaceTree(this);
		results.time	=	maxtime;
		results.face	=	-1;
		return(RaycastInternal(	this,org,dir,
								results.time,
								results.face,false)!=0);
		}

	return(false);
}


// taesoo
void			btSoftBody2::predictMotion(btScalar dt)
{
	/* Update				*/ 
	if(m_bUpdateRtCst)
	{
		m_bUpdateRtCst=false;
		UpdateConstants(this);
		m_fdbvt.clear();
		if(m_cfg.collisions&fCollision::VF_SS)
		{
			InitializeFaceTree(this);
		}
	}

	

	/* Prepare				*/ 
	m_sst.sdt		=	dt*m_cfg.timescale;
	m_sst.isdt		=	1/m_sst.sdt;
	m_sst.velmrg	=	m_sst.sdt*3;
	m_sst.radmrg	=	getCollisionShape()->getMargin();
	m_sst.updmrg	=	m_sst.radmrg*(btScalar)0.25;

	
	assert( m_pSystem);

	m_pSystem->m_TotalForces_ext.zero();

	if(_indices)
	{
		printf("external forces\n");
		for(int i=0; i<_indices->size(); i++)
			m_pSystem->m_TotalForces_ext[(*_indices)[i]]=(*_forces)[i];
	}
	_indices=NULL;
	_forces=NULL;
	m_pSystem->Update( m_sst.sdt);
	
	for(int i=0,ni=getNodes().size();i<ni;++i)
	{
		Node&	n=getNodes()[i];

		m_ndbvt.update(	n.m_leaf,
						btDbvt::Aabb::FromCR(n.x(),m_sst.radmrg),
						n.v()*m_sst.velmrg,
						m_sst.updmrg);
	}

	UpdateBounds(this);	

	/* Faces				*/ 
	if(!m_fdbvt.empty())
		{
		for(int i=0;i<getFaces().size();++i)
			{
			Face&	f=getFaces()[i];			
			const btVector3	v=(	f.m_n[0]->v()+
								f.m_n[1]->v()+
								f.m_n[2]->v())/3;
			m_fdbvt.update(	f.m_leaf,
							BoxOf(f,m_sst.radmrg),
							v*m_sst.velmrg,
							m_sst.updmrg);
			}
		}
	/* Clear contacts	*/ 
	m_rcontacts.resize(0);
	m_scontacts.resize(0);
}

//
void			btSoftBody2::solveConstraints()
{
	
	/* Prepare anchors		*/ 
	for(int i=0,ni=m_anchors.size();i<ni;++i)
	{
		Anchor&			a=m_anchors[i];
		const btVector3	ra=a.m_body->getWorldTransform().getBasis()*a.m_local;
		a.m_c0	=	ImpulseMatrix(	m_sst.sdt,
									a.m_node->im(),
									a.m_body->getInvMass(),
									a.m_body->getInvInertiaTensorWorld(),
									ra);
		a.m_c1	=	ra;
		a.m_c2	=	m_sst.sdt*a.m_node->im();
		a.m_body->activate();
	}
	/* Solve				*/ 
	PSolve_Anchors(this,m_sst.sdt);
	PSolve_RContacts(this,m_sst.sdt);
	PSolve_SContacts(this, m_sst.sdt);
}

//
void			btSoftBody2::solveCommonConstraints(btSoftBody2** /*bodies*/,int /*count*/,int /*iterations*/)
{
/// placeholder
}

//
void			btSoftBody2::integrateMotion()
{
	/* Velocities		*/ 
	for(int i=0,ni=getNodes().size();i<ni;++i)
	{
		Node&	n=getNodes()[i];
		//n.m_v	*=(1-m_cfg.kDP);
	}
	/* Update			*/ 
	UpdateNormals(this);
}

//
void			btSoftBody2::defaultCollisionHandler(btCollisionObject* pco)
{
switch(m_cfg.collisions&fCollision::RVSmask)
	{
	case	fCollision::SDF_RS:
		{
		struct	DoCollide : btDbvt::ICollide
			{
			void		Process(const btDbvt::Node* leaf)
				{
				Node*	node=(Node*)leaf->data;
				DoNode(*node);
				}
			void		DoNode(Node& n) const
				{				
				const btScalar	m=n.im()>0?dynmargin:stamargin;
				RContact		c;
				if(	(!n.m_battach)&&
					CheckContact(psb,prb,n,m,c.m_cti))
					{
					const btScalar	ima=n.im();
					const btScalar	imb=prb->getInvMass();
					const btScalar	ms=ima+imb;
					if(ms>0)
						{
						const btTransform&	wtr=prb->getInterpolationWorldTransform();
						const btMatrix3x3	iwi=prb->getInvInertiaTensorWorld();
						const btVector3		ra=n.x()-wtr.getOrigin();
						const btVector3		va=prb->getVelocityInLocalPoint(ra)*psb->m_sst.sdt;
						const btVector3		vb=n.v()*psb->m_sst.sdt;
						const btVector3		vr=vb-va;
						const btScalar		dn=dot(vr,c.m_cti.m_normal);
						const btVector3		fv=vr-c.m_cti.m_normal*dn;
						c.m_node	=	&n;
						c.m_c0		=	ImpulseMatrix(psb->m_sst.sdt,ima,imb,iwi,ra);
						c.m_c1		=	ra;
						c.m_c2		=	ima*psb->m_sst.sdt;
						psb->m_rcontacts.push_back(c);
						prb->activate();
						}
					}
				}
			btSoftBody2*		psb;
			btRigidBody*	prb;
			btScalar		dynmargin;
			btScalar		stamargin;
			}				docollide;
		btRigidBody*		prb=btRigidBody::upcast(pco);
		const btTransform	wtr=prb->getInterpolationWorldTransform();
		const btTransform	ctr=prb->getWorldTransform();
		const btScalar		timemargin=(wtr.getOrigin()-ctr.getOrigin()).length();
		const btScalar		basemargin=getCollisionShape()->getMargin();
		btVector3			mins;
		btVector3			maxs;
		btDbvt::Aabb		aabb;
		pco->getCollisionShape()->getAabb(	pco->getInterpolationWorldTransform(),
											mins,
											maxs);
		aabb=btDbvt::Aabb::FromMM(mins,maxs);
		aabb.Expand(btVector3(basemargin,basemargin,basemargin));		
		docollide.psb		=	this;
		docollide.prb		=	prb;
		docollide.dynmargin	=	basemargin+timemargin;
		docollide.stamargin	=	basemargin;
		m_ndbvt.collide(aabb,&docollide);
		}
	break;
	}
}

//
void			btSoftBody2::defaultCollisionHandler(btSoftBody2* psb)
{
const int cf=m_cfg.collisions&psb->m_cfg.collisions;
switch(cf&fCollision::SVSmask)
	{
	case	fCollision::VF_SS:
		{
		struct	DoCollide : btDbvt::ICollide
			{
			void		Process(const btDbvt::Node* lnode,
								const btDbvt::Node* lface)
				{
				Node*	node=(Node*)lnode->data;
				Face*	face=(Face*)lface->data;
				btVector3	o=node->x();
				btVector3	p;
				btScalar	d=SIMD_INFINITY;
				ProjectOrigin(	face->m_n[0]->x()-o,
								face->m_n[1]->x()-o,
								face->m_n[2]->x()-o,
								p,d);
				//const btScalar	m=mrg+(o-node->m_q).length()*2;
				const btScalar	m=mrg+(node->v()*psb[0]->m_sst.sdt).length()*2;
				if(d<(m*m))
					{
					const Node*		n[]={face->m_n[0],face->m_n[1],face->m_n[2]};
					const btVector3	w=BaryCoord(n[0]->x(),n[1]->x(),n[2]->x(),p+o);
					const btScalar	ma=node->im();
					btScalar		mb=BaryEval(n[0]->im(),n[1]->im(),n[2]->im(),w);
					if(	(n[0]->im()<=0)||
						(n[1]->im()<=0)||
						(n[2]->im()<=0))
						{
						mb=0;
						}
					const btScalar	ms=ma+mb;
					if(ms>0)
						{
						SContact		c;
						c.m_normal		=	p/-btSqrt(d);
						c.m_margin		=	m;
						c.m_node		=	node;
						c.m_face		=	face;
						c.m_weights		=	w;
						c.m_friction	=	0.2;
						c.m_cfm[0]		=	ma/ms*psb[0]->m_cfg.kSHR;
						c.m_cfm[1]		=	mb/ms*psb[1]->m_cfg.kSHR;
						psb[0]->m_scontacts.push_back(c);
						}
					}	
				}
			btSoftBody2*		psb[2];
			btScalar		mrg;
			}	docollide;
		/* common					*/ 
		docollide.mrg=	getCollisionShape()->getMargin()+
						psb->getCollisionShape()->getMargin();
		/* psb0 nodes vs psb1 faces	*/ 
		docollide.psb[0]=this;
		docollide.psb[1]=psb;
		docollide.psb[0]->m_ndbvt.collide(	&docollide.psb[1]->m_fdbvt,
											&docollide);
		/* psb1 nodes vs psb0 faces	*/ 
		docollide.psb[0]=psb;
		docollide.psb[1]=this;
		docollide.psb[0]->m_ndbvt.collide(	&docollide.psb[1]->m_fdbvt,
											&docollide);
		}
	break;
	}
}

//
// Accessor's
//

//
btSoftBody2::tNodeArray&				btSoftBody2::getNodes()
	{
		return ((btSoftBody2CollisionShape*)m_collisionShape)->m_nodes;
	}
	
//
const btSoftBody2::tNodeArray&		btSoftBody2::getNodes() const
	{
		return ((btSoftBody2CollisionShape*)m_collisionShape)->m_nodes;
	}

//
btSoftBody2::tLinkArray&				btSoftBody2::getLinks()
	{
		return ((btSoftBody2CollisionShape*)m_collisionShape)->m_links;
	}
	
//
const btSoftBody2::tLinkArray&		btSoftBody2::getLinks() const
	{
		return ((btSoftBody2CollisionShape*)m_collisionShape)->m_links;
	}

//
btSoftBody2::tFaceArray&				btSoftBody2::getFaces()
	{
		return ((btSoftBody2CollisionShape*)m_collisionShape)->m_faces;
	}

//
const btSoftBody2::tFaceArray&		btSoftBody2::getFaces() const
	{
		return ((btSoftBody2CollisionShape*)m_collisionShape)->m_faces;
	}
