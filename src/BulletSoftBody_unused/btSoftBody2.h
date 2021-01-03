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

#ifndef _BT_SOFT_BODY2_H
#define _BT_SOFT_BODY2_H

#include "LinearMath/btAlignedObjectArray.h"
#include "LinearMath/btPoint3.h"
#include "LinearMath/btTransform.h"
#include "LinearMath/btIDebugDraw.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"

#include "BulletCollision/CollisionShapes/btConcaveShape.h"
#include "BulletCollision/CollisionDispatch/btCollisionCreateFunc.h"
#include "btSparseSDF.h"
#include "btDbvt.h"

class btBroadphaseInterface;
class btCollisionDispatcher;


class Physics_ParticleSystem ;
class Physics_Impulse;
class Physics_SpringForce;
class vector3;
class vector3N;
class intvectorn;

/// btSoftBody2 is work-in-progress
class	btSoftBody2 : public btCollisionObject
{
public:
	//
	// Enumerations
	//

	///eLType
	struct eLType { enum _ {
		Structural,	///Master constraints 
		Bending,	///Secondary constraints
	};};

	//
	// Flags
	//
	
	///fCollision
	struct fCollision { enum _ {
		RVSmask	=	0x000f,	///Rigid versus soft mask
		SDF_RS	=	0x0001,	///SDF base rigid vs soft
		
		SVSmask	=	0x00f0,	///Rigid versus soft mask		
		VF_SS	=	0x0010,	///Vertex vs face soft vs soft handling
		/* presets	*/ 
		Default	=	SDF_RS,
	};};
	
	//
	// API Types
	//
	
	/* sRayCast		*/ 
	struct sRayCast
	{
		int			face;	/// face
		btScalar	time;	/// time of impact (rayorg+raydir*time)
	};
	
	//
	// Internal types
	//

	typedef btAlignedObjectArray<btScalar>	tScalarArray;
	typedef btAlignedObjectArray<btVector3>	tVector3Array;

	/* btSoftBody2WorldInfo	*/ 
	struct	btSoftBody2WorldInfo
	{
		btSoftBody2WorldInfo()
		{
			m_broadphase=NULL;
			m_dispatcher=NULL;
		}
		~btSoftBody2WorldInfo(){cleanup();}

		void cleanup()
		{
			delete m_broadphase;
			m_broadphase=NULL;
			delete m_dispatcher;
			m_dispatcher=NULL;
		}

		btBroadphaseInterface*	m_broadphase;
		btCollisionDispatcher*	m_dispatcher;
		btVector3				m_gravity;
		btSparseSdf<3>			m_sparsesdf;
	};	

	/* sCti is Softbody contact info	*/ 
	struct	sCti
	{
		btRigidBody*	m_body;		/* Rigid body			*/ 
		btVector3		m_normal;	/* Outward normal		*/ 
		//btScalar		m_offset;	/* Offset from origin	*/ 
		btScalar		m_dp;	// penetrating depth
		btScalar		m_dp2;	// penetrating depth (with respect to margin offset)
	};	

	
	/* Base type	*/ 
	struct	Element
	{
		void*			m_tag;			// User data
	};

	/* Node			*/ 
	struct	Node : Element
	{
		btVector3				m_n;			// Normal
		btScalar				m_area;			// Area
		btDbvt::Node*			m_leaf;			// Leaf data
		int						m_battach:1;	// Attached

		// taesoo
		btVector3 x() const;
		vector3& x_ref();
		btVector3 v() const;
		vector3& v_ref();
		btScalar im() const;	// inverse mass
		Physics_ParticleSystem *mpSystem;
		int m_index;
	};
	/* Link			*/ 
	struct	Link : Element
	{
		Node*					m_n[2];			// Node pointers
		Physics_SpringForce* m_pSpring;
		btScalar				m_c0;			// (ima+imb)
		btScalar				m_c1;			// rl^2
		btSoftBody2::eLType::_	m_type;			// Link type
	};
	/* Face			*/ 
	struct	Face : Element
	{
		Node*					m_n[3];			// Node pointers
		btVector3				m_normal;		// Normal
		btScalar				m_ra;			// Rest area
		btDbvt::Node*			m_leaf;			// Leaf data
	};
	/* RContact		*/ 
	struct	RContact
	{
		btSoftBody2::sCti		m_cti;			// Contact infos
		Node*					m_node;			// Owner node
		btMatrix3x3				m_c0;			// Impulse matrix
		btVector3				m_c1;			// Relative anchor
		btScalar				m_c2;			// ima*dt
	};
	/* SContact		*/ 
	struct	SContact
	{
		Node*					m_node;			// Node
		Face*					m_face;			// Face
		btVector3				m_weights;		// Weigths
		btVector3				m_normal;		// Normal
		btScalar				m_margin;		// Margin
		btScalar				m_friction;		// Friction
		btScalar				m_cfm[2];		// Constraint force mixing
	};
	/* Anchor		*/ 
	struct	Anchor
	{
		Node*					m_node;			// Node pointer
		btVector3				m_local;		// Anchor position in body space
		btRigidBody*			m_body;			// Body
		btMatrix3x3				m_c0;			// Impulse matrix
		btVector3				m_c1;			// Relative anchor
		btScalar				m_c2;			// ima*dt
	};
	
	/* DFld			*/ 
	struct	DFld
	{
		btAlignedObjectArray<btVector3>	pts;
	};

	/* Config		*/ 
	struct	Config
	{
		btScalar				kSpringDamp;
		btScalar				kSHR;			// Soft contacts hardness [0,1]
		btScalar				kAHR;			// Anchors hardness [0,1]
		btScalar				timescale;		// Time scale
		int						collisions;		// Collisions flags
	};

	/* SolverState	*/ 
	struct	SolverState
	{
		btScalar				sdt;			// dt*timescale
		btScalar				isdt;			// 1/sdt
		btScalar				velmrg;			// velocity margin
		btScalar				radmrg;			// radial margin
		btScalar				updmrg;			// Update margin
	};

	//
	// Typedef's
	//

	typedef btAlignedObjectArray<Node>			tNodeArray;
	typedef btAlignedObjectArray<btDbvt::Node*>	tLeafArray;
	typedef btAlignedObjectArray<Link>			tLinkArray;
	typedef btAlignedObjectArray<Face>			tFaceArray;
	typedef btAlignedObjectArray<Anchor>		tAnchorArray;
	typedef btAlignedObjectArray<RContact>		tRContactArray;
	typedef btAlignedObjectArray<SContact>		tSContactArray;
	typedef btAlignedObjectArray<btSoftBody2*>	tSoftBodyArray;

	//
	// Fields
	//
	Config					m_cfg;			// Configuration
	SolverState				m_sst;			// Solver state
	DFld					m_dfld;			// Distance field
	void*					m_tag;			// User data
	btSoftBody2WorldInfo*	m_worldInfo;	//
	tAnchorArray			m_anchors;		// Anchors
	tRContactArray			m_rcontacts;	// Rigid contacts
	tSContactArray			m_scontacts;	// Soft contacts
	btScalar				m_timeacc;		// Time accumulator
	btVector3				m_bounds[2];	// Spatial bounds	
	bool					m_bUpdateRtCst;	// Update runtime constants
	btDbvt					m_ndbvt;		// Nodes tree
	btDbvt					m_fdbvt;		// Faces tree
	
	//
	// Api
	//
	
	/* ctor																	*/ 
	btSoftBody2(	btSoftBody2::btSoftBody2WorldInfo* worldInfo,int node_count,
				const btVector3* x,
				const btScalar* m, double kSpringDamp=10, int integrateMethod=0);
	/* dtor																	*/ 
	virtual ~btSoftBody2();
	/* Check for existing link												*/ 
	bool				checkLink(	int node0,
		int node1) const;
	bool				checkLink(	const btSoftBody2::Node* node0,
		const btSoftBody2::Node* node1) const;
	/* Check for existring face												*/ 
	bool				checkFace(	int node0,
		int node1,
		int node2) const;
	/* Append link															*/ 
	void				appendLink(		int node0,
		int node1,
		btScalar kST,
		btSoftBody2::eLType::_ type,
		bool bcheckexist=false);
	void				appendLink(		btSoftBody2::Node* node0,
		btSoftBody2::Node* node1,
		btScalar kST,
		btSoftBody2::eLType::_ type,
		bool bcheckexist=false);
	/* Append face															*/ 
	void				appendFace(		int node0,
		int node1,
		int node2);
	/* Append anchor														*/ 
	void				appendAnchor(	int node,
		btRigidBody* body);
	/* Set mass																*/ 
	void				setMass(		int node,
		btScalar mass);
	/* Get mass																*/ 
	btScalar			getMass(		int node) const;
	/* Get total mass														*/ 
	btScalar			getTotalMass() const;
	/* Set total mass (weighted by previous masses)							*/ 
	void				setTotalMass(	btScalar mass,
		bool fromfaces=false);
	/* Set total density													*/ 
	void				setTotalDensity(btScalar density);
	/* Transform															*/ 
	void				transform(		const btTransform& trs);
	/* Scale																*/ 
	void				scale(			const btVector3& scl);
	/* Return the volume													*/ 
	btScalar			getVolume() const;
	/* Generate bending constraints based on distance in the adjency graph	*/ 
	int					generateBendingConstraints(	int distance,
		btScalar stiffness);
	/* Randomize constraints to reduce solver bias							*/ 
	void				randomizeConstraints();
	/* Ray casting															*/ 
	bool				rayCast(const btVector3& org,
								const btVector3& dir,
								sRayCast& results,
								btScalar maxtime=SIMD_INFINITY);
	/* predictMotion														*/ 
	void				predictMotion(btScalar dt);
	/* solveConstraints														*/ 
	void				solveConstraints();
	/* solveCommonConstraints												*/ 
	static void			solveCommonConstraints(btSoftBody2** bodies,int count,int iterations);
	/* integrateMotion														*/ 
	void				integrateMotion();
	/* defaultCollisionHandlers												*/ 
	void				defaultCollisionHandler(btCollisionObject* pco);
	void				defaultCollisionHandler(btSoftBody2* psb);
	void addExternalForce(intvectorn const& indices, vector3N const& forces);
	const intvectorn* _indices;
	const vector3N* _forces;
	
	//
	// Accessor's and cast.
	//
	
	tNodeArray&			getNodes();
	const tNodeArray&	getNodes() const;
	tLinkArray&			getLinks();
	const tLinkArray&	getLinks() const;
	tFaceArray&			getFaces();
	const tFaceArray&	getFaces() const;

	
	Physics_ParticleSystem *m_pSystem;
	//
	// Cast
	//
		
	static const btSoftBody2*	upcast(const btCollisionObject* colObj)
	{
		if (colObj->getInternalType()==CO_SOFT_BODY2)
			return (const btSoftBody2*)colObj;
		return 0;
	}
	static btSoftBody2*			upcast(btCollisionObject* colObj)
	{
		if (colObj->getInternalType()==CO_SOFT_BODY2)
			return (btSoftBody2*)colObj;
		return 0;
	}
	
};



#endif //_BT_SOFT_BODY2_H
