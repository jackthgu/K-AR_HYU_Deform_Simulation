// Physics_ParticleSystem.h: interface for the Physics_ParticleSystem class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_PHYSICS_PARTICLESYSTEM_H__76E37C60_EAA1_49C0_A098_3772AE8AA64C__INCLUDED_)
#define AFX_PHYSICS_PARTICLESYSTEM_H__76E37C60_EAA1_49C0_A098_3772AE8AA64C__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#define INTEGRATE_EXPLICIT			0
#define INTEGRATE_IMPLICIT			1
#define INTEGRATE_SEMI_IMPLICIT		2

#include <vector>
class Physics_Contacts;
class Physics_ParticleSystem  
{
public:
	struct Config
	{
		vectorn m_dynamicFrictionCoef;
		m_real DFthresholdVel;
		
		enum {PENALTY_METHOD, LCP_METHOD_FRICTIONLESS, LCP_METHOD, BARAFF98};
		int contactMethod;
		m_real penaltyStiffness;
		m_real penaltyDampness;
		m_real penaltyMuScale;
	};

	Config m_cfg;
	int m_iParticles, m_iInverseIterations;
	Physics_LargeVector m_Positions;
	//Physics_LargeVector m_Locations;
	Physics_LargeVector m_Velocities;
	Physics_LargeVector m_TotalForces_int, m_TotalForces_ext;
	Physics_LargeVector m_dv;
	//Physics_LargeVector m_vContactForce;	// output
	int numForces() { return m_Forces.size();}
	Physics_LargeVector m_vTemp1, m_vTemp2, m_vTemp3, m_PosTemp;
	Physics_LargeVector m_dp;
	
	Physics_LargeVector m_Masses, m_MassInv;

	//
	// This stuff is for the semi-Implicit Desbrun method
	//
	Physics_SymmetricMatrix m_H, m_W;
	m_real m_LastStep, m_ParticleMass;

	//
	// This stuff is for the Implicit implementation
	//
	Physics_SparseSymmetricMatrix m_MxMasses;
	Physics_SparseSymmetricMatrix m_TotalForces_dp;
	Physics_SparseSymmetricMatrix m_TotalForces_dv;

	Physics_SparseSymmetricMatrix m_A;
	Physics_SparseSymmetricMatrix m_MxTemp1, m_MxTemp2;
	Physics_LargeVector m_P, m_PInv;
	Physics_LargeVector m_z, m_b, m_r, m_c, m_q, m_s, m_y;
	matrix3 *m_S;

#pragma warning(disable:4251)
	
	std::vector<Physics_Force*> m_Forces;
	std::list<Physics_Constraint*> m_Constraints;
#pragma warning(default:4251)


	
public:
	Physics_Contacts mContacts;
	Physics_Contacts_Penalty mContacts_peneltyMethod;

	int m_iIntegrationMethod;

	void SetupMatrices();
	Physics_ParticleSystem( int iParticles );
	virtual ~Physics_ParticleSystem();

	inline int numParticle()	{ return m_Positions.Size();}
	inline vector3 &Position( int iParticle )
	{
		return m_Positions[iParticle];
	}

	inline vector3 &PositionCorrection( int iParticle )
	{
		return m_y[iParticle];
	}

	inline vector3 &Velocity( int iParticle )
	{
		return m_Velocities[iParticle];
	}

	inline vector3 GetMass( int iParticle )
	{
		return m_Masses[iParticle];
	}

	inline Physics_Force& Force(int iForce)
	{
		return *m_Forces[iForce];
	}
	inline void SetMass( int iParticle, m_real mass )
	{
		m_Masses[iParticle] = vector3( mass, mass, mass );
		if( mass > (m_real)0.0 )
		{
			m_ParticleMass = mass;
			m_MassInv[iParticle].x = (m_real)1.0 / mass;
			m_MassInv[iParticle].y = (m_real)1.0 / mass;
			m_MassInv[iParticle].z = (m_real)1.0 / mass;
			// ys
			matrix3 t;
			t.zero();
			t._11 = mass;
			t._22 = mass;
			t._33 = mass;
			m_MxMasses(iParticle, iParticle)=t;
		}
		else
		{
			m_MassInv[iParticle].x = (m_real)0;
			m_MassInv[iParticle].y = (m_real)0;
			m_MassInv[iParticle].z = (m_real)0;
			// ys
			matrix3 t;
			t.zero();
			t._11 = 0;
			t._22 = 0;
			t._33 = 0;
			m_MxMasses(iParticle, iParticle)=t;
		}
	}

	inline Physics_LargeVector &GetPositions()
	{
		return m_Positions;
	}

	void AddForce( Physics_Force &Force )
	{
		m_Forces.push_back( &Force );
	}

	void AddConstraint( Physics_Constraint &Constraint );
	bool DeleteConstraint( Physics_Constraint &Constraint );

	void Update( float fTime );

	void UpdateDV_Implicit( float fTime );
	void UpdateDV_Implicit_part1( float fTime );
	void UpdateDV_Implicit_part2( float fTime );
	void IntegrateDV(double fStep);

	void Update_Explicit( float fTime );
	void Update_Implicit( float fTime );
	void Update_SemiImplicit( float fTime );
	void Reset();
};

#endif // !defined(AFX_PHYSICS_PARTICLESYSTEM_H__76E37C60_EAA1_49C0_A098_3772AE8AA64C__INCLUDED_)
