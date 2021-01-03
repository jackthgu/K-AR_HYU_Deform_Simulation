// Physics_GravityForce.cpp: implementation of the Physics_GravityForce class.
//
//////////////////////////////////////////////////////////////////////

#include "DRGShell.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

Physics_GravityForce::Physics_GravityForce( vector3 const&dir )
{
	m_Direction = dir;
    mType= GRAVITY;
}

Physics_GravityForce::~Physics_GravityForce()
{

}

void Physics_GravityForce::Apply( m_real fTime, Physics_LargeVector &masses, bool bDerivs,
								  Physics_LargeVector &p, 
								  Physics_LargeVector &v, 
								  Physics_LargeVector &f_int, Physics_LargeVector &f_ext,
								  Physics_SparseSymmetricMatrix &f_dp, Physics_SparseSymmetricMatrix &f_dv )
{
	for( int i=0; i<f_ext.size(); i++ )
	{
		f_ext[i].x += m_Direction.x * masses[i].x;
		f_ext[i].y += m_Direction.y * masses[i].y;
		f_ext[i].z += m_Direction.z * masses[i].z;
	}
}
