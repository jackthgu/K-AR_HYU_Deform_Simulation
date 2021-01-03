// Physics_Constraint.cpp: implementation of the Physics_Constraint class.
//
//////////////////////////////////////////////////////////////////////

#include "DRGShell.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

Physics_Constraint::Physics_Constraint( int iParticle, int iDegreesOfFreedom, vector3 axis1, vector3 axis2, m_real ConstrainedVelocity)
{
	m_iParticle = iParticle;
	m_iDegreesOfFreedom = iDegreesOfFreedom;
	m_p = axis1;
	m_q = axis2;
	m_ConstrainedVelocity.setValue(ConstrainedVelocity,ConstrainedVelocity,ConstrainedVelocity);
	m_op1.setFromOuterProduct( m_p, m_p );
	m_op2.setFromOuterProduct( m_q, m_q );
}

Physics_Constraint::~Physics_Constraint()
{

}

void Physics_Constraint::Apply( matrix3 S[], Physics_LargeVector &z )
{
	matrix3 I, tmp;

	I.identity();
	z[m_iParticle] = m_ConstrainedVelocity;

	switch( m_iDegreesOfFreedom )
	{
		case 0:
			S[m_iParticle].zero();
			break;
		case 1:
			I.Subtract( m_op1, tmp );
			tmp.Subtract( m_op2, S[m_iParticle] );
			break;
		case 2:
			I.Subtract( m_op1, S[m_iParticle] );
			break;
		case 3:
		default:
			S[m_iParticle].identity();
			z[m_iParticle] .setValue(0,0,0);
			break;
	}
}

