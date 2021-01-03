// Physics_Constraint.h: interface for the Physics_Constraint class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_PHYSICS_CONSTRAINT_H__53433219_3427_4780_84A6_EC806B0F603B__INCLUDED_)
#define AFX_PHYSICS_CONSTRAINT_H__53433219_3427_4780_84A6_EC806B0F603B__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

class Physics_Constraint  
{
private:
	int m_iParticle;
	int m_iDegreesOfFreedom;
	vector3 m_p;
	vector3 m_q;
	vector3 m_ConstrainedVelocity;
	matrix3 m_op1, m_op2;

public:
	virtual void Apply( matrix3 S[], Physics_LargeVector &z );	
	Physics_Constraint( int iParticle, int iDegreesOfFreedom, 
						vector3 axis1 = vector3(0,1,0),
						vector3 axis2 = vector3(0,0,1), m_real ConstrainedVelocity = 0 );
	virtual ~Physics_Constraint();
};

#endif // !defined(AFX_PHYSICS_CONSTRAINT_H__53433219_3427_4780_84A6_EC806B0F603B__INCLUDED_)
