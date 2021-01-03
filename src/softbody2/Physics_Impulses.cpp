// Physics_Impulse.cpp: implementation of the Physics_Impulse class.
//
//////////////////////////////////////////////////////////////////////

#include "DRGShell.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////


Physics_Contacts ::~Physics_Contacts ()
{

}

/*
void Physics_Impulse::Apply( m_real fTime, Physics_LargeVector &masses, bool bDerivs,
								  Physics_LargeVector &p, 
								  Physics_LargeVector &v, 
								  Physics_LargeVector &f_int, Physics_LargeVector &f_ext,
								  Physics_SparseSymmetricMatrix &f_dp, Physics_SparseSymmetricMatrix &f_dv )
{
	for( int i=0; i<f_ext.m_iElements; i++ )
	{
		f_ext.m_pData[i].x += desiredDeltaVelocity.m_pData[i].x * masses.m_pData[i].x / fTime;
		f_ext.m_pData[i].y += desiredDeltaVelocity.m_pData[i].y * masses.m_pData[i].y / fTime;
		f_ext.m_pData[i].z += desiredDeltaVelocity.m_pData[i].z * masses.m_pData[i].z / fTime;
	}

	desiredDeltaVelocity.zero();
}
*/

Physics_Contacts ::Physics_Contacts (int nparticles)
		:relativeVelocity(nparticles),
		deltaPosition(nparticles),
		normal(nparticles),
		prevRelativeVelocity(nparticles),
		prevNormal(nparticles)
{
	contactStatus.resize(nparticles);
	prevContactStatus.resize(nparticles);
	contactInfo.resize(nparticles);
	relativeVelocity.zero();
	deltaPosition.zero();
	normal.zero();
	for(int i=0; i<relativeVelocity.size(); i++)
	{
		contactStatus[i]=no_contact;
		prevContactStatus[i]=no_contact;
		contactInfo[i]=NULL;
	}
}
