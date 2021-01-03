// Physics_SpringForce.cpp: implementation of the Physics_SpringForce class.
//
//////////////////////////////////////////////////////////////////////

#include "DRGShell.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

Physics_SpringForce::Physics_SpringForce()
{
	m_kSpring = 100;
	m_kSpringDamp = 10;
	m_RestDistance = 1;
	m_MaxDistance = (m_real)1.05;
	m_bFixup = false;
    mType= SPRING;
}

Physics_SpringForce::~Physics_SpringForce()
{

}

void Physics_SpringForce::Related( m_real fTime, Physics_LargeVector &p, Physics_LargeVector &v, 
                  vectorn & F0_Related, boolN IsContact, boolN IsRelated, intvectorn RelatedIndex)
{
	if(IsRelated[m_iParticle[0]])
	{
		if(IsContact[m_iParticle[1]])
		{
			vector3 p1, p2, *vv[2], v1, dC_dp[2], force;
			m_real C, C_Dot, len;
			//int i, j;

			p1 = p[m_iParticle[0]];
			p2 = p[m_iParticle[1]];

			vv[0] = &v[m_iParticle[0]];
			vv[1] = &v[m_iParticle[1]];

			VECTOR3_SUB( p2, p1, v1 );

			len = VECTOR3_LENGTH( v1 );
			C = len - m_RestDistance;

			// Given a condition C which we want to be zero, we associate an energy function Ec by
			//           Ec(x)=k/2 C(x)^T C(x)

			// then for each particle i

			//      f_i=- dEc / dx_i = -k dC(x)/dxi C(x)

			// 위에 정의된 C를 사용하여 위 식을 전개하면
			// 아래처럼 우리가 흔히 보는 spring force의 식이 나온다.

			// len=|| p2-p1 ||

			// f_1=-m_k* (p1-p2)/len * C
			// f_2=-m_k* (p2-p1)/len * C

			// dC/dp[0]= (p1-p2)/len
			// dC/dp[1]= (p2-p1)/len

			m_real cnst1 = (m_real)1.0/len;
			m_real cnst2 = (m_real)1.0/len/len/len;
			VECTOR3_SCALE( v1, -cnst1, dC_dp[0] );
			VECTOR3_SCALE( v1, cnst1, dC_dp[1] );

			C_Dot = dC_dp[0].x * vv[0]->x + 
				dC_dp[0].y * vv[0]->y + 
				dC_dp[0].z * vv[0]->z + 
				dC_dp[1].x * vv[1]->x + 
				dC_dp[1].y * vv[1]->y + 
				dC_dp[1].z * vv[1]->z;

			force.x = -m_kSpring * dC_dp[0].x * C - m_kSpringDamp * dC_dp[0].x * C_Dot;
			force.y = -m_kSpring * dC_dp[0].y * C - m_kSpringDamp * dC_dp[0].y * C_Dot;
			force.z = -m_kSpring * dC_dp[0].z * C - m_kSpringDamp * dC_dp[0].z * C_Dot;


			int index;


			for(int i=0; i<RelatedIndex.size(); i++)
			{
				if(RelatedIndex[i]==m_iParticle[0])
					index=i;
			}
	
			printf("m_iParticle[0]=%d \n", m_iParticle[0]);
			printf("RelatedIndex[index]=%d \n",RelatedIndex[index]);
			printf("index=%d \n", index);
	        printf("contact index=%d \n",m_iParticle[1]); 

			F0_Related(index*3) += force.x;
			F0_Related(index*3+1) += force.y;
			F0_Related(index*3+2) += force.z;

		}
	}

	else if(IsRelated[m_iParticle[1]])
	{
		if(IsContact[m_iParticle[0]])
		{
			vector3 p1, p2, *vv[2], v1, dC_dp[2], force;
			m_real C, C_Dot, len;
			//int i, j;

			p1 = p[m_iParticle[0]];
			p2 = p[m_iParticle[1]];

			vv[0] = &v[m_iParticle[0]];
			vv[1] = &v[m_iParticle[1]];

			VECTOR3_SUB( p2, p1, v1 );

			len = VECTOR3_LENGTH( v1 );
			C = len - m_RestDistance;

			m_real cnst1 = (m_real)1.0/len;
			m_real cnst2 = (m_real)1.0/len/len/len;
			VECTOR3_SCALE( v1, -cnst1, dC_dp[0] );
			VECTOR3_SCALE( v1, cnst1, dC_dp[1] );

			C_Dot = dC_dp[0].x * vv[0]->x + 
				dC_dp[0].y * vv[0]->y + 
				dC_dp[0].z * vv[0]->z + 
				dC_dp[1].x * vv[1]->x + 
				dC_dp[1].y * vv[1]->y + 
				dC_dp[1].z * vv[1]->z;

			force.x = -m_kSpring * dC_dp[1].x * C - m_kSpringDamp * dC_dp[1].x * C_Dot;
			force.y = -m_kSpring * dC_dp[1].y * C - m_kSpringDamp * dC_dp[1].y * C_Dot;
			force.z = -m_kSpring * dC_dp[1].z * C - m_kSpringDamp * dC_dp[1].z * C_Dot;

	    	int index;
	
			for(int i=0; i<RelatedIndex.size(); i++)
			{
				if(RelatedIndex[i]==m_iParticle[1])
					index=i;
			}

			printf("m_iParticle[1]=%d \n", m_iParticle[1]);
			printf("RelatedIndex[index]=%d \n",RelatedIndex[index]);
			printf("index=%d \n", index);
	        printf("contact index=%d \n",m_iParticle[0]); 

			F0_Related(index*3) += force.x;
			F0_Related(index*3+1) += force.y;
			F0_Related(index*3+2) += force.z;

		}
	}

}


void Physics_SpringForce::Apply( m_real fTime, Physics_LargeVector &masses, bool bDerivs,
								 Physics_LargeVector &p, Physics_LargeVector &v, 
								 Physics_LargeVector &f_int, Physics_LargeVector &f_ext,
								 Physics_SparseSymmetricMatrix &f_dp, Physics_SparseSymmetricMatrix &f_dv )
{
	vector3 p1, p2, *vv[2], v1, dC_dp[2], force;
	matrix3 dp, d2C_dp2[2][2];
	m_real C, C_Dot, len;
	int i, j;

	p1 = p[m_iParticle[0]];
	p2 = p[m_iParticle[1]];

	vv[0] = &v[m_iParticle[0]];
	vv[1] = &v[m_iParticle[1]];

	VECTOR3_SUB( p2, p1, v1 );

	len = VECTOR3_LENGTH( v1 );
	C = len - m_RestDistance;
	
	// Given a condition C which we want to be zero, we associate an energy function Ec by
	//           Ec(x)=k/2 C(x)^T C(x)

	// then for each particle i
	
	//      f_i=- dEc / dx_i = -k dC(x)/dxi C(x)
	
	// 위에 정의된 C를 사용하여 위 식을 전개하면
	// 아래처럼 우리가 흔히 보는 spring force의 식이 나온다.

	// len=|| p2-p1 ||

	// f_1=-m_k* (p1-p2)/len * C
	// f_2=-m_k* (p2-p1)/len * C
	
	// dC/dp[0]= (p1-p2)/len
	// dC/dp[1]= (p2-p1)/len

	m_real cnst1 = (m_real)1.0/len;
	m_real cnst2 = (m_real)1.0/len/len/len;
	VECTOR3_SCALE( v1, -cnst1, dC_dp[0] );
	VECTOR3_SCALE( v1, cnst1, dC_dp[1] );

	C_Dot = dC_dp[0].x * vv[0]->x + 
			dC_dp[0].y * vv[0]->y + 
			dC_dp[0].z * vv[0]->z + 
			dC_dp[1].x * vv[1]->x + 
			dC_dp[1].y * vv[1]->y + 
			dC_dp[1].z * vv[1]->z;

	force.x = -m_kSpring * dC_dp[0].x * C - m_kSpringDamp * dC_dp[0].x * C_Dot;
	force.y = -m_kSpring * dC_dp[0].y * C - m_kSpringDamp * dC_dp[0].y * C_Dot;
	force.z = -m_kSpring * dC_dp[0].z * C - m_kSpringDamp * dC_dp[0].z * C_Dot;

	VECTOR3_ADD( f_int[m_iParticle[0]], force, f_int[m_iParticle[0]] );

	force.x = -m_kSpring * dC_dp[1].x * C - m_kSpringDamp * dC_dp[1].x * C_Dot;
	force.y = -m_kSpring * dC_dp[1].y * C - m_kSpringDamp * dC_dp[1].y * C_Dot;
	force.z = -m_kSpring * dC_dp[1].z * C - m_kSpringDamp * dC_dp[1].z * C_Dot;

	VECTOR3_ADD( f_int[m_iParticle[1]], force, f_int[m_iParticle[1]] );

	if( bDerivs )
	{
		// 유사한 방식으로 df/dx 매트릭스도 계산할 수 있다. 
		// 논문 David Baraff, Andrew Witkin, Large steps in cloth simulation 참고할 것.

		d2C_dp2[0][0]._11 = -cnst2 * ( (v1.x * v1.x) ) + cnst1 ;
		d2C_dp2[0][0]._12 = 0;
		d2C_dp2[0][0]._13 = 0;
		d2C_dp2[0][0]._21 = 0;
		d2C_dp2[0][0]._22 = -cnst2 * ( (v1.y * v1.y) ) + cnst1 ;
		d2C_dp2[0][0]._23 = 0;
		d2C_dp2[0][0]._31 = 0;
		d2C_dp2[0][0]._32 = 0;
		d2C_dp2[0][0]._33 = -cnst2 * ( (v1.z * v1.z) ) + cnst1 ;

		d2C_dp2[0][1]._11 = cnst2 * ( (v1.x * v1.x) ) - cnst1 ;
		d2C_dp2[0][1]._12 = 0;
		d2C_dp2[0][1]._13 = 0;
		d2C_dp2[0][1]._21 = 0;
		d2C_dp2[0][1]._22 = cnst2 * ( (v1.y * v1.y) ) - cnst1 ;
		d2C_dp2[0][1]._23 = 0;
		d2C_dp2[0][1]._31 = 0;
		d2C_dp2[0][1]._32 = 0;
		d2C_dp2[0][1]._33 = cnst2 * ( (v1.z * v1.z) ) - cnst1 ;

		d2C_dp2[1][0]._11 = cnst2 * ( (v1.x * v1.x) ) - cnst1 ;
		d2C_dp2[1][0]._12 = 0;
		d2C_dp2[1][0]._13 = 0;
		d2C_dp2[1][0]._21 = 0;
		d2C_dp2[1][0]._22 = cnst2 * ( (v1.y * v1.y) ) - cnst1 ;
		d2C_dp2[1][0]._23 = 0;
		d2C_dp2[1][0]._31 = 0;
		d2C_dp2[1][0]._32 = 0;
		d2C_dp2[1][0]._33 = cnst2 * ( (v1.z * v1.z) ) - cnst1 ;

		d2C_dp2[1][1]._11 = -cnst2 * ( (v1.x * v1.x) ) + cnst1 ;
		d2C_dp2[1][1]._12 = 0;
		d2C_dp2[1][1]._13 = 0;
		d2C_dp2[1][1]._21 = 0;
		d2C_dp2[1][1]._22 = -cnst2 * ( (v1.y * v1.y) ) + cnst1 ;
		d2C_dp2[1][1]._23 = 0;
		d2C_dp2[1][1]._31 = 0;
		d2C_dp2[1][1]._32 = 0;
		d2C_dp2[1][1]._33 = -cnst2 * ( (v1.z * v1.z) ) + cnst1 ;

		matrix3 df_dp, df_dv;

		for( i=0; i<2; i++ )
		{
			for( j=i; j<2; j++ )
			{
				dp.setFromOuterProduct( dC_dp[i], dC_dp[j] );
				df_dp._11 = -m_kSpring * ( dp._11 + d2C_dp2[i][j]._11 * C ) -
								 m_kSpringDamp * ( d2C_dp2[i][j]._11 * C_Dot );
				df_dp._12 = -m_kSpring * ( dp._12 + d2C_dp2[i][j]._12 * C ) -
								 m_kSpringDamp * ( d2C_dp2[i][j]._12 * C_Dot );
				df_dp._13 = -m_kSpring * ( dp._13 + d2C_dp2[i][j]._13 * C ) -
								 m_kSpringDamp * ( d2C_dp2[i][j]._13 * C_Dot );
				df_dp._21 = -m_kSpring * ( dp._21 + d2C_dp2[i][j]._21 * C ) -
								 m_kSpringDamp * ( d2C_dp2[i][j]._21 * C_Dot );
				df_dp._22 = -m_kSpring * ( dp._22 + d2C_dp2[i][j]._22 * C ) -
								 m_kSpringDamp * ( d2C_dp2[i][j]._22 * C_Dot );
				df_dp._23 = -m_kSpring * ( dp._23 + d2C_dp2[i][j]._23 * C ) -
								 m_kSpringDamp * ( d2C_dp2[i][j]._23 * C_Dot );
				df_dp._31 = -m_kSpring * ( dp._31 + d2C_dp2[i][j]._31 * C ) -
								 m_kSpringDamp * ( d2C_dp2[i][j]._31 * C_Dot );
				df_dp._32 = -m_kSpring * ( dp._32 + d2C_dp2[i][j]._32 * C ) -
								 m_kSpringDamp * ( d2C_dp2[i][j]._32 * C_Dot );
				df_dp._33 = -m_kSpring * ( dp._33 + d2C_dp2[i][j]._33 * C ) -
								 m_kSpringDamp * ( d2C_dp2[i][j]._33 * C_Dot );

				f_dp(m_iParticle[i],m_iParticle[j])+=df_dp;
#ifdef USE_GMM
				if(i!=j)
					f_dp(m_iParticle[j],m_iParticle[i])+=df_dp;
#endif

				df_dv._11 = -m_kSpringDamp * dp._11;
				df_dv._12 = -m_kSpringDamp * dp._12;
				df_dv._13 = -m_kSpringDamp * dp._13;

				df_dv._21 = -m_kSpringDamp * dp._21;
				df_dv._22 = -m_kSpringDamp * dp._22;
				df_dv._23 = -m_kSpringDamp * dp._23;

				df_dv._31 = -m_kSpringDamp * dp._31;
				df_dv._32 = -m_kSpringDamp * dp._32;
				df_dv._33 = -m_kSpringDamp * dp._33;

				f_dv(m_iParticle[i], m_iParticle[j])+=df_dv;
#ifdef USE_GMM
				if(i!=j)
					f_dv(m_iParticle[j], m_iParticle[i])+=df_dv;
#endif

			}
		}
	}
 }

void Physics_SpringForce::PrepareMatrices( Physics_SymmetricMatrix &A, Physics_SparseSymmetricMatrix &B)
{

	A(m_iParticle[0], m_iParticle[1]) += m_kSpring;
	A(m_iParticle[0], m_iParticle[0]) -= m_kSpring;
	A(m_iParticle[1], m_iParticle[1]) -= m_kSpring;

#ifdef USE_GMM
	A(m_iParticle[1], m_iParticle[0]) += m_kSpring;

	matrix3 temp;

	for( int i=0; i<2; i++ )
		for( int j=0; j<2; j++ )
		{
			temp = B(m_iParticle[i], m_iParticle[j]);
			temp._11=temp._11*2.0;
			B(m_iParticle[i], m_iParticle[j])=temp;
		}
#else
	m_real temp;

	for( int i=0; i<2; i++ )
		for( int j=i; j<2; j++ )
		{
			temp = B(m_iParticle[i], m_iParticle[j])._11;
			B(m_iParticle[i], m_iParticle[j])._11 = temp * (m_real)2.0;
		}
#endif
}

void Physics_SpringForce::Fixup( Physics_LargeVector &invmasses, Physics_LargeVector &p )
{	
	if( m_bFixup )
	{
		vector3 Dist;
		m_real len;

		VECTOR3_SUB( p[m_iParticle[0]], p[m_iParticle[1]], Dist ); 
		len = Dist.length();

		if( len > m_MaxDistance ) 
		{
			len -= m_MaxDistance;
			len /= 2;
			Dist.normalize();
			VECTOR3_SCALE( Dist, len, Dist );
			if( invmasses[ m_iParticle[0] ].x )
			{
				if( invmasses[ m_iParticle[1] ].x )
				{
					p[m_iParticle[0]].x -= Dist.x;
					p[m_iParticle[0]].y -= Dist.y;
					p[m_iParticle[0]].z -= Dist.z;
					p[m_iParticle[1]].x += Dist.x;
					p[m_iParticle[1]].y += Dist.y;
					p[m_iParticle[1]].z += Dist.z;
				}
				else
				{
					p[m_iParticle[0]].x -= Dist.x*(m_real)2.0;
					p[m_iParticle[0]].y -= Dist.y*(m_real)2.0;
					p[m_iParticle[0]].z -= Dist.z*(m_real)2.0;
				}
			}
			else
			{
				if( invmasses[ m_iParticle[1] ].x )
				{
					p[m_iParticle[1]].x += Dist.x*(m_real)2.0;
					p[m_iParticle[1]].y += Dist.y*(m_real)2.0;
					p[m_iParticle[1]].z += Dist.z*(m_real)2.0;
				}
			}
		}
	}
}
