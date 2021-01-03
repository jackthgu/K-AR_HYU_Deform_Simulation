// Physics_Physics_ParticleSystem.cpp: implementation of the Physics_Physics_ParticleSystem class.
//
//////////////////////////////////////////////////////////////////////

#include "DRGShell.h"
#include "Physics_Impulses.h"
#include "../../gmm.h"
#include "../../../../BaseLib/math/Operator.h"
#include "../../../../BaseLib/math/Operator_NR.h"
#include "../../../../PhysicsLib/sDIMS/lcp.h"
#include "../../../QP_controller/quadprog.h"
#include "../../../../MainLib/OgreFltk/objectList.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

//#define FLOOR_Y	((m_real)-0.99)
#define FLOOR_Y	((m_real)30)

void fMat_setReference(fMat& mat, matrixn const& matMine)
{
	double* buffer;
	int stride, n, m, on;
	matMine._getPrivate(buffer, stride, n, m, on);
	Msg::verify(stride==n, "fMat_set failed!");
	mat.setReference(buffer, n, m);
}

Physics_ParticleSystem::Physics_ParticleSystem( int iParticles ) :
	m_Positions( iParticles ), m_Velocities( iParticles ),
	m_dv( iParticles ), m_dp( iParticles ), 
	m_vTemp1( iParticles ), m_vTemp2( iParticles ), m_vTemp3( iParticles ),
	m_PosTemp( iParticles ),
	m_TotalForces_int( iParticles ), m_TotalForces_ext( iParticles ),
	m_W( iParticles ), m_H( iParticles ),
	m_Masses( iParticles ), m_MassInv( iParticles ),
	m_MxMasses( iParticles, iParticles ),
	m_TotalForces_dp( iParticles, iParticles ),
	m_TotalForces_dv( iParticles, iParticles ),
	m_A( iParticles, iParticles ),
	m_P( iParticles ),
	m_PInv( iParticles ),
	m_MxTemp1( iParticles, iParticles ), m_MxTemp2( iParticles, iParticles ),
	m_z( iParticles ), m_b( iParticles ), m_r( iParticles ), 
	m_c( iParticles ), m_q( iParticles ), m_s( iParticles ), m_y( iParticles ),
	mContacts(iParticles),
	m_vContactForce(iParticles)
{
	//m_iIntegrationMethod = INTEGRATE_SEMI_IMPLICIT;
	m_iIntegrationMethod = INTEGRATE_IMPLICIT;

	m_cfg.m_dynamicFrictionCoef.resize(iParticles);
	m_cfg.m_dynamicFrictionCoef.setAllValue(0.0);
	m_cfg.DFthresholdVel=0.1;
	m_cfg.contactMethod=Config::LCP_METHOD;
	matrix3 matI;
	matI.identity();
	for( int i=0; i<iParticles; i++ )
	{
		m_Masses[i] = vector3( 1,1,1 );
		m_MassInv[i] = vector3( 1,1,1 );
		m_MxMasses(i,i)=matI;
	}

	m_S = new matrix3[iParticles];
	m_iParticles = iParticles;


	m_vContactForce.zero();
	m_TotalForces_int.zero();
	m_TotalForces_ext.zero();
	m_LastStep = 1.0;
}

Physics_ParticleSystem::~Physics_ParticleSystem()
{
	SAFE_DELETE_ARRAY( m_S );

	for(int i=0; i<m_Forces.size(); i++)
		delete m_Forces[i];
}

void Physics_ParticleSystem::Update( float fTime )
{
	switch( m_iIntegrationMethod )
	{
		case INTEGRATE_EXPLICIT:
			Update_Explicit( fTime );
			break;
		case INTEGRATE_IMPLICIT:
			Update_Implicit( fTime );
			break;
		case INTEGRATE_SEMI_IMPLICIT:
			Update_SemiImplicit( fTime );
			break;
	}
}

void Physics_ParticleSystem::SetupMatrices()
{

	Physics_Force *pForce ;
	//DWORD dwTimeIn = GetTickCount(), dwTimeOut;

	if( m_iIntegrationMethod != INTEGRATE_EXPLICIT )
	{
		m_H.zero();

		for(int i=0, ni=m_Forces.size(); i<ni; i++)
		{			
			m_Forces[i]->PrepareMatrices(m_H, m_TotalForces_dp);
		}	
	}

	//
	// Setup the implicit matrices
	//
	if( m_iIntegrationMethod == INTEGRATE_IMPLICIT )
	{
		m_TotalForces_dv.Copy( m_TotalForces_dp );
		m_A.Copy( m_TotalForces_dp );
		m_MxTemp1.Copy( m_TotalForces_dp );
		m_MxTemp2.Copy( m_TotalForces_dp );
	}

	//
	// Setup the semi-implicit matrices
	//
	if( m_iIntegrationMethod == INTEGRATE_SEMI_IMPLICIT )
	{
		m_W.identity();
		m_H.Scale( (m_real)1.0/m_ParticleMass, m_H );
		m_W.Subtract( m_H, m_W );
		m_W.Invert();
	}


	//dwTimeOut = GetTickCount();
	//char szTemp[50];
	//sprintf( szTemp, "Initialize: %lu msec\r\n", dwTimeOut - dwTimeIn );
	//Msg::print("%s\n", szTemp );

}

//
// Use Explicit integration with Deformation Constraints
//
void Physics_ParticleSystem::Update_Explicit( float fTime )
{
	m_real fTotal = 0.0f, fStep = (m_real)fTime;
	vector3 vCOG, dTorque, tmp, tmp2;
	int i;

	while( fTotal < fTime )
	{
		// 
		// Zero out everything
		//
		m_TotalForces_int.zero(); m_TotalForces_ext.zero();


		//
		// Apply the forces
		//
		for(int i=0, ni=m_Forces.size(); i<ni; i++) {
			m_Forces[i]->Apply(fStep, m_Masses, false, m_Positions,
					m_Velocities, m_TotalForces_int, m_TotalForces_ext,
					m_TotalForces_dp, m_TotalForces_dv ); }


		//
		// Compute the new velocities and positions
		//
		m_TotalForces_ext.Add( m_TotalForces_int, m_vTemp1 );
		m_vTemp1.ElementMultiply( m_MassInv, m_dv ); m_dv.Scale( fStep, m_dv );
		m_Velocities.Add( m_dv, m_Velocities ); m_Velocities.Scale( fStep,
				m_vTemp1 ); m_Positions.Add( m_vTemp1, m_PosTemp );

		//
		// Apply inverse dynamics to prevent excessive stretch We need 10 or so
		// iterations to be stable -- with more particles in the mesh, we need
		// more iterations.
		//
		for( i=0; i<m_iInverseIterations; i++ ) { for(int i=0,
				ni=m_Forces.size(); i<ni; i++) { m_Forces[i]->Fixup( m_Masses,
				m_PosTemp ); } }
		//
		// Update velocity and position
		//
		m_PosTemp.Subtract( m_Positions, m_vTemp1 ); m_vTemp1.Scale(
				(m_real)1.0 / fStep, m_Velocities );
		/*for( i=0; i<m_iParticles; i++ )
		  if( m_PosTemp[i].y < FLOOR_Y ) { m_PosTemp[i].y = FLOOR_Y;
		  m_Velocities[i].y = 0; }*/ m_Positions = m_PosTemp;

		fTotal += (m_real)fabs( fStep ); 
	} 
}

void Physics_ParticleSystem::Update_Implicit( float fTime )
{ 
	int i,iIterations = 0, iMaxIterations = (int)sqrt(double(m_iParticles))+3; m_real
		fTotal = (m_real)0.0f, fStep = (m_real)fTime; m_real alpha, Delta_0,
			   Delta_old, Delta_new; m_real Eps_Sq = (m_real)1e-22;

	while( fTotal < fTime )
	{
		// 
		// Zero out everything
		//
		m_TotalForces_int.zero(); m_TotalForces_ext.zero();
		m_TotalForces_dp.zero(); m_TotalForces_dv.zero(); m_MxTemp1.zero();
		m_MxTemp2.zero(); m_vTemp3.zero(); 

		// Setting m_S and m_y
		Reset();


		void *Pos = NULL;
		for(int i=0, ni=m_Forces.size(); i<ni; i++)
		{
			m_Forces[i]->Apply( fStep, m_Masses, true, m_Positions, m_Velocities, 
					m_TotalForces_int, m_TotalForces_ext, 
					m_TotalForces_dp, m_TotalForces_dv );
		}

		//
		// Form the symmetric matrix A = M - h * df/dv - h_2 * df/dx (Baraff 논문의 equation 6의 양변에 M을 곱한식)
		// We regroup this as A = M - h * (df/dv + h * df/dx)
		//
		m_TotalForces_int+=m_TotalForces_ext;

		m_MxTemp1.mult(m_TotalForces_dp, fStep);
		m_MxTemp2.add(m_MxTemp1,m_TotalForces_dv);
		m_MxTemp1.mult(m_MxTemp2, fStep);
		m_A .sub(m_MxMasses,m_MxTemp1);



		static ObjectList* g_debugObject=NULL;

		if (g_debugObject==NULL)
			g_debugObject=new ObjectList();

		matrixn lines_fvelocity(0, 3);
		matrixn lines_fforce(0, 3);
	
		matrixn lines_flvelocity(0, 3);
		matrixn lines_flforce(0, 3);

	    matrixn lines_normal(0, 3);

		g_debugObject->registerObject("friction_contactForces", "LineList", "solidred", lines_fforce);
		g_debugObject->registerObject("friction_deltaVelocities", "LineList", "solidblue", lines_fvelocity);

		g_debugObject->registerObject("frictionless_contactForces", "LineList", "solidwhite", lines_flforce);
		g_debugObject->registerObject("frictionless_deltaVelocities", "LineList", "solidblack", lines_flvelocity);
	
		g_debugObject->registerObject("normalForces", "LineList", "solidgreen", lines_normal);

		if(m_cfg.contactMethod==Config::LCP_METHOD_FRICTIONLESS && mContacts_peneltyMethod.m_cti.size())
		{			
		    // n: number of contact points
			int n=mContacts_peneltyMethod.m_cti.size();

			// m: the number of particles involved in contact.  at the moment,
			// m==n because we only consider FV collisions ignoring VF and EE
			// collisions.
			int m=n;	
			// following matrix naming conventions follow [DAK04] and [DDKA06].
			// [DAK04] C.Duriez, C. Andriot, and A.Kheddar, A multi-threaded
			// approach for deformable/rigid contacts with haptic feedback,
			// IEEE haptic symposium, 2004.

			// [DDKA06] Realistic Haptic Rendering of Interacting Deformable
			// Objects in Virtual Environments
			gmm::wsmatrixn H_v,H_f, H_D;

			H_v.resize(3*n,4*m);
			H_f.resize(3*n,4*m);
			H_D.resize(m, 3*m);

			vector3  u, v, tmp; 
			vector3  tmp1, tmp2, tmp3, tmp4; m_real a,b;
			a=(m_real)0.5; b=(m_real)0.3;	
			//quater q;

			vectorn a0;
			a0.setSize(m);

			for(int i=0; i<n; i++)
			{
				Physics_Contacts_Penalty::cti& c=mContacts_peneltyMethod.m_cti[i];
				m_real a_temp=(m_real)c.penetratingDepth;
				a_temp*=(m_real)1.0/fStep;
				a0(i)=a_temp; 

				H_D(i , i*3)=c.normal.x;
				H_D(i , i*3+1)=c.normal.y;
				H_D(i , i*3+2)=c.normal.z;

				u.cross( c.normal, vector3(0,0,1));
				u.normalize();
				v.cross( c.normal, u);
				v.normalize();	

				//VECTOR3_SCALE( u, a , tmp );
				//VECTOR3_ADD( c.normal, tmp, tmp1);
				//VECTOR3_SUB( c.normal, tmp, tmp2);	
				tmp1=c.normal+u*a;
				tmp2=c.normal-u*a;
				tmp3=c.normal+v*a;
				tmp4=c.normal-v*a;

				tmp1.normalize(); tmp2.normalize(); tmp3.normalize(); tmp4.normalize();

				//VECTOR3_SCALE( v, a , tmp );
				//VECTOR3_ADD( c.normal, tmp, tmp3);
				//VECTOR3_SUB( c.normal, tmp, tmp4);

				H_f(i*3, i*4)=tmp1.x; H_f(i*3, i*4+1)=tmp2.x;
				H_f(i*3, i*4+2)=tmp3.x; H_f(i*3, i*4+3)=tmp4.x;

				H_f(i*3+1, i*4)=tmp1.y; H_f(i*3+1, i*4+1)=tmp2.y;
				H_f(i*3+1,i*4+2)=tmp3.y; H_f(i*3+1, i*4+3)=tmp4.y;

				H_f(i*3+2, i*4)=tmp1.z; H_f(i*3+2, i*4+1)=tmp2.z;
				H_f(i*3+2,i*4+2)=tmp3.z; H_f(i*3+2, i*4+3)=tmp4.z;

				VECTOR3_SCALE( u, b , tmp );
				VECTOR3_ADD( c.normal, tmp, tmp1);
				VECTOR3_SUB( c.normal, tmp, tmp2);	

				VECTOR3_SCALE( v, b , tmp );
				VECTOR3_ADD( c.normal, tmp, tmp3);
				VECTOR3_SUB( c.normal, tmp, tmp4);

				tmp1.normalize(); tmp2.normalize();
				tmp3.normalize(); tmp4.normalize();

				H_v(i*3, i*4)=tmp1.x; H_v(i*3, i*4+1)=tmp2.x;
				H_v(i*3,i*4+2)=tmp3.x; H_v(i*3, i*4+3)=tmp4.x;

				H_v(i*3+1, i*4)=tmp1.y; H_v(i*3+1, i*4+1)=tmp2.y;
				H_v(i*3+1,i*4+2)=tmp3.y; H_v(i*3+1, i*4+3)=tmp4.y;

				H_v(i*3+2, i*4)=tmp1.z; H_v(i*3+2, i*4+1)=tmp2.z;
				H_v(i*3+2,i*4+2)=tmp3.z; H_v(i*3+2, i*4+3)=tmp4.z;

			}

			matrixn subA, invA;
			subA.setSize(3*m, 3*m);

			for(int i=0; i<m; i++) for(int j=0; j<m; j++)
				m::assign(
						subA.range(i*3, (i+1)*3, j*3, (j+1)*3).lval(),
						m_A(mContacts_peneltyMethod.m_cti[i].index,mContacts_peneltyMethod.m_cti[j].index));


			//			m::SVinverse(subA, invA);
			m_real log_det; 
			m::LUinvert(invA, subA, log_det);
			invA*=fStep;
			invA*=-1;

			gmm::wsmatrixn temp1,temp2, temp3, temp4, H, invH, sinvA;
			gmm::assign(temp3, invA); 

			temp2.resize(H_D.nrows(),H_v.ncols());
			gmm::mult(H_D, H_v , temp2);

			temp4.resize(H_D.nrows(),H_f.ncols());
			gmm::mult(H_D, H_f , temp4);


			invA*=-1.0;
			gmm::assign(sinvA,invA);

			temp1.resize(temp3.nrows(),H_f.ncols());
			gmm::mult(temp3, H_f, temp1);	

			matrixn G,CE,CI,E0; 
			vectorn g0,ce0,ci0, _x;
			bool use_qpOASES=true;

			g0.setSize(8*m);
			ce0.setSize(3*m); 
			ci0.setSize(2*m);
			_x.setSize(8*m);

			_x.setAllValue(0.0);
			g0.setAllValue(0.0);
			ci0.setAllValue(0.0);
            ce0.setAllValue(0.0);

			for(int i=0; i<m; i++)
			{
				int index=mContacts_peneltyMethod.m_cti[i].index;		
				Physics_Contacts_Penalty::cti& c=mContacts_peneltyMethod.m_cti[i];
			    m_real b0=(c.normal.x)*(m_Velocities[index].x)+(c.normal.y)*(m_Velocities[index].y)+(c.normal.z)*(m_Velocities[index].z);
				ci0(i+m)=a0(i)+b0; 
			}


			G.setSize(8*m,8*m);
			CE.setSize(3*m,8*m);
			CI.setSize(2*m,8*m);

			//min 0.5 * x G x + g0 x
			//s.t.
			//    CE x + ce0 = 0
			//    CI x + ci0 >= 0

			// G.zero();
			/*   for(int i=0;i<8*m;i++)
				 for(int j=0;j<8*m;j++)
				 G(i,j)=0;
				 */
			G.setAllValue(0.0);
			CE.setAllValue(0.0);
			CI.setAllValue(0.0);

			for(int i=0;i<8*m;i++)
				G(i,i)=0.02;

			for(int i=0;i<4*m;i++)
			{
				G(i,i+4*m)=1.0;
				G(i+4*m,i)=1.0; 
			}
			//temp1= -h*invA*H_f

			for(int i=0;i<m;i++)
			{
				for(int j=0;j<m;j++)
				  {
			    CE(i*3,j*4)=temp1(i*3,j*4);
				CE(i*3+1,j*4)=temp1(i*3+1,j*4);
				CE(i*3+2,j*4)=temp1(i*3+2,j*4);

				CE(i*3,j*4+1)=temp1(i*3,j*4+1);
				CE(i*3+1,j*4+1)=temp1(i*3+1,j*4+1);
				CE(i*3+2,j*4+1)=temp1(i*3+2,j*4+1);

				CE(i*3,j*4+2)=temp1(i*3,j*4+2);
				CE(i*3+1,j*4+2)=temp1(i*3+1,j*4+2);
				CE(i*3+2,j*4+2)=temp1(i*3+2,j*4+2);

				CE(i*3,j*4+3)=temp1(i*3,j*4+3);
				CE(i*3+1,j*4+3)=temp1(i*3+1,j*4+3);
				CE(i*3+2,j*4+3)=temp1(i*3+2,j*4+3);

				CE(i*3,4*m+j*4)=H_v(i*3,j*4);
				CE(i*3+1,4*m+j*4)=H_v(i*3+1,j*4);
				CE(i*3+2,4*m+j*4)=H_v(i*3+2,j*4);

				CE(i*3,4*m+j*4+1)=H_v(i*3,j*4+1);
				CE(i*3+1,4*m+j*4+1)=H_v(i*3+1,j*4+1);
				CE(i*3+2,4*m+j*4+1)=H_v(i*3+2,j*4+1);

				CE(i*3,4*m+j*4+2)=H_v(i*3,j*4+2);
				CE(i*3+1,4*m+j*4+2)=H_v(i*3+1,j*4+2);
				CE(i*3+2,4*m+j*4+2)=H_v(i*3+2,j*4+2);

				CE(i*3,4*m+j*4+3)=H_v(i*3,j*4+3);
				CE(i*3+1,4*m+j*4+3)=H_v(i*3+1,j*4+3);
				CE(i*3+2,4*m+j*4+3)=H_v(i*3+2,j*4+3);
					
				//temp4=H_D*H_f			 

				CI(i, j*4)=temp4(i, j*4);
				CI(i, j*4+1)=temp4(i, j*4+1);
				CI(i, j*4+2)=temp4(i, j*4+2);
				CI(i, j*4+3)=temp4(i, j*4+3);
               
				//temp2=H_D*H_v

				CI(i+m, j*4+4*m)=temp2(i, j*4);
				CI(i+m, j*4+1+4*m)=temp2(i, j*4+1);
				CI(i+m, j*4+2+4*m)=temp2(i, j*4+2);
				CI(i+m, j*4+3+4*m)=temp2(i, j*4+3);



				  }
            	
			} 

			solve_quadprog(G,g0,CE,ce0,CI,ci0,_x,use_qpOASES);

			// USE_FORCE

			// A dv= h F where F is contact forces
			// dv = h invA F
			//	  = h invA H_f lambda

			gmm::assign(sinvA, invA);
			std::vector<m_real> dv,Fc, lambda;
			dv.resize(3*n);
			Fc.resize(3*n);

			for(int i=0; i<n; i++)
			{

				int index=mContacts_peneltyMethod.m_cti[i].index;

				Fc[3*i]=H_f(i*3,i*4)*_x(4*i)+H_f(i*3,i*4+1)*_x(4*i+1)+H_f(i*3,i*4+2)*_x(4*i+2)+H_f(i*3,i*4+3)*_x(4*i+3);	          
				Fc[3*i+1]=H_f(i*3+1,i*4)*_x(4*i)+H_f(i*3+1,i*4+1)*_x(4*i+1)+H_f(i*3+1,i*4+2)*_x(4*i+2)+H_f(i*3+1,i*4+3)*_x(4*i+3);	          
				Fc[3*i+2]=H_f(i*3+2,i*4)*_x(4*i)+H_f(i*3+2,i*4+1)*_x(4*i+1)+H_f(i*3+2,i*4+2)*_x(4*i+2)+H_f(i*3+2,i*4+3)*_x(4*i+3);	        

				printf("l1=%f ,l2=%f ,l3=%f  ,l4=%f \n",_x[i*4],_x[i*4+1],_x[i*4+2],_x[i*4+3]);
			}




			lambda.resize(4*m);
			for(int i=0; i<lambda.size(); i++)
				lambda[i]=_x[i];

			temp1.resize(sinvA.nrows(), H_f.ncols());
			gmm::mult(sinvA, H_f, temp1); 
			gmm::mult(temp1, lambda, dv);

			printf("fStep: %f\n",fStep);

			for(int i=0; i<n; i++)
			{
				int index=mContacts_peneltyMethod.m_cti[i].index;
				Physics_Contacts_Penalty::cti& c=mContacts_peneltyMethod.m_cti[i];
				m_Velocities[index]+=vector3(dv[i*3], dv[i*3+1], dv[i*3+2]);
				printf("p1=%f ,p2=%f ,p3=%f  ,p4=%f \n",_x[4*m+i*4],_x[4*m+i*4+1],_x[4*m+i*4+2],_x[4*m+i*4+3]);
        		vector3 U=vector3(dv[i*3]*fStep, dv[i*3+1]*fStep, dv[i*3+2]*fStep);
				m_y[index]=U;
			}

			printf("Use_Force & frictionless\n");

			mContacts_peneltyMethod.m_cti.clear();


		}
		else if(m_cfg.contactMethod==Config::LCP_METHOD &&mContacts_peneltyMethod.m_cti.size())
		{			
			// n: number of contact points
			int n=mContacts_peneltyMethod.m_cti.size();

			// m: the number of particles involved in contact.  at the moment,
			// m==n because we only consider FV collisions ignoring VF and EE
			// collisions.
			int m=n;	
            //
			int k=m_iParticles-n;

			printf("k=%d,m_iParticles=%d \n ",k,m_iParticles);
			printf("m_ParticleMass=%f",m_ParticleMass);
		
			// following matrix naming conventions follow [DAK04] and [DDKA06].
			// [DAK04] C.Duriez, C. Andriot, and A.Kheddar, A multi-threaded
			// approach for deformable/rigid contacts with haptic feedback,
			// IEEE haptic symposium, 2004.

			// [DDKA06] Realistic Haptic Rendering of Interacting Deformable
			// Objects in Virtual Environments

#ifdef DEBUG_DRAW
		    int l=0;  // l is a count number of frictionless point
#endif
		
			intvectorn basesStartIndex(n+1);
			std::vector<vector3N> bases;
			bases.resize(n);
			{
				int Start=0;
				for(int i=0; i<n; i++)
				{
					int index=mContacts_peneltyMethod.m_cti[i].index;		
					m_real coeff=m_cfg.m_dynamicFrictionCoef[index];
					Physics_Contacts_Penalty::cti& c=mContacts_peneltyMethod.m_cti[i];
					basesStartIndex[i]=Start;
					if(coeff==0)
					{
						Start++;
						bases[i].resize(1);
						bases[i][0]=c.normal;
#ifdef DEBUG_DRAW
						l++;
#endif
					}
					else
					{
						// 계산은 여기서 한번에 하고, 아래서는 이걸 갖다 쓰면, 코드 길이1/4 수준으로 줄일 수 있음.
						vector3 u, v , tmp;
						u.cross( c.normal, vector3(0,0,1));
						u.normalize();

						v.cross( c.normal, u);
						v.normalize();	
						Start+=4;
						bases[i].resize(4);
						bases[i][0]=c.normal+u*1.0/coeff;
						bases[i][1]=c.normal-u*1.0/coeff;
						bases[i][2]=c.normal+v*1.0/coeff;
						bases[i][3]=c.normal-v*1.0/coeff;

						bases[i][0].normalize();
						bases[i][1].normalize();
						bases[i][2].normalize();
						bases[i][3].normalize();

					}
				}
				basesStartIndex[n]=Start;
			}

			// 2014.04.01 question: gmm::wsmatrixn -> matrixn 타입으로 변환
			matrixn  H_f, C_N;

			int numBases=basesStartIndex[m]; 
			int numDims=3*m;
#ifdef DEBUG_DRAW
		    int friPoint=n-l;
	        printf("4m-3l=%d, numbases=%d\n",4*m-3*l,numBases);
#endif
			H_f.setSize(numDims,numBases);
			C_N.setSize(2*numBases, numBases+numDims);
            
			H_f.setAllValue(0.0);
			C_N.setAllValue(0.0);

			vector3  u, v , tmp; 
			vector3  tmp1, tmp2, tmp3, tmp4;
		   

			// friction cone's coefficient & velocity cone's coefficient
			// a=tan(angle), b=tan(pi-angle)=cot(angle)=1/tan(angle)=1/a
            /*
			m_real a,b;
			a=(m_real) 1.0; 
			b=(m_real)1.732;		
            */
			vectorn a0, b0;
			a0.setSize(m);  // -c.penetratingDepth/fStep;
			b0.setSize(numBases);// vfree*bases 

    		for(int i=0; i<n; i++)
			{
             	Physics_Contacts_Penalty::cti& c=mContacts_peneltyMethod.m_cti[i];
				int index=mContacts_peneltyMethod.m_cti[i].index;	
				//a0(i)=(m_real)c.penetratingDepth/fStep; 
				a0(i)=0; // this works better for now.
                //printf("bases[%d].size()=%d\n",i,bases[i].size());

				for(int j=0; j<bases[i].size(); j++)
				{
                  b0(basesStartIndex[i]+j)=VECTOR3_DP(bases[i][j], m_Velocities[index]);
			      if(bases[i].size()==1)
				  b0(basesStartIndex[i]+j)+=a0(i); 

				  H_f(i*3, basesStartIndex[i]+j)=bases[i][j].x;
				  H_f(i*3+1, basesStartIndex[i]+j)=bases[i][j].y;
				  H_f(i*3+2, basesStartIndex[i]+j)=bases[i][j].z;
                 
				  C_N(basesStartIndex[i]+numBases+j,i*3+numBases)=bases[i][j].x;
				  C_N(basesStartIndex[i]+numBases+j,i*3+1+numBases)=bases[i][j].y;
				  C_N(basesStartIndex[i]+numBases+j,i*3+2+numBases)=bases[i][j].z;

				  for(int k=0; k<bases[i].size(); k++) 
				  C_N(basesStartIndex[i]+j,basesStartIndex[i]+k)=VECTOR3_DP(bases[i][j],bases[i][k]);  
				}
			}

			matrixn subA, hH_f;
			subA.setSize(numDims, numDims);
            hH_f.setSize(numDims, numBases);

			
			for(int i=0; i<m; i++) for(int j=0; j<m; j++)
				m::assign(
						subA.range(i*3, (i+1)*3, j*3, (j+1)*3).lval(),
						m_A(mContacts_peneltyMethod.m_cti[i].index,mContacts_peneltyMethod.m_cti[j].index));
 
			hH_f=H_f*fStep;
  			subA*=-1;
				
			matrixn G,CE,CI,E0; 
			vectorn g0,ce0,ci0, _x;
			bool use_qpOASES=true;

			g0.setSize(numBases+numDims);
			ce0.setSize(numDims); 
			ci0.setSize(2*numBases);
			_x.setSize(numBases+numDims);

			_x.setAllValue(0.0);
			g0.setAllValue(0.0);
			ci0.setAllValue(0.0);
            ce0.setAllValue(0.0);

			for(int i=0; i<numBases; i++)
				ci0(i+numBases)=b0(i); 

			// x: [lambda dv]'
			// CE: [ hH_f  subA ]=0
			CE.setSize(numDims, numBases+numDims);
			CI.setSize(2*numBases, numBases+numDims);

			//min 0.5 * x G x + g0 x
			//s.t.
			//    CE x + ce0 = 0
			//    CI x + ci0 >= 0

			// G.zero();
			G.identity(numBases+numDims);
			CE.setAllValue(0.0);

			CE.range(0,CE.rows(), 0,hH_f.cols()).assign(hH_f);
			CE.range(0,CE.rows(), hH_f.cols(),CE.cols()).assign(subA);
			CI=C_N;


			solve_quadprog(G,g0,CE,ce0,CI,ci0,_x,use_qpOASES);

			// USE_FORCE

			// A dv= h F where F is contact forces
			// dv = h invA F
			//	  = h invA H_f lambda

			vectorn Fc, lambda;
			std::vector<vector3N> dv;

			Fc.setSize(numDims);
			lambda.setSize(numBases);
			dv.resize(n);


			for(int i=0;i<numBases; i++)
			{
				lambda(i)=_x[i];
				printf("lambda[%d]=%f\n",i,_x[i]);
			}

			for(int i=0; i<n; i++)
			{
				dv[i].resize(1);
				dv[i][0]=vector3(_x[i*3+numBases],_x[i*3+1+numBases],_x[i*3+2+numBases]);
				printf("dv[%d].x=%f , dv[%d].y=%f , dv[%d].z=%f \n",i,dv[i][0].x,i,dv[i][0].y,i,dv[i][0].z);
			}

			Fc.column().mult(H_f,lambda.column());

#ifdef DEBUG_DRAW
			lines_fvelocity.resize(2*friPoint,3);
			lines_fforce.resize(2*friPoint,3);

			lines_flvelocity.resize(2*l,3);
			lines_flforce.resize(2*l,3);

			lines_normal.resize(2*n,3);		

			int count1=0;
			int count2=0;
#endif

			for(int i=0; i<n; i++)
			{

				int index=mContacts_peneltyMethod.m_cti[i].index; 
				m_real coeff=m_cfg.m_dynamicFrictionCoef[index];

				vector3 dpf=vector3(Fc[3*i],Fc[3*i+1],Fc[3*i+2]);
				VECTOR3_SCALE( dpf, 0.01 , dpf );
				VECTOR3_ADD(dpf, m_Positions[index], dpf);

#ifdef DEBUG_DRAW
				if(coeff==0)
				{

					lines_flforce.row(count1*2).setVec3(0, m_Positions[index]);
					lines_flforce.row(count1*2+1).setVec3(0, dpf);

					count1++;
				}

				else
				{	

					lines_fforce.row(count2*2).setVec3(0, m_Positions[index]);
					lines_fforce.row(count2*2+1).setVec3(0, dpf);

					count2++;
				}

				printf("Fc[%d].x=%f , Fc[%d].y=%f , Fc[%d].z=%f \n",i,Fc[i*3],i,Fc[i*3+1],i,Fc[i*3+2]);
#endif
			}

#ifdef DEBUG_DRAW
			count1=0;
			count2=0;
#endif
			for(int i=0; i<n; i++)
			{
				int index=mContacts_peneltyMethod.m_cti[i].index;
				m_real coeff=m_cfg.m_dynamicFrictionCoef[index];
				Physics_Contacts_Penalty::cti& c=mContacts_peneltyMethod.m_cti[i];
				m_Velocities[index]+=dv[i][0];

				vector3 dpdv=m_Velocities[index];
				vector3 dpn=c.normal;

				VECTOR3_SCALE( dpdv, 1 , dpdv );
				VECTOR3_ADD(dpdv, m_Positions[index], dpdv);

				VECTOR3_SCALE( dpn, 20 , dpn );
				VECTOR3_ADD(dpn, m_Positions[index], dpn);

#ifdef DEBUG_DRAW
				if(coeff==0)
				{

					lines_flvelocity.row(count1*2).setVec3(0, m_Positions[index]);
					lines_flvelocity.row(count1*2+1).setVec3(0, dpdv);

					count1++;
				}

				else
				{
					lines_fvelocity.row(count2*2).setVec3(0, m_Positions[index]);
					lines_fvelocity.row(count2*2+1).setVec3(0, dpdv);

					count2++;
				}

				lines_normal.row(i*2).setVec3(0, m_Positions[index]);
				lines_normal.row(i*2+1).setVec3(0, dpn);
#endif
				vector3 U;
				VECTOR3_SCALE( dv[i][0] , fStep , U );
				m_y[index]=U;
			}


#ifdef DEBUG_DRAW
			g_debugObject->registerObject("friction_contactForces", "LineList", "solidred", lines_fforce);
			g_debugObject->registerObject("friction_deltaVelocities", "LineList", "solidblue", lines_fvelocity);

			g_debugObject->registerObject("frictionless_contactForces", "LineList", "solidwhite", lines_flforce);
			g_debugObject->registerObject("frictionless_deltaVelocities", "LineList", "solidblack", lines_flvelocity);

			g_debugObject->registerObject("normalForces", "LineList", "solidgreen", lines_normal);

			printf("Use_Force & friction\n");
#endif
			mContacts_peneltyMethod.m_cti.clear();

		}



     	//
		// Compute b = h * ( f(0) + h * df/dx * v(0) + df/dx * y )
		//
		m_Velocities.Scale( fStep, m_vTemp2 );
		m_vTemp2.Add( m_y, m_vTemp2 );
		m_TotalForces_dp.PostMultiply( m_vTemp2, m_vTemp1 );
		//		m_vTemp1.Scale( fStep, m_vTemp2 );
		m_TotalForces_int.Add( m_vTemp1, m_vTemp1 );
		m_vTemp1.Scale( fStep, m_b );

		const bool useModifiedConjugateGradient_BW98=true;

		if(useModifiedConjugateGradient_BW98)
		{
			//
			// Setup the inverse of preconditioner -- We use a vector for memory efficiency.  
			// Technically it's the diagonal of a matrix
			//
			for( i=0; i<m_iParticles; i++ )
			{
				const matrix3& mm=m_A(i,i);
				m_PInv[i].x = (m_real)mm._11;
				m_PInv[i].y = (m_real)mm._22;
				m_PInv[i].z = (m_real)mm._33;
			}
			m_PInv.Invert( m_P );

			//
			// Modified Preconditioned Conjugate Gradient method
			//

			m_dv = m_z;

			// delta_0 = DotProduct( filter( b ), P * filter( b ) );
			m_b.ElementMultiply( m_S, m_vTemp1 );
			m_P.ElementMultiply( m_vTemp1, m_vTemp2 );
			Delta_0 = m_vTemp2.DotProduct( m_vTemp1 );
			if( Delta_0 < 0 )
			{
				m_b.Dump( "b:\r\n" );
				m_P.Dump( "P:\r\n" );
				Msg::print("%s\n", "Negative Delta_0 most likely caused by a non-Positive Definite matrix\r\n" );
			}

			// r = filter( b - A * dv )
			m_A.PostMultiply( m_dv, m_vTemp1 );
			m_b.Subtract( m_vTemp1, m_vTemp2 );
			m_vTemp2.ElementMultiply( m_S, m_r );

			// c = filter( Pinv * r )
			m_PInv.ElementMultiply( m_r, m_vTemp1 );
			m_vTemp1.ElementMultiply( m_S, m_c );

			Delta_new = m_r.DotProduct( m_c );

			if( Delta_new < Eps_Sq * Delta_0 )
			{
				m_b.Dump( "b: \r\n" );
				m_P.Dump( "P: \r\n" );
				Msg::print("%s\n", "This isn't good!  Probably a non-Positive Definite matrix\r\n" );
			}

			while( (Delta_new > Eps_Sq * Delta_0) && (iIterations < iMaxIterations) )
			{
				m_A.PostMultiply( m_c, m_vTemp1 );

				m_vTemp1.ElementMultiply( m_S, m_q );

				alpha = Delta_new / (m_c.DotProduct( m_q ) );
				m_c.Scale( alpha, m_vTemp1 );
				m_dv.Add( m_vTemp1, m_dv );

				m_q.Scale( alpha, m_vTemp1 );
				m_r.Subtract( m_vTemp1, m_r );

				m_PInv.ElementMultiply( m_r, m_s );
				Delta_old = Delta_new;
				Delta_new = m_r.DotProduct( m_s );

				m_c.Scale( Delta_new / Delta_old, m_vTemp1 );
				m_s.Add( m_vTemp1, m_vTemp2 );
				m_vTemp2.ElementMultiply( m_S, m_c );

				iIterations++;
			}
		}
		else
		{
#ifndef USE_GMM
			ASSERT(0);
#else
			static gmm::wsmatrixn A;
			static vectorn b, x;
			gmm::resize(A, m_A.rows()*3, m_A.cols()*3);

			for(int i=0; i<m_A.rows(); i++)
			{
				gmm::linalg_traits<Physics_SparseSymmetricMatrix::svector_mat3>::const_iterator it = gmm::vect_const_begin(m_A.m_data[i]);
				gmm::linalg_traits<Physics_SparseSymmetricMatrix::svector_mat3>::const_iterator ite = gmm::vect_const_end(m_A.m_data[i]);

				for(;it!=ite; ++it)
				{
					int j=it.index();
					matrix3 const& m=*it;


					A[i*3+0][j*3+0]=m._11;
					A[i*3+0][j*3+1]=m._12;
					A[i*3+0][j*3+2]=m._13;
					A[i*3+1][j*3+0]=m._21;
					A[i*3+1][j*3+1]=m._22;
					A[i*3+1][j*3+2]=m._23;
					A[i*3+2][j*3+0]=m._31;
					A[i*3+2][j*3+1]=m._32;
					A[i*3+2][j*3+2]=m._33;
				}
			}

			b.setSize(m_b.size()*3);
			for(int i=0; i<m_b.size(); i++)
			{
				b[i*3+0]=m_b[i].x;
				b[i*3+1]=m_b[i].y;
				b[i*3+2]=m_b[i].z;
			}

			sm::UMFsolve(A,b,x);

			ASSERT(x.size()==m_b.size()*3);
			for(int i=0; i<m_b.size(); i++)
			{
				m_dv[i].x=x[i*3];
				m_dv[i].y=x[i*3+1];
				m_dv[i].z=x[i*3+2];
			}


#endif

		}

		m_Velocities.Add( m_dv, m_Velocities );
		m_Velocities.Scale( fStep, m_vTemp1 );		
		m_Positions.Add( m_vTemp1, m_Positions );
        m_Positions.Add( m_y, m_Positions );



		fTotal += (m_real)fabs( fStep );
	}
}

void Physics_ParticleSystem::Update_SemiImplicit( float fTime )
{
	m_real fTotal = 0.0f, fStep = (m_real)fTime;
	vector3 vCOG, dTorque, tmp, tmp2;
	int i;

	while( fTotal < fTime )
	{
		//
		// Calculate the center of gravity
		//
		vCOG.x = vCOG.y = vCOG.z = 0;
		for( i=0; i<m_iParticles; i++ )
			VECTOR3_ADD( vCOG, m_Positions[i], vCOG );
		vCOG.x /= m_iParticles;
		vCOG.y /= m_iParticles;
		vCOG.z /= m_iParticles;

		dTorque.x = dTorque.y = dTorque.z = 0.0f;

		//
		// Update the W matrix if necessary
		//
		if( fStep != m_LastStep )
		{
			m_W.identity();
			m_H.Scale( fStep * fStep / m_LastStep / m_LastStep, m_H );
			m_W.Subtract( m_H, m_W );
			m_W.Invert();
			m_LastStep = fStep;
		}

		// 
		// Zero out everything
		//
		m_TotalForces_int.zero();
		m_TotalForces_ext.zero();


		//
		// Apply the forces
		//
		for(int i=0, ni=m_Forces.size(); i<ni; i++)
		{
			m_Forces[i]->Apply(fStep, m_Masses, false, m_Positions, m_Velocities,
					m_TotalForces_int, m_TotalForces_ext,
					m_TotalForces_dp, m_TotalForces_dv );
		}

		//
		// Filter the internal forces
		//
		m_W.PreMultiply( m_TotalForces_int, m_vTemp1 );

		//
		// Update the torque
		//
		for( i=0; i<m_iParticles; i++ )
		{
			tmp .cross( m_vTemp1[i], m_Positions[i] );
			VECTOR3_ADD( dTorque, tmp, dTorque );
		}


		//
		// Compute the new velocities and positions
		//
		m_vTemp1.Add( m_TotalForces_ext, m_dv );
		m_dv.Scale( fStep, m_dv );
		m_dv.ElementMultiply( m_MassInv, m_dv );
		m_Velocities.Add( m_dv, m_Velocities );



		m_Velocities.Scale( fStep, m_vTemp1 );
		m_Positions.Add( m_vTemp1, m_PosTemp );

		/*//
		// Keep us above the floor -- cheesey collision detection
		//
		for( i=0; i<m_iParticles; i++ )
		if( m_PosTemp[i].y < FLOOR_Y )
		{
		m_PosTemp[i].y = FLOOR_Y;
		m_Velocities[i].y = 0;
		}
		*/
		//
		// Post correct for angular momentum
		//

		for( i=0; i<m_iParticles; i++ )
		{
			if( m_Masses[i].x )
			{
				tmp2 .sub( vCOG, m_Positions[i] );
				tmp.cross( tmp2, dTorque);
				VECTOR3_SCALE( tmp, fStep * fStep * fStep * m_MassInv[i].x, tmp );
				VECTOR3_ADD( m_PosTemp[i], tmp, m_PosTemp[i] );
				/*				if( m_PosTemp[i].y < FLOOR_Y )
								{
								m_PosTemp[i].y = FLOOR_Y;
								m_Velocities[i].y = 0;
								}*/
			}
		}

		//
		// Apply inverse dynamics to prevent excessive stretch
		// We need 10 or so iterations to be stable -- with more particles in the
		// mesh, we need more iterations.
		//
		for( i=0; i<m_iInverseIterations; i++ )
		{
			for(int i=0, ni=m_Forces.size(); i<ni; i++)	
			{
				m_Forces[i]->Fixup( m_Masses, m_PosTemp );
			}
		}
		//
		// Update velocityp and position
		//
		/*
		   for( i=0; i<m_iParticles; i++ )
		   if( m_PosTemp[i].y < FLOOR_Y )
		   {
		   m_PosTemp[i].y = FLOOR_Y;
		   m_Velocities[i].y = 0;
		   }*/

		m_PosTemp.Subtract( m_Positions, m_vTemp1 );
		m_vTemp1.Scale( (m_real)1.0 / fStep, m_Velocities );
		m_Positions = m_PosTemp;

		fTotal += (m_real)fabs( fStep );
	}
}

void Physics_ParticleSystem::AddConstraint( Physics_Constraint &Constraint )
{
	m_Constraints.push_back( &Constraint );

	//
	// Apply the constraints
	//
	for( int i=0; i<m_iParticles; i++ )
	{
		m_S[i].identity();
	}
	m_z.zero();


	std::list<Physics_Constraint*>::iterator i;

	for( i=m_Constraints.begin(); i!=m_Constraints.end(); ++i)
	{
		(*i)->Apply(m_S, m_z);
	}
}

bool Physics_ParticleSystem::DeleteConstraint( Physics_Constraint &Constraint )
{

	m_Constraints.remove(&Constraint );


	{
		//
		// Apply the constraints
		//
		for( int i=0; i<m_iParticles; i++ )
		{
			m_S[i].identity();
		}
		m_z.zero();


		std::list<Physics_Constraint*>::iterator i;

		for( i=m_Constraints.begin(); i!=m_Constraints.end(); ++i)
		{
			(*i)->Apply(m_S, m_z);			
		}
	}
	return true;
}

void Physics_ParticleSystem::Reset()
{
	//
	// Update the constraints for the Implicit integration scheme
	//
	if( m_iIntegrationMethod == INTEGRATE_IMPLICIT )
	{
		for( int i=0; i<m_iParticles; i++ )
		{
			m_S[i].identity();
		}
		m_z.zero();
		m_y.zero();

		std::list<Physics_Constraint*>::iterator i;

		for( i=m_Constraints.begin(); i!=m_Constraints.end(); ++i)
		{
			(*i)->Apply(m_S, m_z);
		}
	}
}
