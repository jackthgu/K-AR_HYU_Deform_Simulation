// Physics_Physics_ParticleSystem.cpp: implementation of the Physics_Physics_ParticleSystem class.
//
//////////////////////////////////////////////////////////////////////

#include "DRGShell.h"
#include "Physics_Impulses.h"
#include "../gmm.h"
#include "BaseLib/math/Operator.h"
#include "BaseLib/math/Operator_NR.h"
#include "PhysicsLib/sDIMS/lcp.h"
//#include "../../../QP_controller/quadprog.h"
#include "MainLib/OgreFltk/objectList.h"

//#define PROFILE
#ifdef PROFILE
#include "BaseLib/utility/QPerformanceTimer.h"
#else
#define BEGIN_TIMER(x) 
#define END_TIMER2(x) 
#endif

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
	mContacts(iParticles)
	//m_vContactForce(iParticles)
{
	//m_iIntegrationMethod = INTEGRATE_SEMI_IMPLICIT;
	m_iIntegrationMethod = INTEGRATE_IMPLICIT;

	m_cfg.m_dynamicFrictionCoef.resize(iParticles);
	m_cfg.m_dynamicFrictionCoef.setAllValue(0.0);
	m_cfg.DFthresholdVel=0.1;
	m_cfg.contactMethod=Config::LCP_METHOD;
	m_cfg.penaltyStiffness=1000.0;
	m_cfg.penaltyDampness=100.0;
	m_cfg.penaltyMuScale=1.0;
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


	//m_vContactForce.zero();
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

void Physics_ParticleSystem::UpdateDV_Implicit( float fTime )
{
	UpdateDV_Implicit_part1( fTime );
	UpdateDV_Implicit_part2( fTime );
}

void Physics_ParticleSystem::UpdateDV_Implicit_part1( float fTime )
{
	BEGIN_TIMER(updatedv_part1);
	int i,iIterations = 0, iMaxIterations = (int)sqrt(double(m_iParticles))+3; m_real
		fTotal = (m_real)0.0f, fStep = (m_real)fTime; m_real alpha, Delta_0,
			   Delta_old, Delta_new; m_real Eps_Sq = (m_real)1e-22;

	{
		// 
		// Zero out everything except ext forces
		//
		m_TotalForces_int.zero(); 
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

		m_MxTemp1.mult(m_TotalForces_dp, fStep);
		m_MxTemp2.add(m_MxTemp1,m_TotalForces_dv);
		m_MxTemp1.mult(m_MxTemp2, fStep);
		m_A .sub(m_MxMasses,m_MxTemp1);

     	//
		// Compute b = h * ( f(0) + h * df/dx * v(0) + df/dx * y )
		//
		m_Velocities.Scale( fStep, m_vTemp2 );
		m_vTemp2.Add( m_y, m_vTemp2 );
		m_TotalForces_dp.PostMultiply( m_vTemp2, m_vTemp1 );
	}
#ifdef USE_GMM
		const bool useModifiedConjugateGradient_BW98=false;
#else
		const bool useModifiedConjugateGradient_BW98=true;
#endif

		if(useModifiedConjugateGradient_BW98)
		{
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
		}
	END_TIMER2(updatedv_part1);
}
// A*dv=b
// A*dv2=b+alpha
// let dv2=dv+delta,
// then A*delta=alpha

void Physics_ParticleSystem::UpdateDV_Implicit_part2( float fTime )
{
	BEGIN_TIMER(updatedv_part2);
	int i,iIterations = 0, iMaxIterations = (int)sqrt(double(m_iParticles))+3; m_real
		fTotal = (m_real)0.0f, fStep = (m_real)fTime; m_real alpha, Delta_0,
			   Delta_old, Delta_new; m_real Eps_Sq = (m_real)1e-22;
	{
		m_TotalForces_int+=m_TotalForces_ext;
		m_TotalForces_int.Add( m_vTemp1, m_vTemp1 );
		m_vTemp1.Scale( fStep, m_b );

#ifdef USE_GMM
		const bool useModifiedConjugateGradient_BW98=false;
#else
		const bool useModifiedConjugateGradient_BW98=true;
#endif

		if(useModifiedConjugateGradient_BW98)
		{
			// solve for m_dv
			// 				where m_A * m_dv = m_b 
			//
			//

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
			ASSERT(0);
		}
	}
	END_TIMER2(updatedv_part2);
}
void Physics_ParticleSystem::Update_Implicit( float fTime )
{ 
	BEGIN_TIMER(update_implicit);
	int i,iIterations = 0, iMaxIterations = (int)sqrt(double(m_iParticles))+3; m_real
		fTotal = (m_real)0.0f, fStep = (m_real)fTime; m_real alpha, Delta_0,
			   Delta_old, Delta_new; m_real Eps_Sq = (m_real)1e-22;

	while( fTotal < fTime )
	{
		// 
		// Zero out everything 
		//
		m_TotalForces_int.zero(); //m_TotalForces_ext.zero();
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

		if(m_cfg.contactMethod==Config::PENALTY_METHOD && mContacts_peneltyMethod.m_cti.size())
		{
			// n: number of contact points
			int n=mContacts_peneltyMethod.m_cti.size();
			for(int i=0; i<n; i++)
			{
				Physics_Contacts_Penalty::cti& c=mContacts_peneltyMethod.m_cti[i];
				int index=mContacts_peneltyMethod.m_cti[i].index;		
				m_real depth=(m_real)c.penetratingDepth*-1;
				vector3 relvel=m_Velocities[index];
				double vn=relvel%c.normal*-1;
				double k=m_cfg.penaltyStiffness;
				double d=m_cfg.penaltyDampness;
				double f=depth*k;
				double _depthMax=1;
				//printf("d %f %f\n", depth, vn);
				double f2=vn*d*sop::clampMap(depth, 0, _depthMax);
				f=f+f2;
				if(f<0) f=0;
				vector3 normalForce= c.normal*f;
				// see void DynamicsSimulator::_calcContactForce(CollisionSequence& corbaCollisionSequence)
				//
				vector3 fv=relvel+c.normal*vn;
				double tiny = 1e-8;
				double mu=m_cfg.m_dynamicFrictionCoef[index];
				mu*=m_cfg.penaltyMuScale;
				::vector3 force=normalForce;
				if(fv.length()>tiny)	// slip friction force
				{
					// calc frictionForce
					::vector3 dir;
					dir.normalize(fv);

					force-=dir*f*mu;
				}
				else
				{
					//printf("static friction needed\n");
				}
				m_TotalForces_ext[index] +=force;

			}
			mContacts_peneltyMethod.m_cti.clear();
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



#ifdef DEBUG_DRAW
		static ObjectList* g_debugObject=NULL;

		if (g_debugObject==NULL)
			g_debugObject=new ObjectList();

		matrixn lines(0, 3);
		matrixn lines_other(0, 3);
	
		g_debugObject->registerObject("contactForces", "LineList", "solidred", lines);
		g_debugObject->registerObject("deltaVelocities", "LineList", "solidblue", lines_other);
#endif


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

				VECTOR3_SCALE( u, a , tmp );
				VECTOR3_ADD( c.normal, tmp, tmp1);
				VECTOR3_SUB( c.normal, tmp, tmp2);	

				VECTOR3_SCALE( v, a , tmp );
				VECTOR3_ADD( c.normal, tmp, tmp3);
				VECTOR3_SUB( c.normal, tmp, tmp4);

				tmp1.normalize(); tmp2.normalize(); 
				tmp3.normalize(); tmp4.normalize();

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

			ASSERT(false);
			//solve_quadprog(G,g0,CE,ce0,CI,ci0,_x,use_qpOASES);

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

			// following matrix naming conventions follow [DAK04] and [DDKA06].
			// [DAK04] C.Duriez, C. Andriot, and A.Kheddar, A multi-threaded
			// approach for deformable/rigid contacts with haptic feedback,
			// IEEE haptic symposium, 2004.

			// [DDKA06] Realistic Haptic Rendering of Interacting Deformable
			// Objects in Virtual Environments
			gmm::wsmatrixn H_v,H_f, H_D, C_N;

			H_v.resize(3*n,4*m);
			H_f.resize(3*n,4*m);
			H_D.resize(m, 3*m);
			C_N.resize(8*m, 8*m);

			
			vector3  u, v, tmp; 
			vector3  tmp1, tmp2, tmp3, tmp4; m_real a,b;
			a=(m_real) 1.0; b=(m_real) 1.0;	
	

			vectorn a0, b0;
			a0.setSize(m);
			b0.setSize(4*m);

			for(int i=0; i<n; i++)
			{
				Physics_Contacts_Penalty::cti& c=mContacts_peneltyMethod.m_cti[i];
				int index=mContacts_peneltyMethod.m_cti[i].index;		
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

				VECTOR3_SCALE( u, a , tmp );
				VECTOR3_ADD( c.normal, tmp, tmp1);
				VECTOR3_SUB( c.normal, tmp, tmp2);	

				VECTOR3_SCALE( v, a , tmp );
				VECTOR3_ADD( c.normal, tmp, tmp3);
				VECTOR3_SUB( c.normal, tmp, tmp4);
				/*
								tmp1.normalize(); tmp2.normalize(); 
								tmp3.normalize(); tmp4.normalize();
				*/
				H_f(i*3, i*4)=tmp1.x; H_f(i*3, i*4+1)=tmp2.x;
				H_f(i*3, i*4+2)=tmp3.x; H_f(i*3, i*4+3)=tmp4.x;

				H_f(i*3+1, i*4)=tmp1.y; H_f(i*3+1, i*4+1)=tmp2.y;
				H_f(i*3+1,i*4+2)=tmp3.y; H_f(i*3+1, i*4+3)=tmp4.y;

				H_f(i*3+2, i*4)=tmp1.z; H_f(i*3+2, i*4+1)=tmp2.z;
				H_f(i*3+2,i*4+2)=tmp3.z; H_f(i*3+2, i*4+3)=tmp4.z;

               C_N(i*4,i*4)=VECTOR3_DP(tmp1, tmp1);
		  	   C_N(i*4,i*4+1)=VECTOR3_DP(tmp1, tmp2);
		  	   C_N(i*4,i*4+2)=VECTOR3_DP(tmp1, tmp3);
		  	   C_N(i*4,i*4+3)=VECTOR3_DP(tmp1, tmp4);
 
               C_N(i*4+1,i*4)=VECTOR3_DP(tmp2, tmp1);
		  	   C_N(i*4+1,i*4+1)=VECTOR3_DP(tmp2, tmp2);
		  	   C_N(i*4+1,i*4+2)=VECTOR3_DP(tmp2, tmp3);
		  	   C_N(i*4+1,i*4+3)=VECTOR3_DP(tmp2, tmp4);
 
   
               C_N(i*4+2,i*4)=VECTOR3_DP(tmp3, tmp1);
		  	   C_N(i*4+2,i*4+1)=VECTOR3_DP(tmp3, tmp2);
		  	   C_N(i*4+2,i*4+2)=VECTOR3_DP(tmp3, tmp3);
		  	   C_N(i*4+2,i*4+3)=VECTOR3_DP(tmp3, tmp4);
 
               C_N(i*4+3,i*4)=VECTOR3_DP(tmp4, tmp1);
		  	   C_N(i*4+3,i*4+1)=VECTOR3_DP(tmp4, tmp2);
		  	   C_N(i*4+3,i*4+2)=VECTOR3_DP(tmp4, tmp3);
		  	   C_N(i*4+3,i*4+3)=VECTOR3_DP(tmp4, tmp4);
 
				VECTOR3_SCALE( u, b , tmp );
				VECTOR3_ADD( c.normal, tmp, tmp1);
				VECTOR3_SUB( c.normal, tmp, tmp2);	

				VECTOR3_SCALE( v, b , tmp );
				VECTOR3_ADD( c.normal, tmp, tmp3);
				VECTOR3_SUB( c.normal, tmp, tmp4);
				H_v(i*3, i*4)=tmp1.x; H_v(i*3, i*4+1)=tmp2.x;
				H_v(i*3,i*4+2)=tmp3.x; H_v(i*3, i*4+3)=tmp4.x;

				H_v(i*3+1, i*4)=tmp1.y; H_v(i*3+1, i*4+1)=tmp2.y;
				H_v(i*3+1,i*4+2)=tmp3.y; H_v(i*3+1, i*4+3)=tmp4.y;

				H_v(i*3+2, i*4)=tmp1.z; H_v(i*3+2, i*4+1)=tmp2.z;
				H_v(i*3+2,i*4+2)=tmp3.z; H_v(i*3+2, i*4+3)=tmp4.z;

			   
               C_N(i*4+4*m,i*4+4*m)=VECTOR3_DP(tmp1, tmp1);
		  	   C_N(i*4+4*m,i*4+1+4*m)=VECTOR3_DP(tmp1, tmp2);
		  	   C_N(i*4+4*m,i*4+2+4*m)=VECTOR3_DP(tmp1, tmp3);
		  	   C_N(i*4+4*m,i*4+3+4*m)=VECTOR3_DP(tmp1, tmp4);
 
               C_N(i*4+1+4*m,i*4+4*m)=VECTOR3_DP(tmp2, tmp1);
		  	   C_N(i*4+1+4*m,i*4+1+4*m)=VECTOR3_DP(tmp2, tmp2);
		  	   C_N(i*4+1+4*m,i*4+2+4*m)=VECTOR3_DP(tmp2, tmp3);
		  	   C_N(i*4+1+4*m,i*4+3+4*m)=VECTOR3_DP(tmp2, tmp4);
 
               C_N(i*4+2+4*m,i*4+4*m)=VECTOR3_DP(tmp3, tmp1);
		  	   C_N(i*4+2+4*m,i*4+1+4*m)=VECTOR3_DP(tmp3, tmp2);
		  	   C_N(i*4+2+4*m,i*4+2+4*m)=VECTOR3_DP(tmp3, tmp3);
		  	   C_N(i*4+2+4*m,i*4+3+4*m)=VECTOR3_DP(tmp3, tmp4);
 
               C_N(i*4+3+4*m,i*4+4*m)=VECTOR3_DP(tmp4, tmp1);
		  	   C_N(i*4+3+4*m,i*4+1+4*m)=VECTOR3_DP(tmp4, tmp2);
		  	   C_N(i*4+3+4*m,i*4+2+4*m)=VECTOR3_DP(tmp4, tmp3);
		  	   C_N(i*4+3+4*m,i*4+3+4*m)=VECTOR3_DP(tmp4, tmp4);
               

			   m_real t0;
                
			  
			   t0=(m_real) a0(i)*VECTOR3_DP(tmp1, c.normal)*(0.5);
               b0(i*4)=VECTOR3_DP(tmp1, m_Velocities[index]);
               b0(i*4)+=t0;

			   t0=(m_real) a0(i)*VECTOR3_DP(tmp2, c.normal)*(0.5);
               b0(i*4+1)=VECTOR3_DP(tmp2, m_Velocities[index]);
               b0(i*4+1)+=t0;

			   t0=(m_real) a0(i)*VECTOR3_DP(tmp3, c.normal)*(0.5);
               b0(i*4+2)=VECTOR3_DP(tmp3, m_Velocities[index]);
               b0(i*4+2)+=t0;

			   t0=(m_real) a0(i)*VECTOR3_DP(tmp4, c.normal)*(0.5);
               b0(i*4+3)=VECTOR3_DP(tmp4, m_Velocities[index]);
               b0(i*4+3)+=t0;

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
	
			invA*=-1.0;
			gmm::assign(sinvA,invA);

			temp1.resize(temp3.nrows(),H_f.ncols());
			gmm::mult(temp3, H_f, temp1);	

			matrixn G,CE,CI,E0; 
			vectorn g0,ce0,ci0, _x;
			bool use_qpOASES=true;

			g0.setSize(8*m);
			ce0.setSize(3*m); 
			ci0.setSize(8*m);
			_x.setSize(8*m);

			_x.setAllValue(0.0);
			g0.setAllValue(0.0);
			ci0.setAllValue(0.0);
            ce0.setAllValue(0.0);

			for(int i=0; i<m; i++)
			{

				ci0(i*4+4*m)=b0(i*4); 
				ci0(i*4+1+4*m)=b0(i*4+1); 
				ci0(i*4+2+4*m)=b0(i*4+2); 
				ci0(i*4+3+4*m)=b0(i*4+3);	
			}


			G.setSize(8*m,8*m);
			CE.setSize(3*m,8*m);
			CI.setSize(8*m,8*m);

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

			for(int i=0;i<4*m;i++)
			{
				G(i, i)=0.01;
				G(i+4*m, i+4*m)=0.01;


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
				  }

				CI(i*4, i*4)=C_N(i*4, i*4);         	
				CI(i*4, i*4+1)=C_N(i*4, i*4+1);
				CI(i*4, i*4+2)=C_N(i*4, i*4+2);
				CI(i*4, i*4+3)=C_N(i*4, i*4+3);

				CI(i*4+1, i*4)=C_N(i*4+1, i*4);         	
				CI(i*4+1, i*4+1)=C_N(i*4+1, i*4+1);
				CI(i*4+1, i*4+2)=C_N(i*4+1, i*4+2);
				CI(i*4+1, i*4+3)=C_N(i*4+1, i*4+3);

				CI(i*4+2, i*4)=C_N(i*4+2, i*4);         	
				CI(i*4+2, i*4+1)=C_N(i*4+2, i*4+1);
				CI(i*4+2, i*4+2)=C_N(i*4+2, i*4+2);
				CI(i*4+2, i*4+3)=C_N(i*4+2, i*4+3);

				CI(i*4+3, i*4)=C_N(i*4+3, i*4);         	
				CI(i*4+3, i*4+1)=C_N(i*4+3, i*4+1);
				CI(i*4+3, i*4+2)=C_N(i*4+3, i*4+2);
				CI(i*4+3, i*4+3)=C_N(i*4+3, i*4+3);

				CI(i*4+4*m, i*4+4*m)=C_N(i*4+4*m, i*4+4*m);         	
				CI(i*4+4*m, i*4+1+4*m)=C_N(i*4+4*m, i*4+1+4*m);
				CI(i*4+4*m, i*4+2+4*m)=C_N(i*4+4*m, i*4+2+4*m);
				CI(i*4+4*m, i*4+3+4*m)=C_N(i*4+4*m, i*4+3+4*m);


				CI(i*4+1+4*m, i*4+4*m)=C_N(i*4+1+4*m, i*4+4*m);         	
				CI(i*4+1+4*m, i*4+1+4*m)=C_N(i*4+1+4*m, i*4+1+4*m);
				CI(i*4+1+4*m, i*4+2+4*m)=C_N(i*4+1+4*m, i*4+2+4*m);
				CI(i*4+1+4*m, i*4+3+4*m)=C_N(i*4+1+4*m, i*4+3+4*m);


				CI(i*4+2+4*m, i*4+4*m)=C_N(i*4+2+4*m, i*4+4*m);         	
				CI(i*4+2+4*m, i*4+1+4*m)=C_N(i*4+2+4*m, i*4+1+4*m);
				CI(i*4+2+4*m, i*4+2+4*m)=C_N(i*4+2+4*m, i*4+2+4*m);
				CI(i*4+2+4*m, i*4+3+4*m)=C_N(i*4+2+4*m, i*4+3+4*m);


				CI(i*4+3+4*m, i*4+4*m)=C_N(i*4+3+4*m, i*4+4*m);         	
				CI(i*4+3+4*m, i*4+1+4*m)=C_N(i*4+3+4*m, i*4+1+4*m);
				CI(i*4+3+4*m, i*4+2+4*m)=C_N(i*4+3+4*m, i*4+2+4*m);
				CI(i*4+3+4*m, i*4+3+4*m)=C_N(i*4+3+4*m, i*4+3+4*m);

			} 

			ASSERT(false);
			//solve_quadprog(G,g0,CE,ce0,CI,ci0,_x,use_qpOASES);

			// USE_FORCE

			// A dv= h F where F is contact forces
			// dv = h invA F
			//	  = h invA H_f lambda

			gmm::assign(sinvA, invA);
			std::vector<m_real> dv,Fc, lambda;
			dv.resize(3*n);
			Fc.resize(3*n);

			
#ifdef DEBUG_DRAW
			lines.resize(2*n,3);
#endif
		
			for(int i=0; i<n; i++)
			{

				int index=mContacts_peneltyMethod.m_cti[i].index;

				Fc[3*i]=H_f(i*3,i*4)*_x(4*i)+H_f(i*3,i*4+1)*_x(4*i+1)+H_f(i*3,i*4+2)*_x(4*i+2)+H_f(i*3,i*4+3)*_x(4*i+3);	          
				Fc[3*i+1]=H_f(i*3+1,i*4)*_x(4*i)+H_f(i*3+1,i*4+1)*_x(4*i+1)+H_f(i*3+1,i*4+2)*_x(4*i+2)+H_f(i*3+1,i*4+3)*_x(4*i+3);	          
				Fc[3*i+2]=H_f(i*3+2,i*4)*_x(4*i)+H_f(i*3+2,i*4+1)*_x(4*i+1)+H_f(i*3+2,i*4+2)*_x(4*i+2)+H_f(i*3+2,i*4+3)*_x(4*i+3);	        
	
				vector3 dpf=vector3(Fc[3*i],Fc[3*i+1],Fc[3*i+2]);

				VECTOR3_SCALE( dpf, 0.001 , dpf );
				VECTOR3_ADD(dpf, m_Positions[index], dpf);

#ifdef DEBUG_DRAW
				lines.row(i*2).setVec3(0, m_Positions[index]);
				lines.row(i*2+1).setVec3(0, dpf);

				printf("l1=%f ,l2=%f ,l3=%f  ,l4=%f \n",_x[i*4],_x[i*4+1],_x[i*4+2],_x[i*4+3]);
#endif
			}


#ifdef DEBUG_DRAW
			g_debugObject->registerObject("contactForces", "LineList", "solidred", lines);
#endif


			lambda.resize(4*m);
			for(int i=0; i<lambda.size(); i++)
				lambda[i]=_x[i];

			temp1.resize(sinvA.nrows(), H_f.ncols());
			gmm::mult(sinvA, H_f, temp1); 
			gmm::mult(temp1, lambda, dv);

#ifdef DEBUG_DRAW
			printf("fStep: %f\n",fStep);

			lines_other.resize(2*n,3);
#endif

			for(int i=0; i<n; i++)
			{
				int index=mContacts_peneltyMethod.m_cti[i].index;
				Physics_Contacts_Penalty::cti& c=mContacts_peneltyMethod.m_cti[i];
				m_Velocities[index]+=vector3(dv[i*3], dv[i*3+1], dv[i*3+2]);
				printf("p1=%f ,p2=%f ,p3=%f  ,p4=%f \n",_x[4*m+i*4],_x[4*m+i*4+1],_x[4*m+i*4+2],_x[4*m+i*4+3]);
        
		     	vector3 dpdv=m_Velocities[index];

				VECTOR3_SCALE( dpdv, 1 , dpdv );
				VECTOR3_ADD(dpdv, m_Positions[index], dpdv);

#ifdef DEBUG_DRAW
				lines_other.row(i*2).setVec3(0, m_Positions[index]);
				lines_other.row(i*2+1).setVec3(0, dpdv);
#endif

				vector3 U=vector3(dv[i*3]*fStep, dv[i*3+1]*fStep, dv[i*3+2]*fStep);
				m_y[index]=U;
			}

#ifdef DEBUG_DRAW
			g_debugObject->registerObject("deltaVelocities", "LineList", "solidblue", lines_other);
			
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

#ifdef USE_GMM
		const bool useModifiedConjugateGradient_BW98=false;
#else
		const bool useModifiedConjugateGradient_BW98=true;
#endif

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

			BEGIN_TIMER(update_implicit_solve);
			//sm::UMFsolve(A,b,x);
			Umfsolver umf_A;
			umf_A.umf_factorize(A);
			BEGIN_TIMER(update_implicit_solve_part2);
			umf_A.umf_solve(x , b);
			END_TIMER2(update_implicit_solve_part2);
			END_TIMER2(update_implicit_solve);

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
	END_TIMER2(update_implicit);
}

void Physics_ParticleSystem::IntegrateDV( double fStep)
{
	m_Velocities.Add( m_dv, m_Velocities );
	m_Velocities.Scale( fStep, m_vTemp1 );		
	m_Positions.Add( m_vTemp1, m_Positions );
	m_Positions.Add( m_y, m_Positions );
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
