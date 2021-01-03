#ifndef UMFSOLVER_H
#define UMFSOLVER_H

#ifdef _MSC_VER
#include "../../../dependencies/UMFPACK5.2/UMFPACK/include/umfpack.h"
#else
#ifdef __APPLE__
#include <umfpack.h>
#else
#include "/usr/include/suitesparse/umfpack.h"
#endif
#endif
#include "dependencies/gmm-3.0/include/gmm/gmm_interface.h"
#include "dependencies/gmm-3.0/include/gmm/gmm_kernel.h"




namespace gmm
{
	typedef wsvector<m_real> wsvectorn;	// sparse vector for write operations
	typedef rsvector<m_real> rsvectorn; // sparse vector for read operations

	typedef row_matrix<wsvectorn> wsmatrixn;	// sparse matrix for write operations
	typedef col_matrix<wsvectorn> wsmatrixn2;	// sparse matrix for write operations
	typedef row_matrix<rsvectorn> rsmatrixn;		// sparse matrix for read operations

	typedef linalg_traits<wsvectorn>::iterator wsv_iterator;
	typedef linalg_traits<wsvectorn>::const_iterator wsv_const_iterator;
	void conversion(rsmatrixn & out, wsmatrixn & in);


	void assign(wsmatrixn& out, matrixn const& in);
	void zero(wsmatrixn& inout, int n, int m);

}

class Umfsolver{
	
	intvectorn Ap;
	intvectorn Ai;
	vectorn Ax;
	int _n, _m ;

public:
	Umfsolver()	{}
	~Umfsolver(){}

	// factorize matrix A
	void umf_factorize(matrixn const& A)
	{
		_n = A.rows();
		_m = A.cols();

		Ai.setSize(0);
		Ax.setSize(0);
		Ap.setSize(_n+1);

		for (int j =0; j<_m; j++)
		{
			Ap[j] = Ai.size() ;

			for (int i =0; i<_n; i++)
			{
				//cout << "[ " << i << ", " << j <<" ]" << A[i][j] << endl;

				if (A[i][j] != 0.0)
				{
					Ai.pushBack(i);
					Ax.pushBack(A[i][j]);
				}
			}
		}

		
		Ap[_n] = Ai.size();		
	}

	// factorize gmm::wsmatrixn A
	void umf_factorize(gmm::wsmatrixn const& A)
	{
		_n = gmm::mat_nrows(A);
		_m = gmm::mat_ncols(A);

		Ai.setSize(0);
		Ax.setSize(0);
		Ap.setSize(_n+1);

		for (int j =0; j<_m; j++)
		{
			Ap[j] = Ai.size() ;

			for(gmm::wsv_const_iterator i=A[j].begin(); i!=A[j].end(); i++)
			{
				//cout << "[ " << i << ", " << j <<" ]" << A[i][j] << endl;

				if (*i!= 0.0)
				{
					Ai.pushBack(i.index());
					Ax.pushBack(*i);
				}
			}
		}

		
		Ap[_n] = Ai.size();		
	}

	// solve A'x=b  (note that the input matrix A gets transposed.)
	void umf_solve(vectorn & x, vectorn const& b) const
	{
		int n=_n;
		int m=_m;

		x.setSize(m);
		void *Symbolic, *Numeric ;
		double *null = (double *) NULL ;
//#define REPORT_MEMORY_USAGE
#ifdef REPORT_MEMORY_USAGE
		double info[UMFPACK_INFO];
		(void) umfpack_di_symbolic (n, m, Ap.dataPtr(), Ai.dataPtr(), Ax.dataPtr(), &Symbolic, null, info) ;
		printf("peak estimates %g*%g numeric estimates %g symbolic size %g\n", info[UMFPACK_PEAK_MEMORY_ESTIMATE], info[UMFPACK_SIZE_OF_UNIT], info[ UMFPACK_NUMERIC_SIZE_ESTIMATE], info[UMFPACK_SYMBOLIC_SIZE]);
		(void) umfpack_di_numeric (Ap.dataPtr(), Ai.dataPtr(), Ax.dataPtr(), Symbolic, &Numeric, null, info) ;
		printf("peak %g, flops %g numeric size %g\n", info[UMFPACK_PEAK_MEMORY], info[UMFPACK_FLOPS_ESTIMATE], info[UMFPACK_NUMERIC_SIZE]);
		umfpack_di_free_symbolic (&Symbolic) ;
		(void) umfpack_di_solve (UMFPACK_A, Ap.dataPtr(), Ai.dataPtr(), Ax.dataPtr(), x.dataPtr(), b.dataPtr(), Numeric, null, info) ;
		printf("solve flops %g, time %g\n", info[UMFPACK_SOLVE_FLOPS], info [UMFPACK_SOLVE_TIME]);
		umfpack_di_free_numeric (&Numeric) ;
#else
		(void) umfpack_di_symbolic (n, m, Ap.dataPtr(), Ai.dataPtr(), Ax.dataPtr(), &Symbolic, null, null) ;
		(void) umfpack_di_numeric (Ap.dataPtr(), Ai.dataPtr(), Ax.dataPtr(), Symbolic, &Numeric, null, null) ;
		umfpack_di_free_symbolic (&Symbolic) ;
		(void) umfpack_di_solve (UMFPACK_A, Ap.dataPtr(), Ai.dataPtr(), Ax.dataPtr(), x.dataPtr(), b.dataPtr(), Numeric, null, null) ;
		umfpack_di_free_numeric (&Numeric) ;
#endif
	}

private:
};

namespace sm
{
	template <class MAT_TYPE>
	static void UMFsolve(MAT_TYPE const& A, vectorn const& b, vectorn & x)
	{
		Umfsolver umf_A;
		umf_A.umf_factorize(A);
		umf_A.umf_solve(x , b);
	}
	template <class MAT_TYPE>
	static vectorn UMFfactorize(MAT_TYPE const& A)
	{
		union {
			double dbl;
			void* ptr;
		} conv;
		Umfsolver* umf_A=new Umfsolver();
		umf_A->umf_factorize(A);
		conv.ptr=(void*)umf_A;

		vectorn v(1);
		v(0)=conv.dbl;
		return v;
	}
	static void UMFsolveFactorized(vectorn const& solver, vectorn const& b, vectorn &x)
	{
		// its usually safe to assume sizeof(dbl)>=sizeof(void*)
		union {
			double dbl;
			void* ptr;
		} conv;
		conv.dbl=solver(0);
		Umfsolver* umf_A=(Umfsolver*)conv.ptr;
		umf_A->umf_solve(x,b);
	}
	static void freeUMFsolver(vectorn const& solver)
	{
		union {
			double dbl;
			void* ptr;
		} conv;
		conv.dbl=solver(0);
		Umfsolver* umf_A=(Umfsolver*)conv.ptr;
		delete umf_A;
	}
}

#endif
