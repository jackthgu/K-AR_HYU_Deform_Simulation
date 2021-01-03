// Physics_SparseSymmetricMatrix.h: interface for the Physics_SparseSymmetricMatrix class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_PHYSICS_SPARSESYMMETRICMATRIX_H__437FC57A_AF43_40F1_8889_7D0E36ECE7F3__INCLUDED_)
#define AFX_PHYSICS_SPARSESYMMETRICMATRIX_H__437FC57A_AF43_40F1_8889_7D0E36ECE7F3__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include <vector>
#ifdef USE_GMM
#include "dependencies/gmm-3.0/include/gmm/gmm_vector.h"
#include "dependencies/gmm-3.0/include/gmm/gmm_matrix.h"
#endif

// don't have to be a symmetric matrix when USE_GMM is defined.
class Physics_SparseSymmetricMatrix  
{
#ifdef USE_GMM
public:
	//typedef gmm::wsvector<matrix3> svector_mat3 ;
	typedef gmm::rsvector<matrix3> svector_mat3 ;
	gmm::row_matrix<svector_mat3> m_data;
	int rows() const;
	int cols() const;
#else
protected:

	int m_iRows, m_iColumns;
	int m_iValues;
	int *m_pRows;
	int *m_pColumns;

	int m_dwCheckSum;

	matrix3* m_pData;
#endif
public:
	Physics_SparseSymmetricMatrix( int iRows, int iColumns );
	virtual ~Physics_SparseSymmetricMatrix();

	void Empty();
	void zero();

#ifdef USE_GMM
	gmm::ref_elt_vector<matrix3, svector_mat3> operator() (int row, int col) ;
#else
    matrix3 & operator() (int i, int j);
#endif
	void mult(Physics_SparseSymmetricMatrix const& a, m_real b);
	void add( Physics_SparseSymmetricMatrix const& a, Physics_SparseSymmetricMatrix const& b);
	void sub( Physics_SparseSymmetricMatrix const& a, Physics_SparseSymmetricMatrix const& b);
	
	// deprecated
	bool PostMultiply( Physics_LargeVector &V, Physics_LargeVector &Dst );
	bool PreMultiply( Physics_LargeVector &V, Physics_LargeVector &Dst );
	bool Add( Physics_SparseSymmetricMatrix &M, Physics_SparseSymmetricMatrix &Dst );
	bool Subtract( Physics_SparseSymmetricMatrix &M, Physics_SparseSymmetricMatrix &Dst );
	bool Scale( m_real scale, Physics_SparseSymmetricMatrix &Dst ) const;
	void Dump( char *szTitle = NULL );
	//bool IsSymmetric();
	//bool IsPositiveDefinite();
	void Copy( Physics_SparseSymmetricMatrix const&M );
};

#endif // !defined(AFX_PHYSICS_SPARSESYMMETRICMATRIX_H__437FC57A_AF43_40F1_8889_7D0E36ECE7F3__INCLUDED_)
