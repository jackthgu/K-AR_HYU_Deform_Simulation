// Physics_SparseSymmetricMatrix.cpp: implementation of the Physics_SparseSymmetricMatrix class.
//
//////////////////////////////////////////////////////////////////////

#include "DRGShell.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////
#ifdef USE_GMM

#include "gmm.h"
class Physics_SparseSymmetricMatrix_Data
{
public:
	Physics_SparseSymmetricMatrix_Data(){}
	~Physics_SparseSymmetricMatrix_Data(){}
};
#endif

Physics_SparseSymmetricMatrix::Physics_SparseSymmetricMatrix( int iRows, int iColumns)
{
#ifdef USE_GMM
	gmm::resize(m_data, iRows, iColumns);
#else
	//
	// Do some sanity checking -- probably should throw an exception :-)
	//
	iRows = std::max( 1, iRows );
	iColumns = std::max( 1, iColumns );
	m_iRows = iRows;
	m_iColumns = iColumns;
	m_iValues = 0;
	m_pRows = new int[iRows+1];
	memset( m_pRows, 0, (iRows+1) * sizeof( int ) );
	m_pColumns = NULL;
	m_pData = NULL;
	m_dwCheckSum = 0;
#endif
}

Physics_SparseSymmetricMatrix::~Physics_SparseSymmetricMatrix()
{
#ifndef USE_GMM
	SAFE_DELETE_ARRAY( m_pColumns );
	SAFE_DELETE_ARRAY( m_pRows );
	//SAFE_DELETE_ARRAY( m_pData ); //죽음. 왜지?
#endif
}

void Physics_SparseSymmetricMatrix::Empty()
{
#ifdef USE_GMM
	m_data.clear_mat();
#else
	SAFE_DELETE_ARRAY( m_pColumns );
	SAFE_DELETE_ARRAY( m_pData );

	m_iValues = 0;
	memset( m_pRows, 0, (m_iRows+1) * sizeof( int ) );
	m_dwCheckSum = 0;
#endif
}

void Physics_SparseSymmetricMatrix::zero()
{
#ifdef USE_GMM
	m_data.clear_mat();
#else
	if( m_pData && m_iValues ) 
		memset( m_pData, 0, m_iValues * sizeof( matrix3 ) );
#endif
}

#ifdef USE_GMM
void gmm_scale(gmm::row_matrix<Physics_SparseSymmetricMatrix::svector_mat3> const& in, double scalef, gmm::row_matrix<Physics_SparseSymmetricMatrix::svector_mat3> & out)
{
	gmm::copy(in, out);

	for(int i=0; i<out.nrows(); i++)
	{
		Physics_SparseSymmetricMatrix::svector_mat3& v1=out[i];
		gmm::linalg_traits<Physics_SparseSymmetricMatrix::svector_mat3>::iterator it = gmm::vect_begin(v1);
		gmm::linalg_traits<Physics_SparseSymmetricMatrix::svector_mat3>::iterator ite = gmm::vect_end(v1);

		for(;it!=ite; ++it)
		{
			(*it)*=scalef;
		}
	}
}

vector3 dotProduct(Physics_SparseSymmetricMatrix::svector_mat3 const& v1, std::vector<vector3> const& v2)
{
	vector3 out(0);
	vector3 temp;
	
	gmm::linalg_traits<Physics_SparseSymmetricMatrix::svector_mat3>::const_iterator it = gmm::vect_const_begin(v1);
	gmm::linalg_traits<Physics_SparseSymmetricMatrix::svector_mat3>::const_iterator ite = gmm::vect_const_end(v1);
	
	while (it != ite) 
	{ // loops on the components
		out+=(*it)*v2[it.index()];
		++it;
	}
	return out;
}

#endif
bool Physics_SparseSymmetricMatrix::PostMultiply( Physics_LargeVector &V, Physics_LargeVector &Dst )
{
#ifdef USE_GMM
	//gmm::mult(m_data, V.mVec, Dst.mVec);
	Dst.zero();

	for(int i=0; i<m_data.nrows(); i++)
	{
		Dst[i]+=dotProduct(m_data[i], V.mVec);
	}

	return true;
#else
	if( (Dst.size() != m_iRows) || (V.size() != m_iColumns) )
		return false;

	int i, j, ct, idx, iCol;
	vector3 tmp;

	Dst.zero();

	if( m_iValues )
	{
		for( i=0; i<m_iRows; i++ )
		{
			idx = m_pRows[i];
			ct = m_pRows[i+1] - idx;
			for( j = 0; j<ct; j++ )
			{
				iCol = m_pColumns[idx+j];
				Dst[i].x += m_pData[idx+j]._11 * V[iCol].x + m_pData[idx+j]._12 * V[iCol].y + m_pData[idx+j]._13 * V[iCol].z;
				Dst[i].y += m_pData[idx+j]._21 * V[iCol].x + m_pData[idx+j]._22 * V[iCol].y + m_pData[idx+j]._23 * V[iCol].z;
				Dst[i].z += m_pData[idx+j]._31 * V[iCol].x + m_pData[idx+j]._32 * V[iCol].y + m_pData[idx+j]._33 * V[iCol].z;
				if( i != iCol )
				{
					Dst[iCol].x += V[i].x * m_pData[idx+j]._11 + V[i].y * m_pData[idx+j]._21 + V[i].z * m_pData[idx+j]._31;
					Dst[iCol].y += V[i].x * m_pData[idx+j]._12 + V[i].y * m_pData[idx+j]._22 + V[i].z * m_pData[idx+j]._32;
					Dst[iCol].z += V[i].x * m_pData[idx+j]._13 + V[i].y * m_pData[idx+j]._23 + V[i].z * m_pData[idx+j]._33;
				}
			}
		}
	}
#endif
	return true;
}

//
// Calculate Dst = V * this
//
bool Physics_SparseSymmetricMatrix::PreMultiply( Physics_LargeVector &V, Physics_LargeVector &Dst )
{
#ifdef USE_GMM
	//gmm::mult(gmm::transposed(m_data), V.mVec, Dst.mVec);

	Dst.zero();

	for(int i=0; i<m_data.nrows(); i++)
	{
		vector3 temp;
		
		svector_mat3& v1=m_data[i];
		gmm::linalg_traits<svector_mat3>::const_iterator it = gmm::vect_const_begin(v1);
		gmm::linalg_traits<svector_mat3>::const_iterator ite = gmm::vect_const_end(v1);
		
		while (it != ite) 
		{ // loops on the components
			int j=it.index();
			Dst[j]+=V[i]*(*it);
			++it;
		}
	}
	return true;
#else

	if( (Dst.size() != m_iColumns) || (V.size() != m_iRows) )
		return false;

	int i, j, ct, idx;

	Dst.zero();

	if( m_iValues )
	{
		for( i=0; i<m_iRows; i++ )
		{
			idx = m_pRows[i];
			ct = m_pRows[i+1] - idx;
			for( j = 0; j<ct; j++ )
			{
				Dst[i].x += V[m_pColumns[idx+j]].x * m_pData[idx+j]._11 + V[m_pColumns[idx+j]].y * m_pData[idx+j]._21 + V[m_pColumns[idx+j]].z * m_pData[idx+j]._31;
				Dst[i].y += V[m_pColumns[idx+j]].x * m_pData[idx+j]._12 + V[m_pColumns[idx+j]].y * m_pData[idx+j]._22 + V[m_pColumns[idx+j]].z * m_pData[idx+j]._32;
				Dst[i].z += V[m_pColumns[idx+j]].x * m_pData[idx+j]._13 + V[m_pColumns[idx+j]].y * m_pData[idx+j]._23 + V[m_pColumns[idx+j]].z * m_pData[idx+j]._33;
				if( i != m_pColumns[idx+j] )
				{
					Dst[m_pColumns[idx+j]].x += V[i].x * m_pData[idx+j]._11 + V[i].y * m_pData[idx+j]._12 + V[i].z * m_pData[idx+j]._13;
					Dst[m_pColumns[idx+j]].y += V[i].x * m_pData[idx+j]._21 + V[i].y * m_pData[idx+j]._22 + V[i].z * m_pData[idx+j]._23;
					Dst[m_pColumns[idx+j]].z += V[i].x * m_pData[idx+j]._31 + V[i].y * m_pData[idx+j]._32 + V[i].z * m_pData[idx+j]._33;
				}
			}
		}
	}
#endif
	return true;
}

//
// Calculate Dst = this + M
//
bool Physics_SparseSymmetricMatrix::Add( Physics_SparseSymmetricMatrix &M, Physics_SparseSymmetricMatrix &Dst )
{
	Dst.add(*this,M);
	return true;
}

void Physics_SparseSymmetricMatrix::add( Physics_SparseSymmetricMatrix const& a, Physics_SparseSymmetricMatrix const& M)
{
#ifdef USE_GMM
	gmm::add(a.m_data, M.m_data, this->m_data);
#else
	Physics_SparseSymmetricMatrix &Dst =*this;

	ASSERT( (M.m_iColumns == a.m_iColumns) && (Dst.m_iColumns == a.m_iColumns) &&
			(M.m_iRows == a.m_iRows) && (Dst.m_iRows == a.m_iRows) );

	int i, j, ct, idx;

	if( a.m_dwCheckSum && (Dst.m_dwCheckSum == a.m_dwCheckSum) && (M.m_dwCheckSum == a.m_dwCheckSum) )
	{
		for( i=0; i<a.m_iValues; i++ )
		{
			Dst.m_pData[i] .add(a.m_pData[i],M.m_pData[i]);
		}
	}
	else
	{
		Dst.zero();

		if( a.m_iValues )
		{
			for( i=0; i<a.m_iRows; i++ )
			{
				idx = a.m_pRows[i];
				ct = a.m_pRows[i+1] - idx;
				for( j = 0; j<ct; j++ )
				{
					Dst(i, a.m_pColumns[idx+j] ) = a.m_pData[idx+j];
				}
			}
		}
		if( M.m_iValues )
		{
			for( i=0; i<M.m_iRows; i++ )
			{
				idx = M.m_pRows[i];
				ct = M.m_pRows[i+1] - idx;
				for( j = 0; j<ct; j++ )
				{
					Dst(i, M.m_pColumns[idx+j] )+=M.m_pData[idx+j];
				}
			}
		}
	}
#endif
}

void Physics_SparseSymmetricMatrix::sub( Physics_SparseSymmetricMatrix const& a, Physics_SparseSymmetricMatrix const& M)
{
#ifdef USE_GMM
	gmm::row_matrix<svector_mat3> temp;
	gmm::resize(temp, M.m_data.nrows(), M.m_data.ncols());
	gmm_scale(M.m_data, -1, temp);
	gmm::add(a.m_data, temp, this->m_data);
	//gmm::add(a.m_data, gmm::scaled(M.m_data, matrix3(-1.0)), this->m_data);
#else
	Physics_SparseSymmetricMatrix &Dst =*this;

	ASSERT( (M.m_iColumns == a.m_iColumns) && (Dst.m_iColumns == a.m_iColumns) &&
			(M.m_iRows == a.m_iRows) && (Dst.m_iRows == a.m_iRows) );

	int i, j, ct, idx;

	if( a.m_dwCheckSum && (Dst.m_dwCheckSum == a.m_dwCheckSum) && (M.m_dwCheckSum == a.m_dwCheckSum) )
	{
		for( i=0; i<a.m_iValues; i++ )
		{
			Dst.m_pData[i] .sub(a.m_pData[i],M.m_pData[i]);
		}
	}
	else
	{
		Dst.zero();

		if( a.m_iValues )
		{
			for( i=0; i<a.m_iRows; i++ )
			{
				idx = a.m_pRows[i];
				ct = a.m_pRows[i+1] - idx;
				for( j = 0; j<ct; j++ )
				{
					Dst(i, a.m_pColumns[idx+j] ) = a.m_pData[idx+j];
				}
			}
		}
		if( M.m_iValues )
		{
			for( i=0; i<M.m_iRows; i++ )
			{
				idx = M.m_pRows[i];
				ct = M.m_pRows[i+1] - idx;
				for( j = 0; j<ct; j++ )
				{
					Dst(i, M.m_pColumns[idx+j] )-=M.m_pData[idx+j];
				}
			}
		}
	}
#endif
}


//
// Calculate Dst = this - M
//
bool Physics_SparseSymmetricMatrix::Subtract( Physics_SparseSymmetricMatrix &M, Physics_SparseSymmetricMatrix &Dst )
{
	Dst.sub(*this,M);
	return true;
}

void Physics_SparseSymmetricMatrix::mult( Physics_SparseSymmetricMatrix const& a, m_real scale )
{
#ifdef USE_GMM
	// 이렇게 구현하면 matrix3(scale,scale,scale,...)이 곱해지면서 실제로는 scale되지 않고, 이상해진다.
	//gmm::copy(a.m_data,this->m_data);
	//gmm::scale(this->m_data, matrix3(scale));

	gmm_scale(a.m_data, scale, m_data);
#else
	Physics_SparseSymmetricMatrix &Dst =*this;
	int i, j, ct, idx;

	if( a.m_dwCheckSum && (Dst.m_dwCheckSum == a.m_dwCheckSum) )
	{
		for( i=0; i<a.m_iValues; i++ )
		{
			a.m_pData[i].Multiply( scale, Dst.m_pData[i] );
		}
	}
	else
	{
		Dst.zero();
		if( a.m_iValues )
		{
			for( i=0; i<a.m_iRows; i++ )
			{
				idx = a.m_pRows[i];
				ct = a.m_pRows[i+1] - idx;
				for( j = 0; j<ct; j++ )
				{
					Dst(i,a.m_pColumns[idx+j]).mult(a.m_pData[idx+j],scale);
				}
			}
		}
	}
#endif
}



bool Physics_SparseSymmetricMatrix::Scale( m_real scale, Physics_SparseSymmetricMatrix &Dst ) const
{
	Dst.mult(*this, scale);
	return true;
}


#ifdef USE_GMM

gmm::ref_elt_vector<matrix3, Physics_SparseSymmetricMatrix::svector_mat3> Physics_SparseSymmetricMatrix::operator() (int row, int col) 
{
	//m_data(row, col)
	//gmm::ref_elt_vector<matrix3, svector_mat3> t(NULL,0);
	//t=(const gmm::ref_elt_vector<matrix3, svector_mat3> &)m_data(row, col);
	//return m_data[row][col];

	svector_mat3* ptr=(Physics_SparseSymmetricMatrix::svector_mat3* )&m_data.row(row);
	return gmm::ref_elt_vector<matrix3, Physics_SparseSymmetricMatrix::svector_mat3> (ptr, gmm::size_type(col));
	//(row, col);
}

int Physics_SparseSymmetricMatrix::rows() const
{
	return m_data.nrows();
}
int Physics_SparseSymmetricMatrix::cols() const
{
	return m_data.ncols();
}
#else
matrix3 & Physics_SparseSymmetricMatrix::operator() (int row, int col)
{

	int i, k, l, ct;

	if( row > col  )
	{
		k = row;
		row = col;
		col = k;
	}

	k = m_pRows[row];
	ct = m_pRows[row+1] - k;
	for( l=0; l<ct; l++ )
		if( m_pColumns[k+l] == col )
			break;
	if(  (ct != 0) && (l != ct) )
	{
		return m_pData[k+l];
	}
	else
	{			
		int *pSave = m_pColumns;
		matrix3 *pSaveData = m_pData;
		
		m_pColumns = new int[m_iValues+1];
		m_pData = new matrix3[m_iValues+1];

		if( pSave && k)
		{
			memcpy( m_pColumns, pSave, k * sizeof( int ) );
			memcpy( m_pData, pSaveData, k * sizeof( matrix3 ) );
		}
		for( l=0; l<ct; l++ )
		{
			if( pSave[k+l] < col )
			{
				m_pColumns[k+l] = pSave[k+l];
				m_pData[k+l] = pSaveData[k+l];
			}
			else
			{
				break;
			}
		}
		m_pColumns[k+l] = col;
		m_pData[k+l].zero();
		for( i=row+1; i<=m_iRows; i++ )
			m_pRows[i]++;
		if( pSave  )
		{
			if( (m_iValues-k-l) > 0 )
				memcpy( &m_pColumns[k+l+1], &pSave[k+l], (m_iValues-k-l) * sizeof( int ) );
			delete [] pSave;
		}
		if( pSaveData   )
		{
			if( (m_iValues-k-l) > 0 )
				memcpy( &m_pData[k+l+1], &pSaveData[k+l], (m_iValues-k-l) * sizeof( matrix3 ) );
			delete [] pSaveData;
		}
		m_iValues++;
		//
		// How well does this work as a check sum?  Who knows :-)
		//
		m_dwCheckSum += (row+1) * 190237 + (col+1);

		return m_pData[k+l];
	}
}
#endif

void Physics_SparseSymmetricMatrix::Dump( char *szTitle )
{
#ifndef USE_GMM
	char szTemp[100];
	int i,j, k, idx, ct;

	if( szTitle != NULL )
		Msg::print("%s\n", szTitle );

	sprintf( szTemp, "Rows: %d Columns: %d\r\n", m_iRows, m_iColumns );
	Msg::print("%s\n", szTemp );

	if( m_iValues )
	{
		for( i=0; i<m_iRows; i++ )
		{
			idx = m_pRows[i];
			ct = m_pRows[i+1] - idx;
			for( k=0; k<3; k++ )
			{
				for( j = 0; j<ct; j++ )
				{
					sprintf( szTemp, "(%3d,%3d): %8.4f %8.4f %8.4f ", i, m_pColumns[idx+j], 
						m_pData[idx+j].m[k][0],
						m_pData[idx+j].m[k][1],
						m_pData[idx+j].m[k][2]);
					Msg::print("%s\n", szTemp );
				}
				Msg::print("%s\n", "\r\n" );
			}
		}
	}
#endif
}

/*
bool Physics_SparseSymmetricMatrix::IsSymmetric()
{
	int i, j, k, idx, ct, idx2, ct2;

	if( !m_iValues )
		return true;

	for( i=0; i<m_iRows; i++ )
	{
		idx = m_pRows[i];
		ct = m_pRows[i+1] - idx;
		for( j = 0; j<ct; j++ )
		{
			idx2 = m_pRows[m_pColumns[idx+j]];
			ct2 = m_pRows[m_pColumns[idx+j]+1] - idx2;
			for( k = 0; k<ct2; k++ )
			{
				if( m_pColumns[idx2+k] == i )
				{
					if( !m_pData[idx+j].isTranspose( m_pData[idx2+k] ) )
						return false;
					else
						break;
				}
			}
			if( ct2 && (k == ct2) )
				return false;
		}
	}
	return true;
}*/
/*
bool Physics_SparseSymmetricMatrix::IsPositiveDefinite()
{
	int i, iRow = 0;
	m_real sum = 0;

	if( !m_iValues )
		return true;

	for( i=0; i<m_iValues; i++ )
	{
		while( i > m_pRows[iRow+1] && (m_pRows[iRow+1] > m_pRows[iRow]) )
			iRow++;
		if( (iRow == m_pColumns[i]) && ((m_pData[i]._11 <= 0 )||
			(m_pData[i]._22 <= 0 ) ||
			(m_pData[i]._33 <= 0 ) ))
			return false;
		sum += m_pData[i]._11 + m_pData[i]._12 + m_pData[i]._13+
			   m_pData[i]._21 + m_pData[i]._22 + m_pData[i]._23+
			   m_pData[i]._31 + m_pData[i]._32 + m_pData[i]._33;
	}
	return (sum>0);
}*/

//
// Set this = M
//
void Physics_SparseSymmetricMatrix::Copy( Physics_SparseSymmetricMatrix const&M )
{
#ifdef USE_GMM

	gmm::copy(M.m_data, m_data);
#else
	SAFE_DELETE_ARRAY( m_pRows );
	SAFE_DELETE_ARRAY( m_pColumns );
	SAFE_DELETE_ARRAY( m_pData );

	m_iRows = M.m_iRows;
	m_iColumns = M.m_iColumns;
	m_iValues = M.m_iValues;
	m_pRows = new int[m_iRows+1];
	memcpy( m_pRows, M.m_pRows, (m_iRows+1) * sizeof( int ) );
	m_pColumns = new int[m_iValues];
	memcpy( m_pColumns, M.m_pColumns, (m_iValues) * sizeof( int ) );
	m_pData = new matrix3[m_iValues];
	memcpy( m_pData, M.m_pData, (m_iValues) * sizeof( matrix3 ) );
	m_dwCheckSum = M.m_dwCheckSum;
#endif
}
