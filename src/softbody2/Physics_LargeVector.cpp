// Physics_LargeVector.cpp: implementation of the Physics_LargeVector class.
//
//////////////////////////////////////////////////////////////////////

#include "DRGShell.h"

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

#ifdef USE_GMM
#include "gmm.h"

Physics_LargeVector::Physics_LargeVector( int iElements )
{
	mVec.resize(iElements);
}	
Physics_LargeVector::~Physics_LargeVector()
{
}
void Physics_LargeVector::operator=( Physics_LargeVector const&copy ){ gmm::copy(copy.mVec,mVec);}
void Physics_LargeVector::Resize( int iNewElements ){mVec.resize(iNewElements);}

#else
Physics_LargeVector::Physics_LargeVector( int iElements )
:_tvectorn<vector3, m_real>()
{
	setSize(iElements);
}
Physics_LargeVector::~Physics_LargeVector()
{
}
void Physics_LargeVector::operator=( Physics_LargeVector const&copy ){_tvectorn<vector3, m_real>::assign(copy);}
void Physics_LargeVector::Resize( int iNewElements ){resize(iNewElements);}

#endif



void Physics_LargeVector::zero()
{
	if( size())
		memset(&value(0), 0, size()* sizeof( vector3 ) );
}



void Physics_LargeVector::add( Physics_LargeVector const& a, Physics_LargeVector const& b) 
{
	ASSERT((size() == a.size() ) && (size() == b.size() ) );
	
	for( int i=0; i<size(); i++ )
	{
		value(i).add(a[i], b[i]);
	}
}

void Physics_LargeVector::operator+=(Physics_LargeVector const& b) 
{

	ASSERT((size() == b.size() ) );
	
	for( int i=0; i<size(); i++ )
	{
		value(i)+=b[i];
	}
}

void Physics_LargeVector::operator*=(Physics_LargeVector const& b) 
{

	ASSERT((size() == b.size() ) );
	
	for( int i=0; i<size(); i++ )
	{
		value(i).x*=b[i].x;
		value(i).y*=b[i].y;
		value(i).z*=b[i].z;
	}
}

void Physics_LargeVector::operator*=(m_real b) 
{
	for( int i=0; i<size(); i++ )
	{
		value(i)*=b;
	}
}


void Physics_LargeVector::sub( Physics_LargeVector const& a, Physics_LargeVector const& b) 
{
	ASSERT((size() == a.size() ) && (size() == b.size() ) );
	
	for( int i=0; i<size(); i++ )
	{
		value(i).sub(a[i], b[i]);
	}
}

void Physics_LargeVector::mult(Physics_LargeVector const& a, Physics_LargeVector const& b)
{
	ASSERT((size() == a.size() ) && (size() == b.size() ) );

	for( int i=0; i<size(); i++ )
	{
		value(i).x=a[i].x*b[i].x;
		value(i).y=a[i].y*b[i].y;
		value(i).z=a[i].z*b[i].z;
	}
}


bool Physics_LargeVector::Add( Physics_LargeVector &v, Physics_LargeVector &dst )
{
	if( (size() != v.size() ) || (size() != dst.size() ) )
		return false;

	for( int i=0; i<size(); i++ )
	{
		VECTOR3_ADD( (*this)[i], v[i], dst[i] );
	}
	return true;
}

bool Physics_LargeVector::Subtract( Physics_LargeVector &v, Physics_LargeVector &dst )
{
	if( (size() != v.size() ) || (size() != dst.size() ) )
		return false;

	for( int i=0; i<size(); i++ )
	{
		VECTOR3_SUB( (*this)[i], v[i], dst[i] );
	}
	return true;
}

bool Physics_LargeVector::ElementMultiply( Physics_LargeVector &v, Physics_LargeVector &dst )
{
	if( (size() != v.size() ) || (size() != dst.size() ) )
		return false;

	for( int i=0; i<size(); i++ )
	{
		dst[i].x = (*this)[i].x * v[i].x;
		dst[i].y = (*this)[i].y * v[i].y;
		dst[i].z = (*this)[i].z * v[i].z;
	}
	return true;
}

bool Physics_LargeVector::ElementMultiply( matrix3 S[], Physics_LargeVector &dst )
{
	if( size() != dst.size() )
		return false;

	for( int i=0; i<size(); i++ )
	{
		S[i].PreMultiply( (*this)[i], dst[i] );
	}
	return true;
}

bool Physics_LargeVector::Scale( m_real scale, Physics_LargeVector &dst )
{
	if( size() != dst.size() )
		return false;

	for( int i=0; i<size(); i++ )
	{
		VECTOR3_SCALE( (*this)[i], scale, dst[i] );
	}
	return true;
}

bool Physics_LargeVector::Invert( Physics_LargeVector &dst )
{
	if( size() != dst.size() )
		return false;

	for( int i=0; i<size(); i++ )
	{
		dst[i].x = (m_real)1.0 / (*this)[i].x;
		dst[i].y = (m_real)1.0 / (*this)[i].y;
		dst[i].z = (m_real)1.0 / (*this)[i].z;
	}
	return true;
}

m_real Physics_LargeVector::DotProduct( Physics_LargeVector &v )
{
	m_real res = (m_real)0.0;

	for( int i=0; i<size(); i++ )
		res += (*this)[i].x * v[i].x + 
			   (*this)[i].y * v[i].y + 
			   (*this)[i].z * v[i].z;
	return res;
}

		
		

void Physics_LargeVector::Dump( char *szTitle )
{
	char szTemp[500];

	if( szTitle )
		Msg::print("%s\n", szTitle );

	sprintf( szTemp, "Elements: %d\r\n", size() );

	for( int i=0; i<size(); i++ )
	{
		sprintf( szTemp, "%3d: %8.4f %8.4f %8.4f\r\n", i, (*this)[i].x, (*this)[i].y, (*this)[i].z );
		Msg::print("%s\n", szTemp );
	}
}
