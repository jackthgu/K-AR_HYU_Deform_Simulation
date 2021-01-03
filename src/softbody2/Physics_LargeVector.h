// Physics_LargeVector.h: interface for the Physics_LargeVector class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_PHYSICS_LARGEVECTOR_H__FDF65582_D36F_4061_A343_BDDA44E2DFDE__INCLUDED_)
#define AFX_PHYSICS_LARGEVECTOR_H__FDF65582_D36F_4061_A343_BDDA44E2DFDE__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000


#include <assert.h>
#include "Terrain.h"

class matrix3;
class vector3;

class Physics_LargeVector  
#ifndef USE_GMM
	: public _tvectorn<vector3, m_real> // so this is almost like vector3N, but has different APIs
#endif
{

public:

#ifdef USE_GMM
	std::vector<vector3> mVec;
	inline vector3& value(int i)				{return mVec[i];}
	inline vector3 const& value(int i)	const	{return mVec[i];}
	inline vector3& operator[](int i)				{return mVec[i];}
	inline vector3 const& operator[](int i)	const	{return mVec[i];}

	inline int size() const						{return (int)mVec.size();}
#endif
	Physics_LargeVector( int iElements=1 );
	virtual ~Physics_LargeVector();

	void zero();
	void add(Physics_LargeVector const& a, Physics_LargeVector const& b);
	void sub(Physics_LargeVector const& a, Physics_LargeVector const& b);
	void mult(Physics_LargeVector const& a, Physics_LargeVector const& b);
	void mult(Physics_LargeVector const& a, m_real scale);

	void operator=( Physics_LargeVector const&copy );

	void operator+=(Physics_LargeVector const& b) ;
	void operator*=(Physics_LargeVector const& b) ;
	void operator*=(m_real b) ;


	// deprecated:
	inline int Size()	{ return size();}
	void Resize( int iNewElements );

	bool Add( Physics_LargeVector &v, Physics_LargeVector &dst );
	bool Subtract( Physics_LargeVector &v, Physics_LargeVector &dst );
	m_real DotProduct( Physics_LargeVector &v );
	bool ElementMultiply( Physics_LargeVector &v, Physics_LargeVector &dst );
	bool ElementMultiply( matrix3 S[], Physics_LargeVector &dst );
	bool Scale( m_real scale, Physics_LargeVector &dst );
	bool Invert( Physics_LargeVector &dst );
	
	void Dump( char *szTitle = NULL );

	

};

#endif // !defined(AFX_PHYSICS_LARGEVECTOR_H__FDF65582_D36F_4061_A343_BDDA44E2DFDE__INCLUDED_)
