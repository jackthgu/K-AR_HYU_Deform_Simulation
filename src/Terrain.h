#ifndef TERRAIN_H
#define TERRAIN_H
#pragma once

#include "BaseLib/math/tvector.h"
#include "Mesh_old.h"

struct Region2D
{
    m_real left, top, right, bottom;

    Region2D()
    {
    }
    Region2D( long l, long t, long r, long b )
    {
        left = l;
        top = t;   
        right = r;
        bottom = b;                
    }

	m_real width() const				{ return right-left;}
	m_real height() const				{ return bottom-top;}
	bool isInside(vector2 pt)	const	{ if(pt.x()>=left && pt.y()>=top && pt.x()<right && pt.y()<bottom) return true; return false;}
};

namespace MeshLoader
{
	class Terrain : public Mesh
	{
		matrixn mHeight;
		matrixn mMaxHeight;
		vector3 mSize;
	public :
		Terrain(const char* filename, int imageSizeX, int imageSizeY, m_real sizeX, m_real sizeZ, m_real heightMax, int ntexSegX, int ntexSegZ);

		~Terrain(){}

		index2 _lowindex(vector2 x) const;
		index2 _highindex(vector2 x) const;
		m_real maxHeight(Region2D const & r) const;
		m_real maxHeight(vector2 x) const;
		m_real height(vector2 x, vector3& normal) const;
	};
}



class Raw2Bytes: public _tmat<unsigned short>
{
public:
	Raw2Bytes():_tmat<unsigned short>(){}
	template <class T>
	Raw2Bytes(const T& other):_tmat<unsigned short>(other){}
};

#endif
