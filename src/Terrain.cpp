#include "stdafx.h"
#include "Terrain.h"

index2 MeshLoader::Terrain::_lowindex(vector2 x) const
{
	index2 out;
	out(0)=int(floor((x(0)/mSize.x)*(mHeight.cols()-1)));
	out(1)=int(floor((x(1)/mSize.z)*(mHeight.rows()-1)));
	return out;
}

index2 MeshLoader::Terrain::_highindex(vector2 x) const
{
	index2 out=_lowindex(x);
	out(0)++;
	out(1)++;
	return out;
}
m_real MeshLoader::Terrain::maxHeight(Region2D const & r) const
{
	index2 low=_lowindex(vector2(r.left, r.top));
	index2 high=_highindex(vector2(r.bottom, r.right));

	if(high(0)<0) return -FLT_MAX;
	if(high(1)<0) return -FLT_MAX;
	if(low(0)>=mHeight.cols()) return -FLT_MAX;
	if(low(1)>=mHeight.rows()) return -FLT_MAX;

	low(0)=MIN(low(0), 0);
	low(1)=MIN(low(1), 0);

	high(0)=MAX(high(0), mHeight.cols()-1);
	high(1)=MAX(high(1), mHeight.rows()-1);

	m_real maxHeight=-FLT_MAX;
	for(int j=low(1); j<=high(1); j++)
	{
		for(int i=low(0); i<=high(0); i++)
		{
			if(mHeight(j,i)>maxHeight)
				maxHeight=mHeight(j,i);						
		}
	}

	return maxHeight;
}

m_real MeshLoader::Terrain::maxHeight(vector2 x) const
{
	index2 low=_lowindex(x);
	low(0)=MAX(0, low(0));
	low(1)=MAX(0, low(1));
	low(0)=MIN(low(0), mMaxHeight.cols()-1);
	low(1)=MIN(low(1), mMaxHeight.rows()-1);
	
	return mMaxHeight(low(1), low(0));
}

#include "MainLib/Ogre/intersectionTest.h"

m_real MeshLoader::Terrain::height(vector2 x, vector3& normal) const
{
	int numSegX=mHeight.cols()-1;
	int numSegZ=mHeight.rows()-1;
	
	double nx=(x(0)/mSize.x)*double(numSegX);
	double nz=(x(1)/mSize.z)*double(numSegZ);

	int j=int(floor(nx));   // x coord (left)
							// y coord (vertical)
	int i=int(floor(nz));	// z coord (down)

	double dx=nx-floor(nx);
	double dz=nz-floor(nz);

	if(j<0) 
	{
		j=0; dx=0;
	}
	if(i<0)
	{
		i=0; dz=0;
	}
	if(j>=numSegX)
	{
		j=numSegX-1; dx=1;
	}

	if(i>=numSegZ)
	{
		i=numSegZ-1; dz=1;
	}


	int faceIndex;
	if(dx>dz)
	{
		// upper triangle ( (j,i), (j+1, i+1), (j+1, i) )
		faceIndex=i*numSegX+j;
	}
	else
	{
		// lower triangle ( (j,i), (j, i+1), (j+1, i+1) )
		faceIndex=i*numSegX+j+numSegX*numSegZ;
	}

	const int* vi=getFace(faceIndex).vertexIndex;

	Plane p(getVertex(vi[0]).pos, getVertex(vi[1]).pos, getVertex(vi[2]).pos);

	Ray r(vector3(x(0), 1000, x(1)), vector3(0,-1,0));
	std::pair<bool, m_real> res=r.intersects(p);

	ASSERT(res.first);
	normal=p.normal;
	return r.getPoint(res.second).y;
}

MeshLoader::Terrain::Terrain(const char *filename, int sizeX, int sizeY, m_real width, m_real height, m_real heightMax, int ntexSegX, int ntexSegZ)
:Mesh()
{
	mSize.x=width;
	mSize.z=height;
	mSize.y=heightMax;

	ASSERT(sizeof(short)==2);
	Raw2Bytes image;
	image.setSize(sizeY, sizeX);
	FILE* file=fopen(filename, "rb");
	fread( (void*)(&image(0,0)), sizeof(short), sizeX*sizeY, file);
	
	fclose(file);

#ifdef BAD_NOTEBOOK
	// taesoo's notebook_only
	for(int i=0; i<64; i++)
	{
		for(int j=0; j<64; j++)
		image(i,j)=image(image.rows()-i-1, j);
	}
	sizeY=64;
	sizeX=64;
#endif

	mHeight.setSize(sizeY, sizeX);

	for(int y=0; y<sizeY; y++)
	{
		for(int x=0; x<sizeX; x++)
		{
			mHeight(y,x)=heightMax*m_real(image(y,x))/65536.0;
		}
	}


	Mesh& mesh=*this;

	mesh.m_arrayVertex.resize(sizeX*sizeY);

	for(int y=0; y<sizeY; y++)
	{
		for(int x=0; x<sizeX; x++)
		{
			int index=y*sizeX+x;
			
			m_real& tu=mesh.m_arrayVertex[index].texCoord(0);
			m_real& tv=mesh.m_arrayVertex[index].texCoord(1);

			tu=m_real(x)*m_real(ntexSegX)/m_real(sizeX-1);
			tv=m_real(y)*m_real(ntexSegZ)/m_real(sizeY-1);
			
			vector3& pos=mesh.m_arrayVertex[index].pos;
			pos.x=m_real(x)*width/m_real(sizeX-1);
			pos.z=m_real(y)*height/m_real(sizeY-1);
			pos.y=mHeight(y,x);
		}
	}

	int numSegX=sizeX-1;
	int numSegZ=sizeY-1;

	mMaxHeight.setSize(numSegZ, numSegX);
	
	for(int i=0; i<numSegZ; i++)
	{
		for(int j=0; j<numSegX; j++)
		{
			mMaxHeight[i][j]=std::max(
				std::max(mHeight[i][j], mHeight[i][j+1]), 
				std::max(mHeight[i+1][j],mHeight[i+1][j+1]));
		}
	}

	mesh.m_arrayFace.resize(numSegX*numSegZ*2);


	vector3N normalSum(mesh.numVertex());
	normalSum.setAllValue(vector3(0,0,0));

	intvectorn normalCount(mesh.numVertex());
	normalCount.setAllValue(0);

	for(int i=0; i<numSegZ; i++)
	{
		for(int j=0; j<numSegX; j++)
		{
			// insert lower triangle
			int faceIndex=i*(numSegX)+j;
			{
				Face& f=mesh.m_arrayFace[faceIndex];
				f.vertexIndex[0]=i*(numSegX+1)+j;
				f.vertexIndex[1]=(i+1)*(numSegX+1)+j+1;
				f.vertexIndex[2]=i*(numSegX+1)+j+1;

				vector3 v0=mesh.getVertex(f.vi(0)).pos;
				vector3 v1=mesh.getVertex(f.vi(1)).pos;
				vector3 v2=mesh.getVertex(f.vi(2)).pos;

				vector3 faceNormal;
				faceNormal.cross(v1-v0, v2-v0);
				faceNormal.normalize();

				for(int vv=0; vv<3; vv++)
				{
					normalSum[f.vi(vv)]+=faceNormal;
					normalCount[f.vi(vv)]++;
				}
			}

			// insert upper triangle
			faceIndex+=numSegX*numSegZ;
			Face& f=mesh.m_arrayFace[faceIndex];
			f.vertexIndex[0]=i*(numSegX+1)+j;
			f.vertexIndex[1]=(i+1)*(numSegX+1)+j;
			f.vertexIndex[2]=(i+1)*(numSegX+1)+(j+1);

			vector3 v0=mesh.getVertex(f.vi(0)).pos;
			vector3 v1=mesh.getVertex(f.vi(1)).pos;
			vector3 v2=mesh.getVertex(f.vi(2)).pos;

			vector3 faceNormal;
			faceNormal.cross(v1-v0, v2-v0);
			faceNormal.normalize();

			for(int vv=0; vv<3; vv++)
			{
				normalSum[f.vi(vv)]+=faceNormal;
				normalCount[f.vi(vv)]++;
			}
		}
	}

	for(int i=0; i<mesh.numVertex(); i++)
	{
		mesh.m_arrayVertex[i].normal=normalSum[i]/m_real(normalCount[i]);
		mesh.m_arrayVertex[i].normal.normalize();
	}

}

