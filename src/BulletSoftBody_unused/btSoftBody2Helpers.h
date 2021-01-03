/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2008 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef SOFT_BODY_HELPERS2_H
#define SOFT_BODY_HELPERS2_H

#include "btSoftBody2.h"
#include "btSoftBodyHelpers.h"

//
// Helpers
//
struct	btSoftBody2Helpers
{
	/* Draw body															*/ 
	static void				Draw(		btSoftBody2* psb,
										btIDebugDraw* idraw,
										int drawflags=fDrawFlags::Std);
	/* Draw body infos														*/ 
	static	void			DrawInfos(	btSoftBody2* psb,
										btIDebugDraw* idraw,
										bool masses,
										bool areas,
										bool stress);
	/* Draw node tree														*/ 
	static void				DrawNodeTree(	btSoftBody2* psb,
											btIDebugDraw* idraw,
											int mindepth=0,
											int maxdepth=-1);
	/* Draw face tree														*/ 
	static void				DrawFaceTree(	btSoftBody2* psb,
											btIDebugDraw* idraw,
											int mindepth=0,
											int maxdepth=-1);
	/* Draw rigid frame														*/ 
	static	void			DrawFrame(		btSoftBody2* psb,
											btIDebugDraw* idraw);
	/* Create a rope														*/ 
	static	btSoftBody2*		CreateRope(	 btSoftBody2::btSoftBody2WorldInfo& worldInfo,const btVector3& from,
							   const btVector3& to,
							   int res,
							   int fixeds);
	/* Create a patch														*/ 
	static	btSoftBody2*		CreatePatch(	btSoftBody2::btSoftBody2WorldInfo& worldInfo,const btVector3& corner00,
								const btVector3& corner10,
								const btVector3& corner01,
								const btVector3& corner11,
								int resx,
								int resy,
								int fixeds,
								bool gendiags);
	/* Create an ellipsoid													*/ 
	static	btSoftBody2*		CreateEllipsoid(btSoftBody2::btSoftBody2WorldInfo& worldInfo,const btVector3& center,
									const btVector3& radius,
									int res);
	/* Create from convex-hull												*/ 
	static	btSoftBody2*		CreateFromConvexHull(	btSoftBody2::btSoftBody2WorldInfo& worldInfo, const btVector3* vertices,
										 int nvertices);											
	/* Create from trimesh													*/ 
	static	btSoftBody2*		CreateFromTriMesh(	btSoftBody2::btSoftBody2WorldInfo& worldInfo, const btScalar*	vertices,
									  const int* triangles,
									  int ntriangles);
};

#endif //SOFT_BODY_HELPERS_H
