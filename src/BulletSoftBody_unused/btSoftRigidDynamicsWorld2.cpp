/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/


#include "btSoftRigidDynamicsWorld2.h"
#include "LinearMath/btQuickprof.h"

//softbody & helpers
#include "btSoftBody2.h"
#include "btSoftBody2Helpers.h"

btSoftRigidDynamicsWorld2::btSoftRigidDynamicsWorld2(btDispatcher* dispatcher,btBroadphaseInterface* pairCache,btConstraintSolver* constraintSolver,btCollisionConfiguration* collisionConfiguration)
:btDiscreteDynamicsWorld(dispatcher,pairCache,constraintSolver,collisionConfiguration)
{

}
		
btSoftRigidDynamicsWorld2::~btSoftRigidDynamicsWorld2()
{

}

void	btSoftRigidDynamicsWorld2::predictUnconstraintMotion(btScalar timeStep)
{
	btDiscreteDynamicsWorld::predictUnconstraintMotion( timeStep);

	for ( int i=0;i<m_softBodies.size();++i)
	{
		btSoftBody2*	psb= m_softBodies[i];

		psb->predictMotion(timeStep);		
	}
}
		
void	btSoftRigidDynamicsWorld2::internalSingleStepSimulation( btScalar timeStep)
{
	btDiscreteDynamicsWorld::internalSingleStepSimulation( timeStep );

	///solve soft bodies constraints
	solveSoftBodiesConstraints();

	///update soft bodies
	updateSoftBodies();

}

void	btSoftRigidDynamicsWorld2::updateSoftBodies()
{
	BT_PROFILE("updateSoftBodies");
	
	for ( int i=0;i<m_softBodies.size();i++)
	{
		btSoftBody2*	psb=(btSoftBody2*)m_softBodies[i];
		psb->integrateMotion();	
	}
}

void	btSoftRigidDynamicsWorld2::solveSoftBodiesConstraints()
{
	BT_PROFILE("solveSoftConstraints");
		
	for(int i=0;i<m_softBodies.size();++i)
	{
		btSoftBody2*	psb=(btSoftBody2*)m_softBodies[i];
		psb->solveConstraints();
	}	
}

void	btSoftRigidDynamicsWorld2::addSoftBody(btSoftBody2* body)
{
	m_softBodies.push_back(body);

	btCollisionWorld::addCollisionObject(body, btBroadphaseProxy::DefaultFilter, btBroadphaseProxy::AllFilter);
}

void	btSoftRigidDynamicsWorld2::removeSoftBody(btSoftBody2* body)
{
	m_softBodies.remove(body);

	btCollisionWorld::removeCollisionObject(body);
}

void	btSoftRigidDynamicsWorld2::debugDrawWorld()
{
	btDiscreteDynamicsWorld::debugDrawWorld();

	if (getDebugDrawer())
	{
		int i;
		for (  i=0;i<this->m_softBodies.size();i++)
		{
			btSoftBody2*	psb=(btSoftBody2*)this->m_softBodies[i];
			btSoftBody2Helpers::DrawFrame(psb,m_debugDrawer);
			
			//btSoftBody2Helpers::Draw(psb,m_debugDrawer,fDrawFlags::Std);
			//btSoftBody2Helpers::Draw(psb,m_debugDrawer,fDrawFlags::Links);
			btSoftBody2Helpers::Draw(psb,m_debugDrawer,fDrawFlags::SLinks);
			if (m_debugDrawer && (m_debugDrawer->getDebugMode() & btIDebugDraw::DBG_DrawAabb))
			{
				btSoftBody2Helpers::DrawNodeTree(psb,m_debugDrawer);
				//btSoftBody2Helpers::DrawFaceTree(psb,m_debugDrawer);
			}
		}
	}
}
