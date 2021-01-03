/** \file
	\brief Implementation of SoftBodyLCP class
	\author Taesoo Kwon
*/
#define ASSERT(x) 
#define RANGE_ASSERT(x) 

#include "PhysicsLib/physicsLib.h"
#include "BaseLib/utility/QPerformanceTimer.h"
#if 1
// to disable profiling, set 1 above.
#undef BEGIN_TIMER
#undef END_TIMER2
#define BEGIN_TIMER(x)
#define END_TIMER2(x)
#endif
#ifdef __WIN32__
#define NOMINMAX
#endif

#include "PhysicsLib/TRL/ContactForceSolver.h"
#include "SoftBodyLCP.h"

#include <limits>

#include "PhysicsLib/OpenHRPcommon.h"
#include "PhysicsLib/TRL/eigenSupport.h"
#include "PhysicsLib/TRL/Link.h"
#include "PhysicsLib/TRL/Body.h"


inline double norm2(const vector3& v) { return v.length();}

// settings

static const double VEL_THRESH_OF_DYNAMIC_FRICTION = 1.0e-4;

//static const bool ENABLE_STATIC_FRICTION = true;
//static const bool ONLY_STATIC_FRICTION_FORMULATION = (true );
static const bool IGNORE_CURRENT_VELOCITY_IN_STATIC_FRICTION = false;

static const bool USE_SMOOTHED_CONTACT_DYNAMICS = false;
static const bool ALLOW_SUBTLE_PENETRATION_FOR_STABILITY = false;
static const double ALLOWED_PENETRATION_DEPTH = 0.0001;
static const double NEGATIVE_VELOCITY_RATIO_FOR_ALLOWING_PENETRATION = 10.0; 


using namespace OpenHRP;
using namespace TRL;
using namespace std;

#define PI 3.14159265358979323846
#define PI_2 1.57079632679489661923
//#define VERBOSE
namespace TRL
{

#ifdef __WIN32__
	const double CFSImpl::PI   = 3.14159265358979323846;
	const double CFSImpl::PI_2 = 1.57079632679489661923;
#endif
};


bool SoftBodyLCP::addCollisionCheckLinkPair
(int bodyIndex1, GLink* link1, int bodyIndex2, GLink* link2, double muStatic, double muDynamic, double epsilon)
{
	int n = collisionCheckLinkPairs.size();
	int index=n;
	collisionCheckLinkPairs.resize(index+1);

	LinkPair& linkPair = collisionCheckLinkPairs[index];

	linkPair.isSameBodyPair = (bodyIndex1 == bodyIndex2);
	linkPair.bodyIndex[0] = bodyIndex1;
	linkPair.linkData[0] = link1;
	linkPair.bodyIndex[1] = bodyIndex2;
	linkPair.linkData[1] = link2;
	linkPair.index = index;
	linkPair.muStatic = muStatic;
	linkPair.muDynamic = muDynamic;
	linkPair.epsilon = epsilon;
	linkPair.connection = 0;

    return (index >= 0 );
}



void SoftBodyLCP::initialize(void)
{

	int numBodies = world.numBodies();


	connectedLinkPairs.clear();

	for(int bodyIndex=0; bodyIndex < numBodies; ++bodyIndex){

		TRL::GBody* body = world.body(bodyIndex);
		body->initialize();
		body->clearExternalForces();
	}

	int numLinkPairs = collisionCheckLinkPairs.size();
    for(int i=0; i < numLinkPairs; ++i){
        LinkPair& linkPair = collisionCheckLinkPairs[i];
		for(int j=0; j < 2; ++j){
			GBody* bodyData = bodiesData[linkPair.bodyIndex[j]];
			linkPair.bodyData[j] = bodyData;
		}
	}

	prevGlobalNumConstraintVectors = 0;
	prevGlobalNumFrictionVectors = 0;
    numUnconverged = 0;

}


void SoftBodyLCP::clearExternalForces()
{
    for(size_t i=0; i < bodiesData.size(); ++i){
		GBody* bodyData = bodiesData[i];
		if(bodyData->hasConstrainedLinks){
			bodyData->clearExternalForces();
		}
    }
}


void SoftBodyLCP::solve(CollisionSequence& corbaCollisionSequence)
{
    for(size_t i=0; i < bodiesData.size(); ++i){
		bodiesData[i]->hasConstrainedLinks = false;
    }

	globalNumConstraintVectors = 0;
	globalNumFrictionVectors = 0;
	constrainedLinkPairs.clear();

	setConstraintPoints(corbaCollisionSequence);


	if(globalNumConstraintVectors > 0){


		const bool constraintsSizeChanged = ((globalNumFrictionVectors   != prevGlobalNumFrictionVectors) ||
											 (globalNumConstraintVectors != prevGlobalNumConstraintVectors));

		if(constraintsSizeChanged){
			initMatrices();
		}



	    setDefaultAccelerationVector();
	    setAccelerationMatrix();

		if(globalNumConstraintVectors - globalNumContactNormalVectors > 0){
			clearSingularPointConstraintsOfClosedLoopConnections();
		}
		
		setConstantVectorAndMuBlock();
#ifdef VERBOSE
		cout<<"mlcp"<<Mlcp<<endl<<"B"<<B<<endl;
#endif

#ifdef VERBOSE
		cout<<"mlcp"<<Mlcp<<endl<<"B"<<B<<endl;
#endif

		bool isConverged;
		solution.setAllValue(0.0);
		// taesoo's Implementation of the smoothed contact dynamics paper
		// M ddq + c = Jt f
		// ddq + invM *c =  invM*Jt f
		//  J ddq + J *invM *c = J*invM*Jt   f
		// so 
		//  A == Mlcp = JinvMJ'  
		// Mlcp*f = b                   ----(5)
		
		// smoothed dynamics version:
		// (M+MA) dv + c = J'f + u, 
		// and (A+R) *f = (v_star - v_)/h  --- (6)
		//
		//  then, 
		//  dv = inv(M+MA)*(J'f + u-c)    --- (1)
		//
		//  let v' be v + dv*h. Then,
		//  
		//  v'= v + inv(M+MA)*(J'f + u-c)*h --- (2)
		//   
		//  let v^ be v+inv(M+MA) *(u-c)*h. Then
		//
		//  v' = v^ + inv(M+MA)*J'f*h        --- (3)
		//
		//  By multiplying J on both sides of (3)
		//
		//  v2 = v_ + Af *h,                  --- (4)
		//  where v2=Jv' and v_ = Jv^.
		//
		// from (4), (5) and the fact that v'=0,
		//    b = (-v_)/h
		//
		// to modify (5) to  (6),
		//
		//    b should change to   b - (-v_)/h +(v_star - v_)/h
		//                        = b + v_star/h
		// 
		//
		//  
		//	desired contact velocity v*:
		//		kp =   (1+epsilon) / (kappa*kappa);
		//		kd = 2*(1+epsilon) / kappa;
		//  	v* : (normal*(vn + h*(kp * (-0 + depth) - kd * vn)));
		//
		// A*F = v_star - Jv^ 
		// , where impulse F is f*dt
		//
		// or A * f = (v_star- v_)/dt   --- (2)
		//
		assert(Mlcp.rows()==Mlcp.cols());
		Mlcp.diag()+=_R;

		BEGIN_TIMER(mcp);
		solveMCPByProjectedGaussSeidel(Mlcp, B, solution);
		END_TIMER2(mcp);
		isConverged = true;

		if(!isConverged){
			++numUnconverged;
#ifdef VERBOSE
				std::cout << "LCP didn't converge" << numUnconverged << std::endl;
#endif
		} else {

			addConstraintForceToLinks();
		}
	}

	prevGlobalNumConstraintVectors = globalNumConstraintVectors;
	prevGlobalNumFrictionVectors = globalNumFrictionVectors;
}


void SoftBodyLCP::setConstraintPoints(CollisionSequence& collisions)
{
    for(size_t i=0; i < collisionCheckLinkPairs.size(); ++i){

        LinkPair& linkPair = collisionCheckLinkPairs[i];
		CollisionPointSequence& points = collisions[i].points;

		if(points.size() > 0){
			constrainedLinkPairs.push_back(&linkPair);
			setContactConstraintPoints(linkPair, points);
			linkPair.bodyData[0]->hasConstrainedLinks = true;
			linkPair.bodyData[1]->hasConstrainedLinks = true;
		}
    }
	globalNumContactNormalVectors = globalNumConstraintVectors;

	for(size_t i=0; i < connectedLinkPairs.size(); ++i){
        LinkPair& linkPair = connectedLinkPairs[i];
		constrainedLinkPairs.push_back(&linkPair);
		if(setConnectionConstraintPoints(linkPair)){
			linkPair.bodyData[0]->hasConstrainedLinks = true;
			linkPair.bodyData[1]->hasConstrainedLinks = true;
		} else {
			constrainedLinkPairs.pop_back();
		}
    }
	globalNumConnectionVectors = globalNumConstraintVectors - globalNumContactNormalVectors;
}


void SoftBodyLCP::setContactConstraintPoints(LinkPair& linkPair, CollisionPointSequence& collisionPoints)
{
	ConstraintPointArray& constraintPoints = linkPair.constraintPoints;
	constraintPoints.clear();
	int numExtractedPoints = 0;
	int numContactsInPair = collisionPoints.size();

	for(int j=0; j < numContactsInPair; ++j){

		CollisionPoint& collision = collisionPoints[j];
		constraintPoints.push_back(ConstraintPoint());
		ConstraintPoint& contact = constraintPoints.back();

		contact.point= collision.position;
		contact._normalTowardInside0=-collision.normal;
		contact.depth = collision.idepth;
		contact.nodeIndex=collision.inode;

		bool isNeighborhood = false;


		if(isNeighborhood){
			constraintPoints.pop_back();
		} else {
			numExtractedPoints++;
			contact.globalIndex = globalNumConstraintVectors++;

			// check velocities
			vector3 v[2];
			for(int k=0; k < 2; ++k){
				GLink* link = linkPair.linkData[k];
				v[k]=link->getVelocity(contact.point, contact.nodeIndex);
#ifdef VERBOSE
				if (k==0)
				{
					std::cout << "Position " << contact.nodeIndex<<": p = " << contact.point << std::endl;
					std::cout << "Velocity " << contact.nodeIndex<<": v = " << v[k] << std::endl;
				}
#endif
			}
			contact.relVelocityOn0 = v[1] - v[0];
			contact.normalProjectionOfRelVelocityOn0 = dot(contact.normalTowardInside(1), contact.relVelocityOn0);

			vector3 v_tangent(contact.relVelocityOn0 - contact.normalProjectionOfRelVelocityOn0 * contact.normalTowardInside(1));

			contact.globalFrictionIndex = globalNumFrictionVectors;

			double vt_square = dot(v_tangent, v_tangent);
			static const double vsqrthresh = VEL_THRESH_OF_DYNAMIC_FRICTION * VEL_THRESH_OF_DYNAMIC_FRICTION;
			bool isSlipping = (vt_square > vsqrthresh);
			contact.mu = isSlipping ? linkPair.muDynamic : linkPair.muStatic;
			for(int k=0; k<2; ++k){
				GLink* link = linkPair.linkData[k];
				double muDynamic;
				double muStatic;
				if(link->hasCustomFricCoef())
					link->getCustomFrictionCoef(contact.point, contact.nodeIndex, isSlipping, contact.mu);
			}
			setFrictionVectors(contact);
			globalNumFrictionVectors += 2;
		}
	}
}


void SoftBodyLCP::setFrictionVectors(ConstraintPoint& contact)
{
	vector3 u(0.0);
	int minAxis = 0;
	vector3 normal = contact.normalTowardInside(0);

	for(int i=1; i < 3; i++){
		if(fabs(normal[i]) < fabs(normal[minAxis])){
			minAxis = i;
		}
	}
	u[minAxis] = 1.0;

	vector3 t1(cross(normal, u));
	t1 /= norm2(t1);

#ifdef TWO_VECTORS
	contact._frictionVector0[0] = t1;
	contact._frictionVector0[1] =cross(normal, t1);
#else
	contact._frictionVector0 = t1;
#endif
}


bool SoftBodyLCP::setConnectionConstraintPoints(LinkPair& linkPair)
{
	ConstraintPointArray& constraintPoints = linkPair.constraintPoints;

	Body::LinkConnection* connection = linkPair.connection;

	Link* link0 = connection->link[0];
	Link* link1 = connection->link[1];

	vector3 point[2];
	point[0] = link0->p + link0->R * connection->point[0];
	point[1] = link1->p + link1->R * connection->point[1];
	vector3 midPoint((point[0] + point[1]) / 2.0);
	vector3 error(midPoint - point[0]);

	if(dot(error, error) > (0.04 * 0.04)){
		return false;
	}

	// check velocities
	vector3 v[2];
	for(int k=0; k < 2; ++k){
		Link* link = connection->link[k];
		if(link->jointType == Link::FIXED_JOINT){
			v[k] = 0.0;
		} else {
			v[k] = link->vo + cross(link->w, point[k]);
		}
	}
	vector3 relVelocityOn0(v[1] - v[0]);

	for(int i=0; i < connection->numConstraintAxes; ++i){
		ConstraintPoint& constraint = constraintPoints[i];
		constraint.point = midPoint;
		const vector3 axis(link0->R * connection->constraintAxes[i]);
		constraint._normalTowardInside0 =  axis;
		constraint.depth = dot(axis, error);
		constraint.globalIndex = globalNumConstraintVectors++;
		constraint.normalProjectionOfRelVelocityOn0 = dot(constraint.normalTowardInside(1), relVelocityOn0);
	}

	return true;
}





void SoftBodyLCP::initMatrices()
{
	const int n = globalNumConstraintVectors;
	const int m = globalNumFrictionVectors;

	const int dimLCP = n + m;

	Mlcp.setSize(dimLCP, dimLCP);
	B.setSize(dimLCP);
	solution.setSize(dimLCP);


	{
		frictionIndexToContactIndex.resize(m);
	}

	an0.setSize(n);
	at0.setSize(m);

	MCPsolver::initWorkspace();
}

void SoftBodyLCP::setDefaultAccelerationVector()
{
	// calculate accelerations with no constraint force
	for(size_t i=0; i < bodiesData.size(); ++i){
		GBody& bodyData = *bodiesData[i];
		if(bodyData.hasConstrainedLinks && ! bodyData.isStatic){
			bodyData.calcAccelsWithoutExtForce();
		}
	}
	// clear skip check numbers
	/*
	if(SKIP_REDUNDANT_ACCEL_CALC ){
		for(size_t i=0; i < bodiesData.size(); ++i){
			auto* bodyData = bodiesData[i];
			if(bodyData->hasConstrainedLinks){
				for(size_t j=0; j < bodyData->linksData.size(); ++j){
					((GArticulatedBody*)bodyData)->getlinksData(j).numberToCheckAccelCalcSkip = std::numeric_limits<int>::max();
				}
			}
		}
		// add the number of contact points to skip check numbers of the links from a contact target to the root
		int numLinkPairs = constrainedLinkPairs.size();
		for(int i=0; i < numLinkPairs; ++i){
			LinkPair* linkPair = constrainedLinkPairs[i];
			int constraintIndex = linkPair->constraintPoints.front().globalIndex;
			for(int j=0; j < 2; ++j){
				LinkDataArray& linksData = linkPair->bodyData[j]->linksData;
				int linkIndex = linkPair->link[j]->index;
				while(linkIndex >= 0){
					LinkData& linkData = linksData[linkIndex];
					if(linkData.numberToCheckAccelCalcSkip < constraintIndex){
						break;
					}
					linkData.numberToCheckAccelCalcSkip = constraintIndex;
					linkIndex = linkData.parentIndex;
				}
			}
		}
	}
	*/

	// extract accelerations
	for(size_t i=0; i < constrainedLinkPairs.size(); ++i){

		LinkPair& linkPair = *constrainedLinkPairs[i];
		ConstraintPointArray& constraintPoints = linkPair.constraintPoints;

		for(size_t j=0; j < constraintPoints.size(); ++j){
			ConstraintPoint& constraint = constraintPoints[j];

			for(int k=0; k < 2; ++k){
				if(linkPair.bodyData[k]->isStatic){
					constraint.defaultAccel[k] = 0.0;
				} else {
					GLink* link = linkPair.linkData[k];
					constraint.defaultAccel[k] =
						link->getAcceleration(constraint.point,constraint.nodeIndex);
#ifdef VERBOSE
				if (k==0)
				std::cout << "DefaultAcc " << constraint.nodeIndex<<": dv = " << constraint.defaultAccel[k] << std::endl;
#endif
				}
			}

			vector3 relDefaultAccel(constraint.defaultAccel[1] - constraint.defaultAccel[0]);
			an0[constraint.globalIndex] = dot(constraint.normalTowardInside(1), relDefaultAccel);
#ifdef VERBOSE
			cout <<"normal"<< constraint.normalTowardInside(1)<<":"<< an0[constraint.globalIndex]<<std::endl;
#endif

#ifdef VERBOSE
			std::cout <<"normal "<<constraint.normalTowardInside(1) <<std::endl;
#endif
			for(int k=0; k < 2; ++k){
				at0[constraint.globalFrictionIndex + k] = dot(constraint.frictionVector(k,1), relDefaultAccel);
#ifdef VERBOSE
			cout <<"fricnormal"<< constraint.frictionVector(k,1)<<":"<< at0[constraint.globalIndex]<<std::endl;
#endif
			}
		}
	}
}


void SoftBodyLCP::setAccelerationMatrix()
{
	const int n = globalNumConstraintVectors;
	const int m = globalNumFrictionVectors;

	matrixnView Knn=Mlcp.range( 0,n, 0,n);
	matrixnView Ktn=Mlcp.range( 0,n, n,n+m);
	matrixnView Knt=Mlcp.range( n,n+m, 0,n);
	matrixnView Ktt=Mlcp.range( n,n+m, n,n+m);

	for(size_t i=0; i < constrainedLinkPairs.size(); ++i){

		LinkPair& linkPair = *constrainedLinkPairs[i];
		int numConstraintsInPair = linkPair.constraintPoints.size();

		for(int j=0; j < numConstraintsInPair; ++j){

			ConstraintPoint& constraint = linkPair.constraintPoints[j];
			int constraintIndex = constraint.globalIndex;

			// apply test normal force
			for(int k=0; k < 2; ++k){
				GBody& bodyData = *linkPair.bodyData[k];
				if(!bodyData.isStatic){

					bodyData.isTestForceBeingApplied = true;
					const vector3& f = constraint.normalTowardInside(k);

					{
#ifdef VERBOSE
						cout <<"testForce " << f <<endl;
#endif
						linkPair.linkData[k]->addTestForce( f, constraint.point, constraint.nodeIndex);
						if(!linkPair.isSameBodyPair || (k > 0)){
							bodyData.calcAccels();
#ifdef VERBOSE

							vector3 dv0;
							dv0=linkPair.linkData[k]->getAcceleration(constraint.point, constraint.nodeIndex);
							cout <<"testrep "<< dv0<<endl;
							// test reproducibility
							linkPair.linkData[k]->addTestForce( vector3(0,0,0), constraint.point, constraint.nodeIndex);
							bodyData.calcAccels();
							dv0=linkPair.linkData[k]->getAcceleration(constraint.point, constraint.nodeIndex);
							cout <<"testrep2 "<< dv0<<endl;
							linkPair.linkData[k]->addTestForce( f, constraint.point, constraint.nodeIndex);
							bodyData.calcAccels();
							dv0=linkPair.linkData[k]->getAcceleration(constraint.point, constraint.nodeIndex);
							cout <<"testrep3 "<< dv0<<endl;
#endif
						}


					}
				}
			}
#ifdef VERBOSE
		GBody& bodyData0 = *linkPair.bodyData[0];
		GBody& bodyData1 = *linkPair.bodyData[1];

		cout << "isTestForceBeingApplied"<< bodyData0.isTestForceBeingApplied <<", "<<bodyData1.isTestForceBeingApplied<<endl;
#endif
		   	extractRelAccelsOfConstraintPoints(Knn, Knt, constraintIndex, constraintIndex);

			// apply test friction force
			for(int l=0; l < 2; ++l){
				for(int k=0; k < 2; ++k){
					GBody& bodyData = *linkPair.bodyData[k];
					if(!bodyData.isStatic){
						const vector3& f = constraint.frictionVector(l,k);

						{
#ifdef VERBOSE
						cout <<"testForce " << f <<endl;
#endif
							linkPair.linkData[k]->addTestForce( f, constraint.point, constraint.nodeIndex);
							if(!linkPair.isSameBodyPair || (k > 0)){
								bodyData.calcAccels();
							}
						}
					}
				}
				extractRelAccelsOfConstraintPoints(Ktn, Ktt, constraint.globalFrictionIndex + l, constraintIndex);
			}

			linkPair.bodyData[0]->testForceFinished();
			linkPair.bodyData[1]->testForceFinished();
		}
	}



}




void SoftBodyLCP::extractRelAccelsOfConstraintPoints
(matrixn& Kxn, matrixn& Kxt, int testForceIndex, int constraintIndex)
{
	int maxConstraintIndexToExtract =  globalNumConstraintVectors;

	for(size_t i=0; i < constrainedLinkPairs.size(); ++i){

		LinkPair& linkPair = *constrainedLinkPairs[i];

		GBody& bodyData0 = *linkPair.bodyData[0];
		GBody& bodyData1 = *linkPair.bodyData[1];

		if(bodyData0.isTestForceBeingApplied){
			if(bodyData1.isTestForceBeingApplied){
				extractRelAccelsFromLinkPairCase1(Kxn, Kxt, linkPair, testForceIndex, maxConstraintIndexToExtract);
			} else {
				extractRelAccelsFromLinkPairCase2(Kxn, Kxt, linkPair, 0, 1, testForceIndex, maxConstraintIndexToExtract);
			}
		} else {
			if(bodyData1.isTestForceBeingApplied){
				extractRelAccelsFromLinkPairCase2(Kxn, Kxt, linkPair, 1, 0, testForceIndex, maxConstraintIndexToExtract);
			} else {
				extractRelAccelsFromLinkPairCase3(Kxn, Kxt, linkPair, testForceIndex, maxConstraintIndexToExtract);
			}
		}
	}
}


void SoftBodyLCP::extractRelAccelsFromLinkPairCase1
(matrixn& Kxn, matrixn& Kxt, LinkPair& linkPair, int testForceIndex, int maxConstraintIndexToExtract)
{
	ConstraintPointArray& constraintPoints = linkPair.constraintPoints;

	for(size_t i=0; i < constraintPoints.size(); ++i){

		ConstraintPoint& constraint = constraintPoints[i];
		int constraintIndex = constraint.globalIndex;


		GLink* linkData0 = linkPair.linkData[0];
		GLink* linkData1 = linkPair.linkData[1];

		vector3 dv0(linkData0->getAcceleration(constraint.point, constraint.nodeIndex));
		vector3 dv1(linkData1->getAcceleration(constraint.point, constraint.nodeIndex));

		vector3 relAccel(dv1 - dv0);

		Kxn(constraintIndex, testForceIndex) =
			dot(constraint.normalTowardInside(1), relAccel) - an0(constraintIndex);

		for(int j=0; j < 2; ++j){
			const int index = constraint.globalFrictionIndex + j;
			Kxt(index, testForceIndex) = dot(constraint.frictionVector(j,1), relAccel) - at0(index);
		}
	}
}


void SoftBodyLCP::extractRelAccelsFromLinkPairCase2
(matrixn& Kxn, matrixn& Kxt, LinkPair& linkPair, int iTestForce, int iDefault, int testForceIndex, int maxConstraintIndexToExtract)
{
	ConstraintPointArray& constraintPoints = linkPair.constraintPoints;

	for(size_t i=0; i < constraintPoints.size(); ++i){

		ConstraintPoint& constraint = constraintPoints[i];
		int constraintIndex = constraint.globalIndex;


		GLink* linkData = linkPair.linkData[iTestForce];


		vector3 dv(linkData->getAcceleration(constraint.point, constraint.nodeIndex));

		vector3 relAccel(constraint.defaultAccel[iDefault] - dv);

#ifdef VERBOSE
		cout << "relaccel:"<<constraint.defaultAccel[iDefault]<<","<<dv<<" an0:" <<an0(constraintIndex)<< endl;
#endif
		Kxn(constraintIndex, testForceIndex) =
			dot(constraint.normalTowardInside(iDefault), relAccel) - an0(constraintIndex);

		for(int j=0; j < 2; ++j){
			const int index = constraint.globalFrictionIndex + j;
			Kxt(index, testForceIndex) =
				dot(constraint.frictionVector(j,iDefault), relAccel) - at0(index);
		}

	}
}


void SoftBodyLCP::extractRelAccelsFromLinkPairCase3
(matrixn& Kxn, matrixn& Kxt, LinkPair& linkPair, int testForceIndex, int maxConstraintIndexToExtract)
{
	ConstraintPointArray& constraintPoints = linkPair.constraintPoints;

	for(size_t i=0; i < constraintPoints.size(); ++i){

		ConstraintPoint& constraint = constraintPoints[i];
		int constraintIndex = constraint.globalIndex;

		Kxn(constraintIndex, testForceIndex) = 0.0;

		for(int j=0; j < 2; ++j){
			Kxt(constraint.globalFrictionIndex + j, testForceIndex) = 0.0;
		}
	}
}



void SoftBodyLCP::clearSingularPointConstraintsOfClosedLoopConnections()
{
	for(int i = globalNumContactNormalVectors; i < globalNumConstraintVectors; ++i){
		if(Mlcp(i, i) < 1.0e-4){
			for(size_t j=0; j < Mlcp.rows(); ++j){
				Mlcp(j, i) = 0.0;
			}
			Mlcp(i, i) = numeric_limits<double>::max();
		}
	}
}


void SoftBodyLCP::setConstantVectorAndMuBlock()
{
	double dtinv = 1.0 / world.getTimestep();
	const int block2 = globalNumConstraintVectors;
	const int block3 = globalNumConstraintVectors + globalNumFrictionVectors;

	for(size_t i=0; i < constrainedLinkPairs.size(); ++i){

		LinkPair& linkPair = *constrainedLinkPairs[i];
		int numConstraintsInPair = linkPair.constraintPoints.size();

		for(int j=0; j < numConstraintsInPair; ++j){
			ConstraintPoint& constraint = linkPair.constraintPoints[j];
			int globalIndex = constraint.globalIndex;

			// set constant vector of LCP

			// constraints for normal acceleration

			if(linkPair.connection){
				// connection constraint

				const double& error = constraint.depth;
				double v;
				if(error >= 0){
					v = 0.1 * (-1.0 + exp(-error * 20.0));
				} else {
					v = 0.1 * ( 1.0 - exp( error * 20.0));
				}
					
				B(globalIndex) = an0(globalIndex) + (constraint.normalProjectionOfRelVelocityOn0 + v) * dtinv;

			} else {
				// contact constraint

				double v_star=0.0;
				if(USE_SMOOTHED_CONTACT_DYNAMICS)
				{
					double kp =   (1+epsilon) / (kappa*kappa);
					double kd = 2*(1+epsilon) / kappa;
					double h=world.getTimestep();
					double vn=constraint.normalProjectionOfRelVelocityOn0;
					v_star= vn + h*(kp * (-0 + constraint.depth) - kd * vn);

					//printf("2: %f %f %f %f\n", vn, constraint.depth, h, v_star);
				}

				if(ALLOW_SUBTLE_PENETRATION_FOR_STABILITY && constraint.depth < ALLOWED_PENETRATION_DEPTH){

					double extraNegativeVel = (ALLOWED_PENETRATION_DEPTH - constraint.depth) * NEGATIVE_VELOCITY_RATIO_FOR_ALLOWING_PENETRATION;
					B(globalIndex) = an0(globalIndex) + (constraint.normalProjectionOfRelVelocityOn0 + extraNegativeVel) * dtinv;
				} else {
					B(globalIndex) = an0(globalIndex) + constraint.normalProjectionOfRelVelocityOn0 * dtinv;
				}
#ifdef VERBOSE
				cout<<"an0 "<<an0(globalIndex) <<" depth " << constraint.depth << " dtinv " << dtinv<<endl;
				cout<<"B "<<globalIndex<<":vel: "<< constraint.normalProjectionOfRelVelocityOn0 << ":B: "<<B(globalIndex)<<":mu: "<<constraint.mu<<endl;
#endif
				// modify b to a smoothed dynamics version
				B(globalIndex)+= -v_star*dtinv;

				//printf("mu %f\n", constraint.mu);
				contactIndexToMu(globalIndex) = constraint.mu;

				int globalFrictionIndex = constraint.globalFrictionIndex;
				for(int k=0; k < 2; ++k){

					// constraints for tangent acceleration
					double tangentProjectionOfRelVelocity = dot(constraint.frictionVector(k,1), constraint.relVelocityOn0);

#ifdef VERBOSE
				cout<<"at0 "<<at0(globalFrictionIndex) <<endl;
				cout<<"B "<<globalFrictionIndex<< "fric "<<k<<":"<<constraint.frictionVector(k,1)<<":vel: "<< tangentProjectionOfRelVelocity <<endl;
#endif
					B(block2 + globalFrictionIndex) = at0(globalFrictionIndex);
					if( !IGNORE_CURRENT_VELOCITY_IN_STATIC_FRICTION ){
						B(block2 + globalFrictionIndex) += tangentProjectionOfRelVelocity * dtinv;
					}
#ifdef VERBOSE
				cout << "-> " << B(block2 + globalFrictionIndex) <<endl;
#endif
					// for iterative solver
					frictionIndexToContactIndex[globalFrictionIndex] = globalIndex;

					++globalFrictionIndex;
				}
			}
		}
	}
}


void SoftBodyLCP::addConstraintForceToLinks()
{
	clearExternalForces(); // articulated body의 경우 필요 없는데, 소프트바디에서 필요해서 넣어놈.
    int n = constrainedLinkPairs.size();
    for(int i=0; i < n; ++i){
		LinkPair* linkPair = constrainedLinkPairs[i];
		for(int j=0; j < 2; ++j){
			if(!linkPair->linkData[j]->isStatic()){
				addConstraintForceToLink(linkPair, j);
			}
		}
    }
}


void SoftBodyLCP::addConstraintForceToLink(LinkPair* linkPair, int ipair)
{
    //vector3 f_total(0.0);
    //vector3 tau_total(0.0);

    ConstraintPointArray& constraintPoints = linkPair->constraintPoints;
    int numConstraintPoints = constraintPoints.size();

    GLink* link = linkPair->linkData[ipair];
    for(int i=0; i < numConstraintPoints; ++i){

		ConstraintPoint& constraint = constraintPoints[i];
		int globalIndex = constraint.globalIndex;

		vector3 f(solution(globalIndex) * constraint.normalTowardInside(ipair));

		for(int j=0; j < 2; ++j){
			f += solution(globalNumConstraintVectors + constraint.globalFrictionIndex + j) * constraint.frictionVector(j,ipair);
		}

#ifdef VERBOSE
		std::cout << "Constraint force :  " << f << std::endl;
#endif
		link->addConstraintForce(constraint.point, constraint.nodeIndex, f); 
		//f_total   += f;
		//tau_total += cross(constraint.point, f);
    }

	//link->addConstraintForce(f_total, tau_total);


}





SoftBodyLCP::SoftBodyLCP(DynamicsSimulator_TRL_massSpring& _world)
	:
	world(_world),
	bodiesData(_world.bodiesData())
{
	// recommended for low-freq control:
	// kappa=0.05; epsilon=0.1;
	// rigid:
	kappa=0.01; epsilon=0.01;
	_R=1.0e-5;

	//setGaussSeidelParameters(500, 0, 1.0e-3);
}


SoftBodyLCP::~SoftBodyLCP()
{
}

















void SoftBodyLCP::setGaussSeidelParameters(int maxNumIteration, int numInitialIteration, double maxRelError)
{
	SoftBodyLCP	*impll=this;
	impll->maxNumGaussSeidelIteration = maxNumIteration;
	impll->numGaussSeidelInitialIteration = numInitialIteration;
	impll->gaussSeidelMaxRelError = maxRelError;
}
