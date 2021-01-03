/** \file
    \author Taesoo  Kwon
*/

#ifndef TSOFTBODY_LCP_SOLVER_H_INCLUDED
#define TSOFTBODY_LCP_SOLVER_H_INCLUDED


#include "PhysicsLib/OpenHRPcommon.h"
#include "TRL/DynamicsSimulator_TRL_massSpring.h"
#include "PhysicsLib/TRL/ContactForceSolver.h"
namespace TRL
{
	
	class MCPsolver;
	
    class SoftBodyLCP : public MCPsolver
    {
		std::vector<int> frictionIndexToContactIndex;
    public:
		
		DynamicsSimulator_TRL_massSpring& world;
        SoftBodyLCP(DynamicsSimulator_TRL_massSpring& _world);
        ~SoftBodyLCP();
		
        bool addCollisionCheckLinkPair
		(int bodyIndex1, GLink* link1, int bodyIndex2, GLink* link2, double muStatic, double muDynamic, double epsilon);

		void setGaussSeidelParameters(int maxNumIteration, int numInitialIteration, double maxRelError);

		void initialize(void);
        void solve(OpenHRP::CollisionSequence& corbaCollisionSequence);
		void clearExternalForces();
		struct ConstraintPoint {
			int nodeIndex; // currently unused. goes to getVelocity and getAcceleration functions. use as you want.
            int globalIndex;
			::vector3 point;
            ::vector3 _normalTowardInside0;
            inline ::vector3 normalTowardInside(int i) { if(i==0) return _normalTowardInside0; return -_normalTowardInside0; }
			::vector3 defaultAccel[2];
			double normalProjectionOfRelVelocityOn0;
			double depth; // position error in the case of a connection point

			double mu;
			::vector3 relVelocityOn0;
			int globalFrictionIndex;
			::vector3 _frictionVector0[2];
			inline ::vector3 frictionVector(int i, int j) { 
				if(j==0)
					return _frictionVector0[i]; 
				return -_frictionVector0[i]; 
			}
        };
		typedef std::vector<ConstraintPoint> ConstraintPointArray;

		std::vector<GBody*> &bodiesData;

		struct LinkPair {

			int index;
			bool isSameBodyPair;
			int bodyIndex[2];
			GBody* bodyData[2];
			GLink* linkData[2];
			ConstraintPointArray constraintPoints;

			double muStatic;
			double muDynamic;
			double epsilon;

			Body::LinkConnection* connection;

		};
		typedef std::vector<LinkPair> LinkPairArray;

		LinkPairArray collisionCheckLinkPairs;
		LinkPairArray connectedLinkPairs;

		std::vector<LinkPair*> constrainedLinkPairs;

		double kappa;
		double epsilon;
		double _R;

		// constant acceleration term when no external force is applied
		vectorn an0;
		vectorn at0;


		// contact force solution: normal forces at contact points
		vectorn solution;


		matrixn Mlcp;
		vectorn B;

		void setConstraintPoints(OpenHRP::CollisionSequence& collisions);
		void setContactConstraintPoints(LinkPair& linkPair, OpenHRP::CollisionPointSequence& collisionPoints);
		void setFrictionVectors(ConstraintPoint& constraintPoint);
		bool setConnectionConstraintPoints(LinkPair& linkPair);
        void putContactPoints();
		void initMatrices();
		void setDefaultAccelerationVector();
		void setAccelerationMatrix();
		/*
		void initABMForceElementsWithNoExtForce(BodyData& bodyData);
		void calcABMForceElementsWithTestForce(BodyData& bodyData, Link* linkToApplyForce, const ::vector3& f, const ::vector3& tau);
		void calcAccelsABM(BodyData& bodyData, int constraintIndex);
		*/

		void extractRelAccelsOfConstraintPoints
		(matrixn& Kxn, matrixn& Kxt, int testForceIndex, int constraintIndex);

		void extractRelAccelsFromLinkPairCase1
		(matrixn& Kxn, matrixn& Kxt, LinkPair& linkPair, int testForceIndex, int constraintIndex);
		void extractRelAccelsFromLinkPairCase2
		(matrixn& Kxn, matrixn& Kxt, LinkPair& linkPair, int iTestForce, int iDefault, int testForceIndex, int constraintIndex);
		void extractRelAccelsFromLinkPairCase3
		(matrixn& Kxn, matrixn& Kxt, LinkPair& linkPair, int testForceIndex, int constraintIndex);

		void clearSingularPointConstraintsOfClosedLoopConnections();
		
		void setConstantVectorAndMuBlock();
		void addConstraintForceToLinks();
		void addConstraintForceToLink(LinkPair* linkPair, int ipair);

	};
};


#endif

