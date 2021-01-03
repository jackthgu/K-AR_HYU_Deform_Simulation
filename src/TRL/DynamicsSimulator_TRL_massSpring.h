#pragma once
/** \file
	\author Taesoo Kwon
*/

#include "BaseLib/math/Metric.h"
//#include "../../BaseLib/motion/TransitionCost.h"
#include "BaseLib/motion/FullbodyIK.h"
#include "BaseLib/utility/scoped_ptr.h"
#include "BaseLib/math/Operator.h"
#include "MainLib/OgreFltk/Mesh.h"
#include "MainLib/OgreFltk/MotionPanel.h"
#include "../softbody2/Physics_LargeVector.h"
//#include "BMF_Api.h"
//#include "BunnyMesh.h"
//#include "TorusMesh.h"
#include <stdio.h> //printf debugging

#include "PhysicsLib/TRL/DynamicsSimulator_TRL_penalty.h"
#include "Terrain.h"

class Physics_ParticleSystem;
namespace TRL
{
	class GBody;
	class GLink {
		// A single rigidBody or a softBody
		public:
			GLink(GBody* _body, int _index) { body=_body; index=_index;}
			virtual ~GLink(){}
			int index;
			GBody* body;

			// reimplement at least the following functions!!!
			virtual void addTestForce(vector3 const& f, vector3 const& contactPoint,int nodeIndex){}
			virtual vector3 getVelocity(vector3 const& contactPoint, int nodeIndex){ return vector3(0,0,0);} // by default, static object is assumed.
			virtual vector3 getAcceleration(vector3 const& contactPoint, int nodeIndex){ return vector3(0,0,0); }
			virtual void addConstraintForce(vector3 const& contactPoint, int nodeIndex, vector3 const& f){}
			virtual bool isStatic(){ return true;}
			virtual bool hasCustomFricCoef() { return false;} // by default, use the friction coef specified in the linkpair
			virtual void getCustomFrictionCoef(vector3 const& contactPoint, int nodeIndex, bool isSlipping, double& mu){}
	};
	// 여러 종류의 Body를 모두 지원하는 simulator를 만드는 것이 목표임.
	// 현재, GArticulatedBody, GTerrainBody, GSoftBody (mass-spring) 구현.
	// 나중에 GFEMBody 를 추가하면 될 듯.
	// 현재 collision check가 GArticulatedBody끼리의 충돌, 
	// TerrainBody 와 SoftBody 사이의 충돌 두종류만 구현이 되어있고, 추후 모든 조합의 충돌을 구현해야함.
	class GBody {
		protected:
			virtual bool _isStatic() {return true;}

		public:
			enum { ArticulatedBody, SoftBody, SoftBodyFastContact, Terrain, Others};
			virtual int getType() { return Others;}
			std::vector<GLink*> linksData;
			GBody(){}
			virtual ~GBody(){}
			
			// the following three variables are used internally. you do not need to manually set them.
			bool isStatic;
			bool hasConstrainedLinks; 
			bool isTestForceBeingApplied;

			virtual void testForceFinished() { isTestForceBeingApplied=false;}
			int numLinks(){return linksData.size();}


			virtual void initialize(){
				hasConstrainedLinks=false;
				isTestForceBeingApplied=false;
				isStatic=_isStatic();
			}
			// reimplement at least the following functions!!!
			virtual void clearExternalForces(){}
			virtual void calcAccelsWithoutExtForce(){}
			virtual void renderme(){}
			// should work with or without testForce being applied
			virtual void calcAccels(){}
	};
	class GArticulatedBodyLink;
	class GArticulatedBody : public GBody {
		friend class GArticulatedBodyLink;
		TRL::Body* body;
		::vector3 dpf;
		::vector3 dptau;
		virtual bool _isStatic();
		public:
		GArticulatedBody(int ichara, TRL::Body* _body);
		virtual ~GArticulatedBody();

		virtual int getType() { return GBody::ArticulatedBody;}
		inline GArticulatedBodyLink& getlinksData(int j) { return (GArticulatedBodyLink&)(*linksData[j]);}
		virtual void calcAccelsWithoutExtForce();
		virtual void initialze();
		virtual void calcAccels();
		virtual void clearExternalForces();
	};
	class DynamicsSimulator_TRL_massSpring;
	class GSoftBody : public GBody , public GLink {
		std::shared_ptr<Ogre::SceneNode> mNode;
		DynamicsSimulator_TRL_massSpring& _sim;
		public:
		OBJloader::Mesh mSimulatedMesh;
		OBJloader::MeshToEntity* mSimulatedOgreMesh;

		void addExternalForce(intvectorn const& indices, vector3N const& forces);

		// GBody
		virtual void calcAccelsWithoutExtForce();
		virtual void initialze();
		virtual void calcAccels();
		virtual void clearExternalForces();
		virtual bool _isStatic(){ return false;}
		
		// GLink
		virtual void addTestForce(vector3 const& f, vector3 const& contactPoint,int nodeIndex);
		virtual vector3 getVelocity(vector3 const& contactPoint, int nodeIndex);
		virtual vector3 getAcceleration(vector3 const& contactPoint, int nodeIndex);
		virtual void addConstraintForce(vector3 const& contactPoint, int nodeIndex, vector3 const& f);
		virtual bool isStatic(){ return false;}

		virtual bool hasCustomFricCoef() { return true;} 
		virtual void getCustomFrictionCoef(vector3 const& contactPoint, int nodeIndex, bool isSlipping, double& mu);
		// GSoftBody
		virtual void testForceFinished() { 
			GBody::testForceFinished();
			clearExternalForces();
		}
		intmatrixn _links; // springs: [forceIndex, node_index0, node_index1, type]
		int numLinks() { return _links.rows();}
		enum {LINK_FORCE, LINK_N0, LINK_N1, LINK_TYPE};
		enum {Structural, Bending};
		Physics_ParticleSystem *m_pSystem;
		Physics_LargeVector m_internalForces;
		Physics_LargeVector m_b_f; // h* df/dx*v(0)+df/dx*y
		virtual int getType() { return GBody::SoftBody;}
		GSoftBody(DynamicsSimulator_TRL_massSpring& sim, OBJloader::Mesh const& targetMesh);
		GSoftBody(DynamicsSimulator_TRL_massSpring& sim, OBJloader::Mesh const& targetMesh, intmatrixn const& springs, vectorn const& stiffness );
		virtual ~GSoftBody(){}
		virtual void renderme();
		void setTotalMass(OBJloader::Mesh const& mesh, double mass,bool fromfaces);
		void setRestLength(vectorn const& length);
		void changeSoftBodyRestLength(OBJloader::Mesh const& targetMesh);
		void extractLinkLength(OBJloader::Mesh const& targetMesh, vectorn& length);
		vector3& nodePos(int i);
		vector3& nodeVel(int i);
		void appendLink(OBJloader::Mesh const& mesh, int index1, int index2, double stiffness, double _springDamp, int linkType);
		void _CreateFromTriMesh( OBJloader::Mesh const& mesh, OBJloader::EdgeConnectivity const& edges, double stiffness, double damp, int con2, double stiff2);
		void adjustSoftBodyParam(const char* _adjustWhat, const char* selectionFileName, double value);
		void adjustSoftBodyEdgeParam(const char* _adjustWhat, boolN const& selectedEdges, double value);
		void adjustSoftBodyVertexParam(const char* _adjustWhat, boolN const& selectedVertices, double value);
		int	generateBendingConstraints(OBJloader::Mesh const& mesh, int distance, double stiffness, double damp);
	};
	class GSoftBodyFastContact : public GSoftBody {
	public:
		struct ContactSet
		{
			intvectorn nodeIndices; // forces are distributed to these nodes,
			vectorn nodeWeights; // according to these weights.
			int iTargetNode; // velocity, acceleration is measured here.
		};
		std::vector<ContactSet> contactSets;
		GSoftBodyFastContact(DynamicsSimulator_TRL_massSpring& sim, OBJloader::Mesh const& targetMesh);
		GSoftBodyFastContact(DynamicsSimulator_TRL_massSpring& sim, OBJloader::Mesh const& targetMesh, intmatrixn const& springs, vectorn const& stiffness );
		virtual int getType() { return GBody::SoftBodyFastContact;}
		virtual void addTestForce(vector3 const& f, vector3 const& contactPoint,int contactSetIndex);
		virtual vector3 getVelocity(vector3 const& contactPoint, int contactSetIndex);
		virtual vector3 getAcceleration(vector3 const& contactPoint, int contactSetIndex);
		virtual void addConstraintForce(vector3 const& contactPoint, int contactSetIndex, vector3 const& f);
		virtual bool isStatic(){ return false;}
		virtual bool hasCustomFricCoef() { return false;} 
	};
	class GTerrainBody :public GBody {
		public:
		MeshLoader::Terrain* mTerrain;
		vector3 mTerrainPos;
		virtual int getType() { return GBody::Terrain;}
		GTerrainBody(DynamicsSimulator_TRL_massSpring& sim, const char* filename, vector3 const& pos, int sizeX, int sizeY, m_real width, m_real height, m_real heightMax);
		virtual ~GTerrainBody();
		double getTerrainHeight(vector3 const& pos);
	};

	class SoftBodyLCP;

class DynamicsSimulator_TRL_massSpring : public OpenHRP ::DynamicsSimulator_TRL_penalty
{
	double _MA;
	std::vector<GBody*> _bodies;
	void _addEmptyCharacter(const char* name);
	void _addBody(GBody* body);
	bool _hasTerrain;
	bool _usePenaltyMethod;
	intvectorn _softbodyIndices;
	TRL::SoftBodyLCP* _contactForceSolver;
	double _penaltyStiffness;
public:
	void setUsePenaltyMethod(bool use) { _usePenaltyMethod=use;}
	void setPenaltyStiffness(double stiffness) {_penaltyStiffness=stiffness;}
	bool usePenaltyMethod() const { return _usePenaltyMethod;}
	bool hasTerrain() const { return _hasTerrain;}
	GTerrainBody* getTerrain() const ;
	GSoftBody* getSoftBody() const ;
	int numBodies() { return _bodies.size();}
	inline std::vector<GBody*>& bodiesData() {return _bodies;}

	inline GBody* body(int bodyIndex){ return _bodies[bodyIndex];}
	GSoftBody* softbody(int bodyIndex);

	// softbody parameters (will be moved to GSoftBody later)
	void setParameter(const char* _what, double value);
	double _mass;
	double _stiffness;
	double _springDamp;
	double _kDF;
	double _kDFvelThr;
	int _bendingConstraints1;
	double _bendingStiffness;
	int _integrateMethod;
	bool _drawContactForce;
	bool _debugDrawWorld;
	bool _drawSimulatedMesh;

	virtual bool stepSimulation();

	DynamicsSimulator_TRL_massSpring();
	virtual ~DynamicsSimulator_TRL_massSpring();

	void init(double timeStep, OpenHRP::DynamicsSimulator::IntegrateMethod integrateOpt);
	virtual void initSimulation();
	
	// registerCharacter or SoftBody or Terrain
	virtual void _registerCharacter(const char *name, OpenHRP::CharacterInfo const& cinfo);
	void createSoftBody(OBJloader::Mesh const& targetMesh);
	void createSoftBody(OBJloader::Mesh const& targetMesh, intmatrixn const& springs, vectorn const& stiffness );
	void createTerrain(const char* filename, vector3 const& pos, int sizeX, int sizeY, m_real width, m_real height, m_real heightMax);

	void setParam_Epsilon_Kappa(double eps, double kap);
	void setParam_R_B_MA(double r, double b, double ma);
	void _registerCollisionCheckPair(int ibody1, int ibody2, int treeIndex1, int treeIndex2, vectorn const& param);
	void registerCollisionCheckPair( const char *charName1, const char *linkName1, const char *charName2, const char *linkName2, vectorn const& param);
	void registerAllCollisionCheckPairs(int ibody1, int ibody2, vectorn const& param);


	void renderme();

private:
};
}
