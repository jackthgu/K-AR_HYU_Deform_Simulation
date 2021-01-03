gen_lua.enum_types[#gen_lua.enum_types+1]='OpenHRP::DynamicsSimulator::IntegrateMethod'

bindTargetClassification={
	classes={
		{
			decl='class LinearMeshDeformer;',
			name='LinearMeshDeformer',
		},
		{
			decl='class LinearDeformer;',
			inheritsFrom='LinearMeshDeformer',
			include='#include "ElasticBody/MeanValueWeights.h"',
			name='LinearDeformer',
			memberFunctions=[[
			void calculateCorrespondence(OBJloader::Mesh & control, vector3N& embedded);
			bool loadCorrespondence(const char* filename, OBJloader::Mesh & control, vector3N& embedded);
			void transfer(OBJloader::Mesh const& control, double scale_factor);
			]],
			staticMemberFunctions=[[
			LinearDeformer* createMeanValueDeformer();
			LinearDeformer* createHarmonicDeformer();
			LinearDeformer* createKNNIDWDeformer(int m_nK, float m_fNW);
			LinearDeformer* createKNNIDWDeformer(int m_nK, float m_fK, float m_fNW);
			]]
		},
		{
			decl='class InverseDeformer;',
			name='InverseDeformer',
			ctors={'(LinearMeshDeformer* deformer, vector3N const&)'},
			memberFunctions=[[
			void invDeform(vector3N const& embedded, OBJloader::Mesh & orig_control, OBJloader::Mesh & control);
			]]
		},
		{
			ifdef='USE_BULLETDEP',
			name='btRigidBody',
			staticMemberFunctions=[[
			btRigidBody* createTerrain(const char* filename, vector3 const& pos, int sizeX, int sizeY, m_real width, m_real height, m_real heightMax);
			void getTerrainHeight(btRigidBody* terrain, vector3 const& in, vector3 & out, vector3& normal)
			]]
		},
		{
			ifdef='USE_BULLETDEP',
			name='IguanaIKSolver',
			ctors={'(MotionLoader& skel, btRigidBody* terrain)'},
			memberFunctions=[[
			void getAffectedDOF(intvectorn & rot_joint_index, intvectorn & trans_joint_index);
			void FullbodyIK::IKsolve(Posture& poseInOut, vector3N const& conpos);
			]]
		},
		{
			ifdef='USE_BULLETDEP',
			name='FlexibleBodyImplicitSimpleWin',
			wrapperCode=[[
			static OBJloader::Mesh& targetMesh(FlexibleBodyImplicitSimpleWin& win) { return win.mTargetMesh;}
			]],
			memberFunctions={[[
			void createTargetMesh(int iframe, double scale_factor, vector3 const& initialPosition);
			]]},
			staticMemberFunctions={[[
			static OBJloader::Mesh& targetMesh(FlexibleBodyImplicitSimpleWin& win) { return win.mTargetMesh;}
			]]}
		},
		{
			ifdef='USE_BULLETDEP',
			name='FlexibleBodyImplicitSimplePenaltyWin',
			properties=[[
			Motion* mMotion;
			OBJloader::Mesh mTargetMesh;
			SkinnedMeshLoader* mMeshAnimation;
			]],
			wrapperCode=[[
			static OBJloader::Mesh& targetMesh(FlexibleBodyImplicitSimplePenaltyWin& win) { return win.mTargetMesh;}
			]],
			memberFunctions={[[
			void loadIguanaMotion();
			void startWorld();
			void createTargetMesh(int iframe, double scale_factor, vector3 const& initialPosition);
			]]},
			staticMemberFunctions={[[
			static OBJloader::Mesh& targetMesh(FlexibleBodyImplicitSimplePenaltyWin& win) { return win.mTargetMesh;}
			]]}
		},
		{
			ifdef='USE_BULLETDEP',
			name='FlexibleBodyImplicitSimpleLCPWin',
			properties=[[
			Motion* mMotion;
			OBJloader::Mesh mTargetMesh;
			SkinnedMeshLoader* mMeshAnimation;
			]],
			wrapperCode=[[
			static OBJloader::Mesh& targetMesh(FlexibleBodyImplicitSimpleLCPWin& win) { return win.mTargetMesh;}
			]],
			memberFunctions={[[
			void loadIguanaMotion();
			void startWorld(double timestep);
			void createTargetMesh(int iframe, double scale_factor, vector3 const& initialPosition);
			]]},
			staticMemberFunctions={[[
			static OBJloader::Mesh& targetMesh(FlexibleBodyImplicitSimpleLCPWin& win) { return win.mTargetMesh;}
			]]}
		},
		{
			ifdef='USE_BULLETDEP',
			name='FlexibleBodyImplicitWin',
			properties=[[
				double renderingFrameRate;
				double simulationFrameRate;
				bool matchPose;
				bool performIK;	
				FlLayout* mTRCwin;
				SkinnedMeshLoader* mMeshAnimation;
			]],
			wrapperCode=[[
			static OBJloader::Mesh& targetMesh(FlexibleBodyImplicitWin& win) { return win.mTargetMesh;}
			static OBJloader::Mesh& prevTargetMesh(FlexibleBodyImplicitWin& win) { return win.mPrevTargetMesh;}
			]],
			memberFunctions={[[
			void _ctor();
			int trcWin_setDestination(vector3 dest);
			void setTRCwinTerrain();
			void createTargetMesh(int iframe, double scale_factor, vector3 const& initialPosition);
			void setSkeletonVisible(bool bVisible);
			void start();
			void _frameMove(float fElapsedTime);
			void setOption(const char* title, bool value)
			bool isTRCwin_synthesizing();
			void trcWin_singleFrame() 
			void trcWin_transformSynthesized(matrix4 const& tf);
			void performIguanaIK(Posture& pose, vector3N const& con)
			double getTerrainHeight(vector3 const& pos)
			int trcWin_lastDrawn() 
			Motion& trcWin_terrainMotion() 
			Motion& trcWin_motion() 
			int trcWin_handleRendererEvent(const char* evs, int button, int x, int y);
			bool trcWin_hasTerrain() 
			void trcWin_setDrawSphere(bool);
			void stepSimulation(double step)
			]]},
			staticMemberFunctions={[[
			static OBJloader::Mesh& targetMesh(FlexibleBodyImplicitWin& win) 
			static OBJloader::Mesh& prevTargetMesh(FlexibleBodyImplicitWin& win)
			]]}
		},
		{
			ifdef='USE_BULLETDEP',
			name='IguanaLCPWin',
			properties=[[
				double renderingFrameRate;
				double simulationFrameRate;
				bool matchPose;
				bool performIK;	
				FlLayout* mTRCwin;
				SkinnedMeshLoader* mMeshAnimation;
			]],
			wrapperCode=[[
			static OBJloader::Mesh& targetMesh(IguanaLCPWin& win) { return win.mTargetMesh;}
			static OBJloader::Mesh& prevTargetMesh(IguanaLCPWin& win) { return win.mPrevTargetMesh;}
			]],
			memberFunctions={[[
			void _ctor();
			void setTRCwinTerrain(const char* filename, vector3 const& pos, int sizeX, int sizeY, m_real width, m_real height, m_real heightMax);
			void createTargetMesh(int iframe, double scale_factor, vector3 const& initialPosition);
			void setSkeletonVisible(bool bVisible);
			void start();
			void _frameMove(float fElapsedTime);
			void setOption(const char* title, bool value)
			bool isTRCwin_synthesizing();
			void trcWin_singleFrame() 
			void trcWin_transformSynthesized(matrix4 const& tf);
			void performIguanaIK(Posture& pose, vector3N const& con)
			double getTerrainHeight(vector3 const& pos)
			int trcWin_lastDrawn() 
			Motion& trcWin_terrainMotion() 
			Motion& trcWin_motion() 
			int trcWin_handleRendererEvent(const char* evs, int button, int x, int y);
			bool trcWin_hasTerrain() 
			void trcWin_setDrawSphere(bool);
			void stepSimulation(double step)
			void updateHighMesh(double scale_factor)
			]]},
			staticMemberFunctions={[[
			static OBJloader::Mesh& targetMesh(IguanaLCPWin& win) 
			static OBJloader::Mesh& prevTargetMesh(IguanaLCPWin& win)
			]]}
		},
		{
			ifdef='USE_BULLETDEP',
			name='FlexibleBodyWin',
			memberFunctions={[[
			void createFloor(double floorHeight, vector3 const& size);
			void createTerrain(const char* filename, vector3 const& pos, int sizeX, int sizeY, m_real width, m_real height, m_real heightMax);
			]]},
		},
		{
			ifdef='USE_BULLETDEP',
			name='SimulatorImplicit',
			ctors={'()'},
			properties={
				'OBJloader::Mesh mSimulatedMesh',
				'OBJloader::MeshToEntity* mSimulatedOgreMesh',
			},
			wrapperCode=[[
			static int numLinks(SimulatorImplicit& sim)
			{
				return sim.getSoftBody()->getLinks().size();
			}
			static void stepSimulation(SimulatorImplicit& sim, double step)
			{
				sim.m_dynamicsWorld->stepSimulation(step, 1, step);
			}
			]],
			staticMemberFunctions={[[
			static int numLinks(SimulatorImplicit& sim)
			static void stepSimulation(SimulatorImplicit& sim, double step)
			]]},
			memberFunctions={[[
			void addExternalForce(intvectorn const& indices, vector3N const& forces)
			void createFloor(double floorHeight, vector3 const& size);
			void createTerrain(const char* filename, vector3 const& pos, int sizeX, int sizeY, m_real width, m_real height, m_real heightMax);
			void createSoftBody(OBJloader::Mesh const& targetMesh);
			void extractLinkLength(OBJloader::Mesh const& targetMesh, vectorn& length)
			void changeSoftBodyRestLength(OBJloader::Mesh const& targetMesh)
			void setRestLength(vectorn const& length)
			void startWorld();
			void exitWorld();
			bool isWorldValid();
			void setGravity(vector3 const& gravity);
			void adjustSoftBodyParam(const char* _adjustWhat, const char* selectionFileName, double value);
			void adjustSoftBodyEdgeParam(const char* _adjustWhat, boolN const& selectedEdges, double value)
			void adjustSoftBodyVertexParam(const char* _adjustWhat, boolN const& selectedVertices, double value)
			void setParameter(const char* _what, double value);
			void renderme();
			]]},
			enums={
				{"PENALTY_METHOD", "(int)Physics_ParticleSystem::Config::PENALTY_METHOD"},
				{"LCP_METHOD_FRICTIONLESS", "(int)Physics_ParticleSystem::Config::LCP_METHOD_FRICTIONLESS"},
				{"LCP_METHOD", "(int)Physics_ParticleSystem::Config::LCP_METHOD"},
				{"BARAFF98", "(int)Physics_ParticleSystem::Config::BARAFF98"},
			}
		},
		{
			ifdef='USE_BULLETDEP',
			name='FELearningWin',
			wrapperCode=[[
			static void offsetTargetMesh(FELearningWin& win, vector3 const& offset)
			{
				for(int i=0; i<win.mTargetMesh->numVertex(); i++)
					win.mTargetMesh->getVertex(i)+=offset;
				}

				static const vectorn & offset(FELearningWin& win)		{ return win.mOffset;}
				static OBJloader::Mesh& targetMesh(FELearningWin& win) { return *win.mTargetMesh;}
				static OBJloader::Mesh& simulatedMesh(FELearningWin& win) { return win.mSimulator.mSimulatedMesh;}
			]],
			memberFunctions={[[
			void createTargetMesh(int iframe, double scale_factor);
			void updateTargetMesh(double frame);
			void renderScene();
			void stepSimulation(double step);
			void measureError(OBJloader::Mesh const& mesh1, OBJloader::Mesh const& mesh2, vectorn & offset);
			void setOffset(vectorn const& offset);
			]]},
			staticMemberFunctions={[[
			static void offsetTargetMesh(FELearningWin& win, vector3 const& offset)
			static const vectorn & offset(FELearningWin& win)	
			static OBJloader::Mesh& targetMesh(FELearningWin& win)
			static OBJloader::Mesh& simulatedMesh(FELearningWin& win)
			]]}
		},
		{
			name='_quadi',
			decl='#include "src/ElasticBody/TetraMesh.h"',
			memberFunctions=[[
			int operator()(int i) @ __call
			]]
		},
		{
			name='TetraMeshLoader.TetraMesh',
			decl='namespace TetraMeshLoader { class TetraMesh;}',
			headerFile='src/ElasticBody/TetraMesh.h',
			ctors={'()'},
			memberFunctions=[[
			void transform(matrix4 const& b);
			int getNumNode() 
			vector3 const& nodePos(int i) const	
			vector3 & nodePos(int i)		
			int getNumTetra() const 
			_quadi& getTetra(int i) 
			_quadi const& getTetra(int i)const 
			vector3& getTetraCorner(int iTetra, int icorner) ;
			vector3 calcTetraCenter(int i) const;
			bool loadMesh(const char *file_name_);	
			bool saveMesh(const char *file_name_);
			]],
		},

		{
			name='Optimize',
			className='Optimize_lunawrapper',
			decl=true,
			isLuaInheritable=true,
			isExtendableFromLua=true,
			globalWrapperCode=[[
					class Optimize_lunawrapper: public Optimize, public luna_wrap_object
					{
						public:
						Optimize_lunawrapper() :Optimize() { }

						virtual double objectiveFunction(vectorn const& pos)
						{
							lunaStack l(_L);
							if(pushMemberFunc<Optimize_lunawrapper>(l,"_objectiveFunction")){
								l.push<vectorn>(pos);
								l.call(2,1);
								double out;
								l>>out;
								return out;
							} 
							return 0;
						}
					};
			]],
			ctor='()',
			memberFunctions=[[
			virtual double objectiveFunction(vectorn const& pos)
			void optimize(vectorn const& initialSolution)
			vectorn& getResult()
			void init(double stepSize, int ndim, double max_step, double grad_step, Optimize::Method & method)
			]]
		},
		{
			name='math.CMAwrap',
			cppname='CMAwrap',
			decl=true,
			ctors=[[
				(vectorn const& start_p, vectorn const& stdev, int populationSize, int mu);
				(vectorn const& start_p, vectorn const& stdev, int populationSize);
			]],
			memberFunctions=[[
			std::string testForTermination();	
			void samplePopulation();
			int numPopulation();
			int dim();
			vectornView getPopulation(int i);
			void setVal(int i, double eval);
			void resampleSingle(int i);
			void update();
			void getMean(vectorn& out);
			void getBest(vectorn& out);
			]]
		},
		{
			name='Optimize.Method'
		},


		{
			name='IntegratedWorld',
			decl='class IntegratedWorld;',
			headerFile='femworld/IntegratedWorld.h',
			ctors={'(double, double)'},
			memberFunctions=[[
			void addAttachmentConstraint(const char* name, double stiffness, int index, vector3 const& point);
			bool TimeStepping();
			void addBody(const char*);
			void addBody(const char*, TetraMeshLoader::TetraMesh const& mesh, double m)
			void getAttachmentCst(vector3N& points);
			int addLinearMuscleConstraint(const char* name, double stiffness, int itet, TetraMeshLoader::TetraMesh const& mesh, vector3 const& fiber_direction);
			void setActivationLevel(int iMuscle, double a)
			double getActivationLevel(int iMuscle)  
			void initSimulation()

			// for drawing
			void getCorotateFEMCst(vector3N& lines); // p0-p1, p2-p3, ... six lines per a tet.
			void getLinearMuscleCst(vector3N& lines,  vectorn& activation);
			]]
		},
	},
	modules={
		{
			namespace='Mesh',
			decl='#include "MeshTools.h"',
			functions=[[
			void OBJloader::Triangulation::delaunayTriangulation(OBJloader::Mesh& mesh) @ delaunayTriangulation
			]]
		}
	},
}
function generate()
	loadDefinitionDB(script_path..'/luna_baselib_db.lua')
	loadDefinitionDB(script_path..'/../../PhysicsLib/luna_physicslib_db.lua')
	buildDefinitionDB(bindTargetClassification)
	write(
	[[
	#include "stdafx.h"
	namespace TRL{
	class GSoftBody;
	class DynamicsSimulator_TRL_massSpring;
	}
	namespace OpenHRP {
	class DynamicsSimulator_TRL_penalty;
	}
	#ifdef USE_BULLETDEP
	class FlexibleBodyWin;
	class SimulatorImplicit;
	class FELearningWin;
	class FlexibleBodyImplicitWin;
	class IguanaLCPWin;
	class FlexibleBodyImplicitSimpleWin;
	class FlexibleBodyImplicitSimplePenaltyWin;
	class FlexibleBodyImplicitSimpleLCPWin;
	class IguanaIKSolver;
	#endif
	]]);
	writeIncludeBlock()
	write('#include "MainLib/WrapperLua/luna.h"')
	write('#include "MainLib/WrapperLua/luna_baselib.h"')
	write('#include "PhysicsLib/luna_physics.h"')
	--write('#include "../../MainLib/WrapperLua/luna_mainlib.h"')
	writeHeader(bindTargetClassification)
	flushWritten(source_path..'/luna_flexiblebody.h') -- write to cpp file only when there exist modifications -> no-recompile.

	write([[
	#include "BaseLib/baselib.h"
	#include <OgreAnimable.h>
	#include "MainLib/OgreFltk/FlLayout.h"
#ifdef USE_BULLETDEP
	#include "FlexibleBodyWin.h"
	#include "FELearning.h"
	#include "TRCwin.h"
	#include "IguanaIKSolver.h"
	#include "FlexibleBodyImplicitWin.h"
	#include "IguanaLCPwin.h"
	#include "FlexibleBodyImplicitSimpleWin.h"
	#include "PenaltyWin.h"
	#include "LCPwin.h"
#endif
	#include "TRL/DynamicsSimulator_TRL_massSpring.h"
	#include "softbody2/Physics.h"
	#include "luna_flexiblebody.h"
	//#include "PhysicsLib/TRL/DynamicsSimulator_TRL_penalty.h"
	#include "BaseLib/math/optimize.h"
	#include "cma/CMAwrap.h"
	#include "SurfaceControl.h"
	]])
	writeDefinitions(bindTargetClassification, 'Register_flexiblebody') -- input bindTarget can be non-overlapping subset of entire bindTarget 
	flushWritten(source_path..'/luna_flexiblebody.cpp') -- write to cpp file only when there exist modifications -> no-recompile.
end
