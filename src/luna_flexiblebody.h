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
	
		#include <stdio.h>
		#include <string.h>
		#include <string>
		extern "C"
		{
			#include "lua.h"
			#include "lualib.h"
			#include "lauxlib.h"
			//#include "luadebug.h"
		}
		
#include "MainLib/WrapperLua/luna.h"
#include "MainLib/WrapperLua/luna_baselib.h"
#include "PhysicsLib/luna_physics.h"
#ifndef genlua_luna_flexiblebody_lua45039582_def12            // 1300
#define genlua_luna_flexiblebody_lua45039582_def12            // 1301
// Declare all classes before including this file or using the "decl" property in the luna_gen definition file. // 1302
// e.g. class LMat; class LMatView; ....                      // 1303
// The forward declaration is not automatically made because luna_gen cannot distinguish struct, class, or namespace. // 1304
// But you can declare a class explictly by using something like decl='class AAA;'  // 1305
// Also, you can add a "#include" sentence by using headerFile="AAA.h"  // 1306
// The number at the end of each line denotes the line number of luna_gen.lua which generated that line // 1309
class LinearMeshDeformer;                                     // 1313
class LinearDeformer;                                         // 1313
class InverseDeformer;                                        // 1313
#include "src/ElasticBody/TetraMesh.h"                        // 1313
namespace TetraMeshLoader { class TetraMesh;}                 // 1313
class Optimize_lunawrapper;                                   // 1313
class CMAwrap;                                                // 1313
class IntegratedWorld;                                        // 1313
#include "MeshTools.h"                                        // 1313
template<>                                                    // 1324
 class LunaTraits<LinearMeshDeformer > {
public:                                                       // 1326
    static const char className[];                            // 1335
    static const int uniqueID;                                // 1336
    static luna_RegType methods[];                            // 1337
    static LinearMeshDeformer* _bind_ctor(lua_State *L);      // 1339
    static void _bind_dtor(LinearMeshDeformer* obj);          // 1340
    typedef LinearMeshDeformer base_t;                        // 1342
};                                                            // 1348
template<>                                                    // 1324
 class LunaTraits<LinearDeformer > {
public:                                                       // 1326
    static const char className[];                            // 1335
    static const int uniqueID;                                // 1336
    static luna_RegType methods[];                            // 1337
    static LinearDeformer* _bind_ctor(lua_State *L);          // 1339
    static void _bind_dtor(LinearDeformer* obj);              // 1340
    typedef LinearMeshDeformer base_t;                        // 1342
};                                                            // 1348
template<>                                                    // 1324
 class LunaTraits<InverseDeformer > {
public:                                                       // 1326
    static const char className[];                            // 1335
    static const int uniqueID;                                // 1336
    static luna_RegType methods[];                            // 1337
    static InverseDeformer* _bind_ctor(lua_State *L);         // 1339
    static void _bind_dtor(InverseDeformer* obj);             // 1340
    typedef InverseDeformer base_t;                           // 1342
};                                                            // 1348
#if defined (USE_BULLETDEP)                                   // 1320
template<>                                                    // 1324
 class LunaTraits<btRigidBody > {
public:                                                       // 1326
    static const char className[];                            // 1335
    static const int uniqueID;                                // 1336
    static luna_RegType methods[];                            // 1337
    static btRigidBody* _bind_ctor(lua_State *L);             // 1339
    static void _bind_dtor(btRigidBody* obj);                 // 1340
    typedef btRigidBody base_t;                               // 1342
};                                                            // 1348
#endif //defined (USE_BULLETDEP)                              // 1350
#if defined (USE_BULLETDEP)                                   // 1320
template<>                                                    // 1324
 class LunaTraits<IguanaIKSolver > {
public:                                                       // 1326
    static const char className[];                            // 1335
    static const int uniqueID;                                // 1336
    static luna_RegType methods[];                            // 1337
    static IguanaIKSolver* _bind_ctor(lua_State *L);          // 1339
    static void _bind_dtor(IguanaIKSolver* obj);              // 1340
    typedef IguanaIKSolver base_t;                            // 1342
};                                                            // 1348
#endif //defined (USE_BULLETDEP)                              // 1350
#if defined (USE_BULLETDEP)                                   // 1320
template<>                                                    // 1324
 class LunaTraits<FlexibleBodyImplicitSimpleWin > {
public:                                                       // 1326
    static const char className[];                            // 1335
    static const int uniqueID;                                // 1336
    static luna_RegType methods[];                            // 1337
    static FlexibleBodyImplicitSimpleWin* _bind_ctor(lua_State *L); // 1339
    static void _bind_dtor(FlexibleBodyImplicitSimpleWin* obj); // 1340
    typedef FlexibleBodyImplicitSimpleWin base_t;             // 1342
};                                                            // 1348
#endif //defined (USE_BULLETDEP)                              // 1350
#if defined (USE_BULLETDEP)                                   // 1320
template<>                                                    // 1324
 class LunaTraits<FlexibleBodyImplicitSimplePenaltyWin > {
public:                                                       // 1326
    static const char className[];                            // 1335
    static const int uniqueID;                                // 1336
    static luna_RegType methods[];                            // 1337
    static FlexibleBodyImplicitSimplePenaltyWin* _bind_ctor(lua_State *L); // 1339
    static void _bind_dtor(FlexibleBodyImplicitSimplePenaltyWin* obj); // 1340
    typedef FlexibleBodyImplicitSimplePenaltyWin base_t;      // 1342
static luna__hashmap properties;                              // 1344
static luna__hashmap write_properties;                        // 1345
};                                                            // 1348
#endif //defined (USE_BULLETDEP)                              // 1350
#if defined (USE_BULLETDEP)                                   // 1320
template<>                                                    // 1324
 class LunaTraits<FlexibleBodyImplicitSimpleLCPWin > {
public:                                                       // 1326
    static const char className[];                            // 1335
    static const int uniqueID;                                // 1336
    static luna_RegType methods[];                            // 1337
    static FlexibleBodyImplicitSimpleLCPWin* _bind_ctor(lua_State *L); // 1339
    static void _bind_dtor(FlexibleBodyImplicitSimpleLCPWin* obj); // 1340
    typedef FlexibleBodyImplicitSimpleLCPWin base_t;          // 1342
static luna__hashmap properties;                              // 1344
static luna__hashmap write_properties;                        // 1345
};                                                            // 1348
#endif //defined (USE_BULLETDEP)                              // 1350
#if defined (USE_BULLETDEP)                                   // 1320
template<>                                                    // 1324
 class LunaTraits<FlexibleBodyImplicitWin > {
public:                                                       // 1326
    static const char className[];                            // 1335
    static const int uniqueID;                                // 1336
    static luna_RegType methods[];                            // 1337
    static FlexibleBodyImplicitWin* _bind_ctor(lua_State *L); // 1339
    static void _bind_dtor(FlexibleBodyImplicitWin* obj);     // 1340
    typedef FlexibleBodyImplicitWin base_t;                   // 1342
static luna__hashmap properties;                              // 1344
static luna__hashmap write_properties;                        // 1345
};                                                            // 1348
#endif //defined (USE_BULLETDEP)                              // 1350
#if defined (USE_BULLETDEP)                                   // 1320
template<>                                                    // 1324
 class LunaTraits<IguanaLCPWin > {
public:                                                       // 1326
    static const char className[];                            // 1335
    static const int uniqueID;                                // 1336
    static luna_RegType methods[];                            // 1337
    static IguanaLCPWin* _bind_ctor(lua_State *L);            // 1339
    static void _bind_dtor(IguanaLCPWin* obj);                // 1340
    typedef IguanaLCPWin base_t;                              // 1342
static luna__hashmap properties;                              // 1344
static luna__hashmap write_properties;                        // 1345
};                                                            // 1348
#endif //defined (USE_BULLETDEP)                              // 1350
#if defined (USE_BULLETDEP)                                   // 1320
template<>                                                    // 1324
 class LunaTraits<FlexibleBodyWin > {
public:                                                       // 1326
    static const char className[];                            // 1335
    static const int uniqueID;                                // 1336
    static luna_RegType methods[];                            // 1337
    static FlexibleBodyWin* _bind_ctor(lua_State *L);         // 1339
    static void _bind_dtor(FlexibleBodyWin* obj);             // 1340
    typedef FlexibleBodyWin base_t;                           // 1342
};                                                            // 1348
#endif //defined (USE_BULLETDEP)                              // 1350
#if defined (USE_BULLETDEP)                                   // 1320
template<>                                                    // 1324
 class LunaTraits<SimulatorImplicit > {
public:                                                       // 1326
    static const char className[];                            // 1335
    static const int uniqueID;                                // 1336
    static luna_RegType methods[];                            // 1337
    static SimulatorImplicit* _bind_ctor(lua_State *L);       // 1339
    static void _bind_dtor(SimulatorImplicit* obj);           // 1340
    typedef SimulatorImplicit base_t;                         // 1342
static luna__hashmap properties;                              // 1344
static luna__hashmap write_properties;                        // 1345
};                                                            // 1348
#endif //defined (USE_BULLETDEP)                              // 1350
#if defined (USE_BULLETDEP)                                   // 1320
template<>                                                    // 1324
 class LunaTraits<FELearningWin > {
public:                                                       // 1326
    static const char className[];                            // 1335
    static const int uniqueID;                                // 1336
    static luna_RegType methods[];                            // 1337
    static FELearningWin* _bind_ctor(lua_State *L);           // 1339
    static void _bind_dtor(FELearningWin* obj);               // 1340
    typedef FELearningWin base_t;                             // 1342
};                                                            // 1348
#endif //defined (USE_BULLETDEP)                              // 1350
template<>                                                    // 1324
 class LunaTraits<_quadi > {
public:                                                       // 1326
    static const char className[];                            // 1335
    static const int uniqueID;                                // 1336
    static luna_RegType methods[];                            // 1337
    static _quadi* _bind_ctor(lua_State *L);                  // 1339
    static void _bind_dtor(_quadi* obj);                      // 1340
    typedef _quadi base_t;                                    // 1342
};                                                            // 1348
template<>                                                    // 1324
 class LunaTraits<TetraMeshLoader ::TetraMesh > {
public:                                                       // 1326
    static const char className[];                            // 1335
    static const int uniqueID;                                // 1336
    static luna_RegType methods[];                            // 1337
    static TetraMeshLoader ::TetraMesh* _bind_ctor(lua_State *L); // 1339
    static void _bind_dtor(TetraMeshLoader ::TetraMesh* obj); // 1340
    typedef TetraMeshLoader ::TetraMesh base_t;               // 1342
};                                                            // 1348
template<>                                                    // 1324
 class LunaTraits<Optimize_lunawrapper > {
public:                                                       // 1326
    static const char className[];                            // 1335
    static const int uniqueID;                                // 1336
    static luna_RegType methods[];                            // 1337
    static Optimize_lunawrapper* _bind_ctor(lua_State *L);    // 1339
    static void _bind_dtor(Optimize_lunawrapper* obj);        // 1340
    typedef Optimize_lunawrapper base_t;                      // 1342
};                                                            // 1348
template<>                                                    // 1324
 class LunaTraits<CMAwrap > {
public:                                                       // 1326
    static const char className[];                            // 1335
    static const int uniqueID;                                // 1336
    static luna_RegType methods[];                            // 1337
    static CMAwrap* _bind_ctor(lua_State *L);                 // 1339
    static void _bind_dtor(CMAwrap* obj);                     // 1340
    typedef CMAwrap base_t;                                   // 1342
};                                                            // 1348
template<>                                                    // 1324
 class LunaTraits<Optimize ::Method > {
public:                                                       // 1326
    static const char className[];                            // 1335
    static const int uniqueID;                                // 1336
    static luna_RegType methods[];                            // 1337
    static Optimize ::Method* _bind_ctor(lua_State *L);       // 1339
    static void _bind_dtor(Optimize ::Method* obj);           // 1340
    typedef Optimize ::Method base_t;                         // 1342
};                                                            // 1348
template<>                                                    // 1324
 class LunaTraits<IntegratedWorld > {
public:                                                       // 1326
    static const char className[];                            // 1335
    static const int uniqueID;                                // 1336
    static luna_RegType methods[];                            // 1337
    static IntegratedWorld* _bind_ctor(lua_State *L);         // 1339
    static void _bind_dtor(IntegratedWorld* obj);             // 1340
    typedef IntegratedWorld base_t;                           // 1342
};                                                            // 1348
 class luna__interface__Mesh {
public:                                                       // 1326
    static const char moduleName[];                           // 1331
    typedef LunaModule<luna__interface__Mesh> luna_t;         // 1332
    static luna_RegType methods[];                            // 1333
};                                                            // 1348
#endif                                                        // 1353