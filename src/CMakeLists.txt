# -*- mode: cmake; -*-
cmake_minimum_required(VERSION 2.8)

project (FlexibleBody)


# include ../Common.cmake
#set (CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../taesooLib/Samples")
set (CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/taesooLib/Samples")
get_filename_component(CAR_SAMPLE_DIR ${CMAKE_SOURCE_DIR} PATH)
get_filename_component(PROJ_ROOT_DIR ${CAR_SAMPLE_DIR} PATH)
#set(TAESOOLIB_DIR "${PROJ_ROOT_DIR}/taesooLib")
set(TAESOOLIB_DIR "${CMAKE_SOURCE_DIR}/taesooLib")
#set(SAMPLE_DIR "${PROJ_ROOT_DIR}/taesooLib/Samples")

find_package(Threads)
find_package(OpenCV REQUIRED)

set(UseRealsense FALSE)

if(UseRealsense)
	set(RecordRealsense TRUE)
	set(PlaybackFrames FALSE)

else()
	set(RecordRealsense FALSE)
	set(PlaybackFrames TRUE)
endif()

if(UseRealsense)
	find_package(realsense2 REQUIRED)
	message(STATUS "Found realsense2 in ${realsense_INCLUDE_DIR}")
else()
	add_definitions(-DNO_REALSENSE)
endif()

if(RecordRealsense AND UseRealsense)
	add_definitions(-DRECORD_REALSENSE)
elseif(RecordRealsense AND (NOT UseRealsense))
	message(FATAL_ERROR "Recording realsense requires enable UseRealsense.")
endif()

if(PlaybackFrames AND (NOT UseRealsense))
	add_definitions(-DPLAYBACK_REALSENSE)
elseif(PlaybackFrames AND UseRealsense)
	message(FATAL_ERROR "Playback realsense and Use realsense couldn't be set together.")
endif()

find_package(GLUT REQUIRED)
message(STATUS "Found GLUT in ${GLUT_INCLUDE_DIR}")
find_package(glfw3 REQUIRED)
message(STATUS "Found GLFW3 in ${GLFW3_INCLUDE_DIR}")
find_package(GLEW REQUIRED)
message(STATUS "Found GLEW in ${GLEW_INCLUDE_DIR}")
find_package(OpenGL REQUIRED)
message(STATUS "Found OpenGL in ${OPENGL_INCLUDE_DIR}")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if (WIN32)
	message(${TAESOOLIB_DIR})
	include(Common_fast) 

	set(LUNA_GEN "${TAESOOLIB_DIR}/MainLib/WrapperLua/luna_gen.lua")
else()
	#include(Common_with_bullet)
	#set(NoDevil TRUE)
	#set(NoLapack TRUE)
	#set(NoUMFpack TRUE)
	#include(Common_baselib) 
	#add_subdirectory(../../taesooLib/BaseLib "${CMAKE_SOURCE_DIR}/../../taesooLib/BaseLib/build_${CMAKE_BUILD_TYPE}" )
	#add_subdirectory(../../taesooLib/MainLib "${CMAKE_SOURCE_DIR}/../../taesooLib/MainLib/build_${CMAKE_BUILD_TYPE}" )
	#include(Common_mainlib) 
	#add_subdirectory(../../taesooLib/PhysicsLib "${CMAKE_SOURCE_DIR}/../../taesooLib/PhysicsLib/build_${CMAKE_BUILD_TYPE}" )
	#add_subdirectory(../../taesooLib/ClassificationLib "${CMAKE_SOURCE_DIR}/../../taesooLib/ClassificationLib/build_${CMAKE_BUILD_TYPE}" )
	set(useShared TRUE)
	set(NoDevil TRUE)
	set(NoFreeImage FALSE)
	set(NoLapack TRUE)
	set(NoUMFpack TRUE)


	include(Common_baselib) 
	add_subdirectory(${CMAKE_SOURCE_DIR}/taesooLib/BaseLib "${CMAKE_SOURCE_DIR}/taesooLib/BaseLib/build_shared_${CMAKE_BUILD_TYPE}" )
	add_subdirectory(${CMAKE_SOURCE_DIR}/taesooLib/MainLib "${CMAKE_SOURCE_DIR}/taesooLib/MainLib/build_shared_${CMAKE_BUILD_TYPE}" )
	include(Common_mainlib) 
	# PhysicsLib settings
	set(NoAISTsim TRUE) # use TRL_LCP instead!
	set(NoGMBSsim TRUE) # use TRL_LCP instead!
	set(NoUTsim TRUE) # use TRL_LCP instead!
	set(NoBulletSim TRUE) # use TRL_LCP instead!
	add_subdirectory(${CMAKE_SOURCE_DIR}/taesooLib/PhysicsLib "${CMAKE_SOURCE_DIR}/taesooLib/PhysicsLib/build_shared_${CMAKE_BUILD_TYPE}" )
	add_subdirectory(${CMAKE_SOURCE_DIR}/taesooLib/ClassificationLib "${CMAKE_SOURCE_DIR}/taesooLib/ClassificationLib/build_shared_${CMAKE_BUILD_TYPE}" )


	if (TRUE)
		add_subdirectory( fem )
		link_directories(./fem/)
		include(FindOpenMP)
		if(OPENMP_FOUND)
			set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
			set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
			set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
		endif()
endif()

set(TAESOO_LIBS
	BaseLib 
	${LUA_LIB}
	MainLib 
	#blas
	#LibLinearMath
	#LibBulletCollision
	PhysicsLib 
	ClassificationLib
	)
endif()


include_directories (	
	.
	..
	#../../taesooLib
	#../../taesooLib/Samples/classification
	#../../taesooLib/PhysicsLib/CollisionDetector/Bullet/bullet-2.68/src
	#../../taesooLib/PhysicsLib/CollisionDetector/libccd/
	/usr/include/eigen3
	#../../taesooLib/PhysicsLib/CollisionDetector/Bullet/bullet-2.68/src/BulletCollision/CollisionDispatch
	${GLFW_INCLUDE_DIRS} ${realsense_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR} ${GLEW_INCLUDE_DIRS}
	${TAESOOLIB_DIR}
	)
set(src11
	"./Delaunay.cpp"
	"./Delaunay.h"
	"MeshTools.h"
	"MeshTools.cpp"
  "./INativeRender.cpp"
  "./INativeRender.h"
  "./FltkOpenGLrenderer.cpp"
  "./FltkOpenGLrenderer.h"
  "./TestWin.cpp"
  "./TestWin.h"
   #TRL/DynamicsSimulator_TRL_massSpring.h
	#TRL/DynamicsSimulator_TRL_massSpring.cpp
	SoftBodyLCP.h
	SoftBodyLCP.cpp
	#SurfaceControl.h
	#SurfaceControl.cpp
	"${CMAKE_SOURCE_DIR}/luna_flexiblebody.cpp" # important. without this, luna_gen doesn't run.
	"./cma/cmaes.c"
	"./cma/CMAwrap.cpp"
	"./cma/CMAwrap.h"
	femworld/IntegratedWorld.h
	femworld/IntegratedWorld.cpp
	#	"./ElasticBody/TetraDeformTestWin.cpp"# -- savezbuffer files not uploaded, causing compile errors.
	#"./TestWin2.cpp"
	#"./TestWin2.h"
	)
if(UNIX)
	if(NOT APPLE)
		SET_SOURCE_FILES_PROPERTIES( ${src11} PROPERTIES COMPILE_FLAGS "-std=c++0x")
	endif()
endif()
set(src_unused
retarget.cpp
retarget.h
LCPwin.cpp
LCPwin.h
bulletTools.cpp
bulletTools.h
BunnyMesh.h
ElasticBody
FELearning.cpp
FELearning.h
IguanaIKSolver.cpp
IguanaIKSolver.h
FELearning_luna.cpp
PenaltyWin.cpp
PenaltyWin.h
FlexibleBodyImplicitSimpleWin.cpp
FlexibleBodyImplicitSimpleWin.h
IguanaLCPwin.cpp
IguanaLCPwin.h
FlexibleBodyImplicitWin.cpp
FlexibleBodyImplicitWin.h
FlexibleBodyWin.cpp
FlexibleBodyWin.h
IKSolver.cpp
IKSolver.h
	)
set(src
	#gmm.cpp
	#gmm.h
Mesh_old.cpp
Mesh_old.h
#Multipole.cpp
#Multipole.h
OgreFltk.cpp
OgreFltk.h
#SoftDemo.cpp
#SoftDemo.h
#SoftDemoWin.h
SpringEditWin.cpp
SpringEditWin.h
#OperatorSignal.cpp
#OperatorSignal.h
Concat.cpp
Concat.h
#MotionUtilTwo.cpp
#MotionUtilTwo.h
stdafx.cpp
stdafx.h
#Stitch.cpp
#Stitch.h
Terrain.cpp
Terrain.h
TorusMesh.h
#umfsolver.h
#	  "./OgreFltk.cpp"
RigidBodyWin.cpp
RigidBodyWin.h
#	"./bulletTools.cpp"
#	"./bulletTools.h"
#	"./FELearning.cpp"
#	"./FELearning.h"
#	"./FELearning_luna.cpp"
#	"./FlexibleBodyImplicitWin.cpp"
#	"./FlexibleBodyImplicitWin.h"
#	"./FlexibleBodyWin.cpp"
#	"./FlexibleBodyWin.h"
#	# "./Multipole.cpp"
#	# "./Multipole.h"
#	"./Ogre_ShapeDrawer.cpp"
#	"./Ogre_ShapeDrawer.h"
#	"./SimulatorImplicit.cpp"
#	"./SimulatorImplicit.h"
#	#"./SoftDemo.cpp"
#	#"./SoftDemo.h"
#	#"./SoftDemoWin.h"
#	"./SpringEditWin.cpp"
#	"./Terrain.cpp"
#	"./Terrain.h"
#	"./TRCwin.cpp"
#	"./TRCwin.h"
#"./ElasticBody/AsRigidAsPossible.cpp"
#"./ElasticBody/AsRigidAsPossible.h"
"./ElasticBody/CompareHelper.cpp"
"./ElasticBody/CompareHelper.h"
"./ElasticBody/DeformableBody.cpp"
"./ElasticBody/DeformableBody.h"
"./ElasticBody/EmbeddedSurface.cpp"
"./ElasticBody/EmbeddedSurface.h"
"./ElasticBody/HarmonicCoord.cpp"
"./ElasticBody/MeanValueWeights.cpp"
"./ElasticBody/MeanValueWeights.h"
#"./ElasticBody/ModelingWin.cpp"
#"./ElasticBody/ModelingWin.h"
"./ElasticBody/sgeom.cpp"
"./ElasticBody/sgeom.h"
"./ElasticBody/TetraMesh.cpp"
"./ElasticBody/TetraMesh.h"
"./ElasticBody/TetraMeshEditWin.cpp"

#"./ElasticBody/Tool.cpp"
#"./ElasticBody/Tool.h"
"./ElasticBody/UI_mode.cpp"
"./ElasticBody/UI_mode.h"
#"BulletSoftBody/btSoftBody2.cpp"
#"BulletSoftBody/btSoftBody2.h"
#"BulletSoftBody/btSoftBody2Helpers.cpp"
#"BulletSoftBody/btSoftRigidDynamicsWorld2.cpp"
"Vision/ImagePlane.h"
"Vision/ImagePlane.cpp"
"Vision/ObjModel.h"
"Vision/ObjModel.cpp"
"Vision/realsense3.cpp"
"Vision/SettingLoader.h"
"Vision/SpacePrimitives.h"
"Vision/SpacePrimitives.cpp"
"Vision/TrackerUtils.h"
./saveZbuffer.cpp
./saveZbuffer.h
"sparseicp/ICP_wrap.h"
"sparseicp/ICP_wrap.cpp"
"sparseicp/ICP.h"
"sparseicp/nanoflann.hpp"
)

add_definitions(-DBT_USE_DOUBLE_PRECISION)
#add_definitions(-DTIXML_USE_STL)
#SUBDIRS(BulletSoftBody softbody2)
#SUBDIRS(softbody2)
set(luna_script "${CMAKE_SOURCE_DIR}")

add_custom_command(
	OUTPUT "${luna_script}/luna_flexiblebody.cpp"
	DEPENDS "${luna_script}/luna_flexiblebody.lua" "${LUNA_GEN}" 
	PRE_BUILD
	COMMAND lua "${LUNA_GEN}" "${luna_script}/luna_flexiblebody.lua"
)

add_subdirectory(ElasticBody/opennl)
add_executable(OgreFltk
	${src}
	${src11}
  )

if (WIN32)
	set(UMFPACK_LIBS
		"${CMAKE_SOURCE_DIR}/../../../dependencies/UMFPACK5.2/AMD.lib"
		"${CMAKE_SOURCE_DIR}/../../../dependencies/UMFPACK5.2/UMFPACK_noblas.lib"
		)
else()
	set(UMFPACK_LIBS
		umfpack
		)
endif()
if(WIN32)
	
  set(QPOASES_LIB 
	  # build using cd qp<TAB>;cmake  . ;make
	  "${CMAKE_SOURCE_DIR}/../QP_controller/qpOASES-3.0beta/libs/Release/qpOASES.lib"
	  "${CMAKE_SOURCE_DIR}/../QP_controller/qpOASES-3.0beta/libs/Release/qpOASESextras.lib"
	  )
else()
  set(QPOASES_LIB 
	  # build using cd qp<T../QP_controller/AB>;cmake  . ;make
	  "${CMAKE_SOURCE_DIR}/../QP_controller/qpOASES-3.0beta/libs/libqpOASES.a"
	  "${CMAKE_SOURCE_DIR}/../QP_controller/qpOASES-3.0beta/libs/libqpOASESextras.a" dl
	  )
endif()
target_link_libraries(OgreFltk 
	${TAESOO_LIBS}
	${FLTK_LIBS}
	${IMAGE_LIBS}
	${OGRE_LIBS}
	#LibBulletSoftBody
	#LibSoftBody2
	${UMFPACK_LIBS}
	blender_ONL
	#${QPOASES_LIB}
	dl
	fem
	#tinyxml
	${OpenCV_LIBS}
	${realsense2_LIBRARY}
	${GLFW_STATIC_LIBRARIES}
	${OPENGL_gl_LIBRARY}
	glfw
	GLEW
)
if(WIN32)
  add_custom_command(TARGET OgreFltk
                       POST_BUILD
                       COMMAND lua "${CMAKE_CURRENT_SOURCE_DIR}/build.lua" install)
endif()
