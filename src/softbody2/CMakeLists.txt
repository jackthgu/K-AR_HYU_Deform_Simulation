
project(libsoftbody2)
add_definitions(-DBT_USE_DOUBLE_PRECISION)

set(BULLET_PHYSICS_SOURCE_DIR ../../..)
INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/src }
)

ADD_LIBRARY(LibSoftBody2
"Physics.h"
"Physics_Constraint.cpp"
"Physics_Constraint.h"
"Physics_Force.cpp"
"Physics_Force.h"
"Physics_GravityForce.cpp"
"Physics_GravityForce.h"
"Physics_Impulses.cpp"
"Physics_Impulses.h"
"Physics_LargeVector.cpp"
"Physics_LargeVector.h"
"Physics_Matrix3x3.cpp"
"Physics_Matrix3x3.h"
"Physics_ParticleSystem.cpp"
"Physics_ParticleSystem.h"
"Physics_SparseSymmetricMatrix.cpp"
"Physics_SparseSymmetricMatrix.h"
"Physics_SpringForce.cpp"
"Physics_SpringForce.h"
"Physics_SymmetricMatrix.cpp"
"Physics_SymmetricMatrix.h"
"Physics_Vector3.cpp"
"Physics_Vector3.h"
)
