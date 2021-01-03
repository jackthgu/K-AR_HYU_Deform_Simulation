#include "stdafx.h"

#include "sgeom.h"
#include "math.h"
#include "BaseLib/math/Operator.h"
#include "BaseLib/math/Operator_NR.h"

bool sgeom_inside_tetrahedron(const vector3 &x, const vector3 &p1, const vector3 &p2, const vector3 &p3, const vector3 &p4, double *b, int *err_code)
{
	double d, d1, d2, d3, d4;
	matrixn R(4,4), R1(4,4), R2(4,4), R3(4,4), R4(4,4);

	 R(0,0) = p1[0];	 R(1,0) = p2[0];	 R(2,0) = p3[0];	 R(3,0) = p4[0];
 	 R(0,1) = p1[1];	 R(1,1) = p2[1];	 R(2,1) = p3[1];	 R(3,1) = p4[1];
	 R(0,2) = p1[2];	 R(1,2) = p2[2];	 R(2,2) = p3[2];	 R(3,2) = p4[2];
     R(0,3) = 1.0;		 R(1,3) = 1.0;		 R(2,3) = 1.0;		 R(3,3) = 1.0;
																	 
	R1(0,0) = x[0];		R1(1,0) = p2[0];	R1(2,0) = p3[0];	R1(3,0) = p4[0];
	R1(0,1) = x[1];		R1(1,1) = p2[1];	R1(2,1) = p3[1];	R1(3,1) = p4[1];
	R1(0,2) = x[2];		R1(1,2) = p2[2];	R1(2,2) = p3[2];	R1(3,2) = p4[2];
    R1(0,3) = 1.0;		R1(1,3) = 1.0;		R1(2,3) = 1.0;		R1(3,3) = 1.0;
																	 
	R2(0,0) = p1[0];	R2(1,0) = x[0];		R2(2,0) = p3[0];	R2(3,0) = p4[0];
	R2(0,1) = p1[1];	R2(1,1) = x[1];		R2(2,1) = p3[1];	R2(3,1) = p4[1];
	R2(0,2) = p1[2];	R2(1,2) = x[2];		R2(2,2) = p3[2];	R2(3,2) = p4[2];
    R2(0,3) = 1.0;		R2(1,3) = 1.0;		R2(2,3) = 1.0;		R2(3,3) = 1.0;
																	 
	R3(0,0) = p1[0];	R3(1,0) = p2[0];	R3(2,0) = x[0];		R3(3,0) = p4[0];
	R3(0,1) = p1[1];	R3(1,1) = p2[1];	R3(2,1) = x[1];		R3(3,1) = p4[1];
	R3(0,2) = p1[2];	R3(1,2) = p2[2];	R3(2,2) = x[2];		R3(3,2) = p4[2];
    R3(0,3) = 1.0;		R3(1,3) = 1.0;		R3(2,3) = 1.0;		R3(3,3) = 1.0;
																	 
	R4(0,0) = p1[0];	R4(1,0) = p2[0];	R4(2,0) = p3[0];	R4(3,0) = x[0];
	R4(0,1) = p1[1];	R4(1,1) = p2[1];	R4(2,1) = p3[1];	R4(3,1) = x[1];
	R4(0,2) = p1[2];	R4(1,2) = p2[2];	R4(2,2) = p3[2];	R4(3,2) = x[2];
    R4(0,3) = 1.0;		R4(1,3) = 1.0;		R4(2,3) = 1.0;		R4(3,3) = 1.0;
	
	d = m::determinant(R);
	d1 = m::determinant(R1);
	d2 = m::determinant(R2);
	d3 = m::determinant(R3);
	d4 = m::determinant(R4);

	// invalid determinants, d != d1+d2+d3+d4
	if ( fabs(d - (d1+d2+d3+d4)) > 1E-10 ) {
		if ( err_code != NULL ) *err_code = -3;	
		return false;	
	}

	// degenerate tetrahedron
	if ( fabs(d) < 1E-8 ) {
		if ( err_code != NULL ) *err_code = -2;	
		return false;	
	}

	// barycentric coordinates of x
	b[0] = d1/d;
	b[1] = d2/d;
	b[2] = d3/d;
	b[3] = d4/d;

	// if the sign of any di equals that of d then x is inside the tetrahedron
	if ( err_code != NULL ) *err_code = 0;
	if ( d>=0 && d1>=0 && d2>=0 && d3>=0 && d4>=0 ) return true;
	if ( d<=0 && d1<=0 && d2<=0 && d3<=0 && d4<=0 ) return true;

	// else, x is located outside of the tetrahedron
	if ( err_code != NULL ) *err_code = -1;	
	return false;
}

double Inner(const vector3& x, const vector3& y)
{
	return x%y;
}


bool sgeom_inside_triangle(const vector3 &x, const vector3 &p1, const vector3 &p2, const vector3 &p3)
{
	vector3 n;
	n.cross(p2-p1, p3-p1);

	if ( fabs(Inner(x-p1, n)) > 1E-8 ) return false;		// x is not on the plane

	if ( Inner(x-p1, Cross(n, p2-p1)) < 0 ) return false;	// x is outside of the line segment between p1 and p2
	if ( Inner(x-p2, Cross(n, p3-p2)) < 0 ) return false;	// x is outside of the line segment between p2 and p3
	if ( Inner(x-p3, Cross(n, p1-p3)) < 0 ) return false;	// x is outside of the line segment between p3 and p1

	return true;
}

bool sgeom_inside_line_segment(const vector3 &x, const vector3 &p1, const vector3 &p2)
{
	vector3 m = p2-p1;
	m.normalize();

	if ( fabs(Norm(Cross(x-p1, p2-p1))) > 1E-8 ) return false;	// x is not on the line

	if ( Inner(x-p1, p2-p1) < 0 ) return false;					// x is outside of p1
	if ( Inner(x-p2, p1-p2) < 0 ) return false;					// x is outside of p2

	return true;
}

bool sgeom_positive_side_of_triangle_plane(const vector3 &x, const vector3 &p1, const vector3 &p2, const vector3 &p3, double *pDist_)
{
	vector3 n = Cross(p2-p1, p3-p1);
	double d = Inner(x-p1, n);
	if ( pDist_ != NULL ) *pDist_ = d;
	if ( d > 0 ) return true;
	else return false;
}

