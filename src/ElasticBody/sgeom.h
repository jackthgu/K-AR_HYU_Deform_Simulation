//================================================================================
//         FUNCTIONS FOR SIMPLE GEOMETRY TEST
// 
//                                                               junggon@gmail.com
//================================================================================

#ifndef _FUNCTIONS_FOR_SIMPLE_GEOMETRY_TEST_
#define _FUNCTIONS_FOR_SIMPLE_GEOMETRY_TEST_

double Inner(const vector3& x, const vector3& y);
inline vector3 Cross(const vector3& x, const vector3& y){ return x.cross(y);}
inline double Norm(const vector3& x)	{return x.length();}

bool sgeom_inside_tetrahedron(const vector3 &x, const vector3 &p1, const vector3 &p2, const vector3 &p3, const vector3 &p4, double *b, int *err_code = NULL);
// If x is located inside the tetrahedron (p1, p2, p3, p4), then return true.
// (b[0], b[1], b[2], b[3]) = barycentric coordinates of x, i.e., x = b[0]*p1 + b[1]*p2 + b[2]*p3 + b[3]*p4
// reference: http://steve.hollasch.net/cgindex/geometry/ptintet.html
// err_code =  0 : no error (success)
//            -1 : x is located outside of the tetrahedron
//            -2 : degenerate tetrahedron
//            -3 : invalid determinants

bool sgeom_inside_triangle(const vector3 &x, const vector3 &p1, const vector3 &p2, const vector3 &p3);
// If x is located inside the triangle (p1, p2, p3), then return true.
// (b[0], b[1], b[3]) = barycentric coordinates of x, i.e., x = b[0]*p1 + b[1]*p2 + b[2]*p3

bool sgeom_inside_line_segment(const vector3 &x, const vector3 &p1, const vector3 &p2);
// If x is located inside the line segment of (p1, p2), then return true.
// (b[0], b[1]) = barycentric coordinates of x, i.e., x = b[0]*p1 + b[1]*p2

bool sgeom_positive_side_of_triangle_plane(const vector3 &x, const vector3 &p1, const vector3 &p2, const vector3 &p3, double *pDist_ = NULL);
// If x is located positive side of the plane extended from the triangle (p1, p2, p3), then return true.
#endif

