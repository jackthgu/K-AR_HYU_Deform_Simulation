
#include "nanoflann.hpp"
#include "stdafx.h"
#include "ICP.h" //-- do not include this

inline void copy(matrixn const& p1, Eigen::Matrix<double, 3, Eigen::Dynamic>& vertices)
{
	vertices.resize(Eigen::NoChange, p1.rows());
	for(int curr_vertex=0; curr_vertex<p1.rows(); curr_vertex++)
	{
		vertices(0,curr_vertex) = p1(0,curr_vertex);
		vertices(1,curr_vertex) = p1(1,curr_vertex);
		vertices(2,curr_vertex) = p1(2,curr_vertex);
	}
}
inline void copy(vector3N const& p1, Eigen::Matrix<double, 3, Eigen::Dynamic>& vertices)
{
	vertices.resize(Eigen::NoChange, p1.rows());
	for(int curr_vertex=0; curr_vertex<p1.rows(); curr_vertex++)
	{
		vertices(0,curr_vertex) = p1(curr_vertex).x;
		vertices(1,curr_vertex) = p1(curr_vertex).y;
		vertices(2,curr_vertex) = p1(curr_vertex).z;
	}
}
inline void copy(Eigen::Matrix<double, 3, Eigen::Dynamic>const& vertices, matrixn & p1)
{
	p1.resize(vertices.rows(),3);
	for(int curr_vertex=0; curr_vertex<p1.rows(); curr_vertex++)
	{
		p1(curr_vertex,0)=vertices(0,curr_vertex);
		p1(curr_vertex,1)=vertices(1,curr_vertex);
		p1(curr_vertex,2)=vertices(2,curr_vertex);
	}
}
inline void copy(Eigen::Matrix<double, 3, Eigen::Dynamic>const& vertices, vector3N & p1)
{
	p1.resize(vertices.cols());
	for(int curr_vertex=0; curr_vertex<p1.rows(); curr_vertex++)
	{
		p1(curr_vertex).x=vertices(0,curr_vertex);
		p1(curr_vertex).y=vertices(1,curr_vertex);
		p1(curr_vertex).z=vertices(2,curr_vertex);
	}
}
void copy(Eigen::Affine3d const& mat2, matrix4& mat)
{
	const double* ptr=mat2.data();
	mat._11=ptr[0];
	mat._21=ptr[1];
	mat._31=ptr[2];
	mat._41=ptr[3];
	mat._12=ptr[4];
	mat._22=ptr[5];
	mat._32=ptr[6];
	mat._42=ptr[7];
	mat._13=ptr[8];
	mat._23=ptr[9];
	mat._33=ptr[10];
	mat._43=ptr[11];
	mat._14=ptr[12];
	mat._24=ptr[13];
	mat._34=ptr[14];
	mat._44=ptr[15];
}
void point_to_point(
		Eigen::Matrix<double, 3, Eigen::Dynamic>& source,
		Eigen::Matrix<double, 3, Eigen::Dynamic>& target,
		Eigen::Affine3d& mat2);
void point_to_point(matrixn const& p1, matrixn const& p2, matrix4& mat)
{
	Eigen::Matrix<double, 3, Eigen::Dynamic> source;
	Eigen::Matrix<double, 3, Eigen::Dynamic> target;
	Eigen::Affine3d mat2;
	copy(p1, source);
	copy(p2, target);
	point_to_point(source, target, mat2);
	//copy(source, p1);
	//copy(target, p2);
	copy(mat2, mat);
}
void point_to_point(
		Eigen::Matrix<double, 3, Eigen::Dynamic>& source, 
		Eigen::Matrix<double, 3, Eigen::Dynamic>& target, 
		Eigen::Affine3d& mat2)
{
    //std::cout << "source: " << source.rows() << "x" << source.cols() << std::endl;
	mat2=RigidMotionEstimator::point_to_point(source, target);
}