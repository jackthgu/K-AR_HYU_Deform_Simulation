#include "SpacePrimitives.h"
#include <cmath>

space::IPoint& space::IPoint::operator=(const space::IPoint &rhs)
{
	this->x = rhs.x;
	this->y = rhs.y;
	this->z = rhs.z;
	return *this;
}

space::Point2D& space::Point2D::operator=(const space::Point2D &rhs)
{
	this->x = rhs.x;
	this->y = rhs.y;
	return *this;
}

space::Point& space::Point::operator=(const space::Point &rhs)
{
	this->x = rhs.x;
	this->y = rhs.y;
	this->z = rhs.z;
	return *this;
}

space::Point& space::Point::operator+=(const space::Vector &rhs)
{
	this->x += rhs.x;
	this->y += rhs.y;
	this->z += rhs.z;
	return *this;
}

space::Point space::Point::operator+(const space::Vector &rhs)
{
	Point res = *this;
	return res += rhs;
}

space::Point& space::Point::operator-=(const space::Vector &rhs)
{
	this->x -= rhs.x;
	this->y -= rhs.y;
	this->z -= rhs.z;
	return *this;
}

space::Point space::Point::operator-(const space::Vector &rhs)
{
	Point res = *this;
	return res -= rhs;
}

space::Vector space::Point::operator-(const space::Point &rhs)
{
	space::Vector res;
	res.x = this->x - rhs.x;
	res.y = this->y - rhs.y;
	res.z = this->z - rhs.z;
	return res;
}

float space::Point::squared_distance(const space::Point v1, const space::Point v2)
{
	return (v1.x - v2.x) * (v1.x - v2.x) + (v1.y - v2.y) * (v1.y - v2.y) + (v1.z - v2.z) * (v1.z - v2.z);
}

float space::Vector::squared_length()
{
	return (this->x * this->x) + (this->y * this->y) + (this->z * this->z);
}

space::Vector space::Vector::normalize()
{
	float norm = space::Vector::DotProduct(*this, *this);
	return space::Vector(this->x, this->y, this->z) / sqrt(norm);
}

space::Vector& space::Vector::operator+=(const space::Vector& rhs)
{
	this->x += rhs.x;
	this->y += rhs.y;
	this->z += rhs.z;
	return *this;
}

space::Vector space::Vector::operator+(const space::Vector& rhs)
{
	space::Vector res = *this;
	return res += rhs;
}

space::Vector& space::Vector::operator-=(const space::Vector& rhs)
{
	this->x -= rhs.x;
	this->y -= rhs.y;
	this->z -= rhs.z;
	return *this;
}

space::Vector space::Vector::operator-(const space::Vector& rhs)
{
	space::Vector res = *this;
	return res -= rhs;
}

space::Vector& space::Vector::operator*=(const float& rhs)
{
	this->x *= rhs;
	this->y *= rhs;
	this->z *= rhs;
	return *this;
}

space::Vector space::Vector::operator*(const float& rhs)
{
	space::Vector res = *this;
	return res *= rhs;
}

space::Vector& space::Vector::operator/=(const float& rhs)
{
	this->x /= rhs;
	this->y /= rhs;
	this->z /= rhs;
	return *this;
}

space::Vector space::Vector::operator/(const float& rhs)
{
	space::Vector res = *this;
	return res /= rhs;
}


space::Vector space::Vector::CrossProduct(space::Vector v1, space::Vector v2)
{
	return Vector(v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z, v1.x * v2.y - v1.y * v2.x);
}

float space::Vector::DotProduct(space::Vector v1, space::Vector v2)
{
	return (v1.x * v2.x + v1.y * v2.y + v1.z * v2.z);
}

space::Vector space::Vector::GetNormalFromPts(space::Point p, space::Point q, space::Point r)
{
	Vector a = q - p;
	Vector b = r - p;

	return CrossProduct(a, b);
}