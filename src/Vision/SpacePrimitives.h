#pragma once

namespace space {
	class IPoint;

	class Point2D;

	class Point;

	class Vector;


	class IPoint {
	public:
		int x, y, z;

		IPoint() : x(0), y(0), z(0) {}

		IPoint(int _x, int _y, int _z) : x(_x), y(_y), z(_z) {}

		IPoint &operator=(const IPoint &rhs);
	};

	class Point2D {
	public:
		float x, y;

		Point2D() : x(0.0f), y(0.0f) {}

		Point2D(float _x, float _y) : x(_x), y(_y) {}

		Point2D &operator=(const Point2D &rhs);
	};

	class Point {
	public:
		float x, y, z;

		Point() : x(0.0f), y(0.0f), z(0.0f) {}

		Point(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}

		Point &operator=(const Point &rhs);

		Point &operator+=(const Vector &rhs);

		Point operator+(const Vector &rhs);

		Point &operator-=(const Vector &rhs);

		Point operator-(const Vector &rhs);

		Vector operator-(const Point &rhs);

		static float squared_distance(const Point v1, const Point v2);
	};

	class Vector {
	public:
		float x, y, z;

		Vector() : x(0.0f), y(0.0f), z(0.0f) {}

		Vector(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}

		float squared_length();

		Vector normalize();

		Vector &operator+=(const Vector &rhs);

		Vector operator+(const Vector &rhs);

		Vector &operator-=(const Vector &rhs);

		Vector operator-(const Vector &rhs);

		Vector &operator*=(const float &rhs);

		Vector operator*(const float &rhs);

		Vector &operator/=(const float &rhs);

		Vector operator/(const float &rhs);

		static Vector CrossProduct(Vector v1, Vector v2);

		static float DotProduct(Vector v1, Vector v2);

		static Vector GetNormalFromPts(Point p, Point q, Point r);
	};


	class Line {
	public:
		Point ori;
		Vector dir;

		Line() {
			this->ori = Point();
			this->dir = Vector();
		}

		Line(Point _ori, Vector _dir) {
			this->ori = _ori;
			this->dir = _dir;
		}
	};

	typedef struct {
		float t; // direction coefficient
		Line ray; // ray
		IPoint idx; // vertex indices
		Point v[3]; // vertices
		float coeffs[3]; // barycentric coefficients
		int fIdx; // feature index
		float d; // real-Depth
		Point r;
	} Barycentric;
};