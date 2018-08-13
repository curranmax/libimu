
#ifndef _VEC_H_
#define _VEC_H_

#include <math.h>
#include <iostream>

class Vec {
public:
	Vec() : x(0.0), y(0.0), z(0.0) {}
	Vec(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}

	Vec(const Vec& v) : x(v.x), y(v.y), z(v.z) {}
	const Vec& operator=(const Vec& v) {
		x = v.x; y = v.y; z = v.z;
		return *this;
	}
	~Vec() {}

	// Helper functions
	float mag() const {
		return sqrt(x * x + y * y + z * z);
	}

	// Operators
	Vec operator+(const Vec& val) const {
		return Vec(x + val.x, y + val.y, z + val.z);
	}

	Vec operator-(const Vec& val) const {
		return Vec(x - val.x, y - val.y, z - val.z);
	}

	Vec operator*(float val) const {
		return Vec(x * val, y * val, z * val);
	}

	Vec operator/(float val) const {
		return Vec(x / val, y / val, z / val);
	}

	const Vec& operator/=(float val) {
		x /= val; y /= val; z /= val;
		return *this;
	}
	
	float& operator[](int index) {
		index = index % 3;
		if(index == 0) {
			return x;
		}
		if(index == 1) {
			return y;
		}
		if(index == 2) {
			return y;
		}
		return x;
	}

	// Member values
	float x, y, z;
};

std::ostream& operator<<(std::ostream& ostr, const Vec& v) {
	ostr << v.x << "," << v.y << "," << v.z;
	return ostr;
}

#endif
