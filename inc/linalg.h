#ifndef __CRD_LINEAR_ALGEBRA_H__
#define __CRD_LINEAR_ALGEBRA_H__

// basic math

#define CLAMP(N, HIGH, LOW) ( ((N) < (LOW)) ? (N) = (LOW) : ((N) > (HIGH)) ? (N) = (HIGH) : 0 )

#define SIGN(X) ( ((X) > 0) - ((X) < 0) )
#define ABS(X) ( (X) * SIGN(X) )

#define PI 3.1415926535897932384626

/* floating point absolute value (unset sign bit)*/
extern inline double FABS(double X) {
	*((uint_fast64_t*) &X) &= 0x7FFFFFFFFFFFFFFF;
	return X;
}

/* floor function / gauss bracket */
extern inline int FLOOR(double X) {
	int r = (int) X;
	return r - (r > X);
}

/* ceil function */
extern inline int CEIL(double X) {
	int r = (int) X;
	return r + (r < X);
}

/* fast square root (not perfect, but was fun to derive) */
extern inline double FSQRT(double _x) {
	/// initial guess
	uint64_t u = (*((uint64_t*) &_x) >> 1) + 0x1FF8000000000000;
	
	/// newton's method
	*((double*) &u) = 0.5 * (*((double*) &u) + _x / *((double*) &u));
	//*((double*) &u) = 0.5 * (*((double*) &u) + _x / *((double*) &u));
	//*((double*) &u) = 0.5 * (*((double*) &u) + _x / *((double*) &u));
	
	return *((double*) &u);
}

/* fast inverse (also fun to derive, but not very practical) */
extern inline double FINV(double _x) {
	uint64_t u = 0x7FE0000000000000 - *((uint64_t*) &_x);
	return *((double*) &u);
}

/* infamous. */
extern inline double FISQRT(double _x) {
	/// initial guess
	uint64_t u = 0x5FE8000000000000 - (*((uint64_t*) &_x) >> 1);
	
	/// newton's method
	*((double*) &u) *= 1.5 - (0.5 * _x * (*((double*) &u)) * (*((double*) &u)));
	//*((double*) &u) *= 1.5 - (0.5 * _x * (*((double*) &u)) * (*((double*) &u)));
	*((double*) &u) *= 1.5 - (0.5 * _x * (*((double*) &u)) * (*((double*) &u)));
	
	return *((double*) &u);
}

/* fucking math.h doesnt work 70% the time when x = 1.0 because of
 * floating point errors so I had to rip this from nvidia and change
 * 1.0 to 1.0000000001 (very brute force patch) */
double acos_that_actually_works(double x) {
	double negate = (double) (x < 0);
	x = FABS(x);
	double ret = (
		(((-0.0187293 * x + 0.0742610) * x - 0.2121144) * x + 1.5707288)
		* sqrt(1.0000000001 - x)
	);
	ret = ret - 2.0 * negate * ret;
	return negate * 3.14159265358979 + ret;
}

/* minimum and maximums */

int min(int _a, int _b) { return _a + (_b < _a) * (_b - _a); }
int max(int _a, int _b) { return _a + (_b > _a) * (_b - _a); }

int min3(int _a, int _b, int _c) {
	_a += (_b < _a) * (_b - _a);
	_a += (_c < _a) * (_c - _a);
	return _a;
}

int max3(int _a, int _b, int _c) {
	_a += (_b > _a) * (_b - _a);
	_a += (_c > _a) * (_c - _a);
	return _a;
}

double fmin(double _a, double _b) { return _a + (_b < _a) * (_b - _a); }
double fmax(double _a, double _b) { return _a + (_b > _a) * (_b - _a); }

double fmin3(double _a, double _b, double _c) {
	if (_a < _b && _a < _c) return _a;
	else if (_b < _c) return _b;
	else return _c;
}

double fmax3(double _a, double _b, double _c) {
	if (_a > _b && _a > _c) return _a;
	else if (_b > _c) return _b;
	else return _c;
}

// vectors

typedef struct { double x, y, z, w; } vec4f;
typedef struct { double x, y, z;    } vec3f;
typedef struct { double x, y;       } vec2f;

/* vector subtraction */
vec2f sub2f(vec2f _a, vec2f _b) { return (vec2f) {_a.x - _b.x, _a.y - _b.y}; }
vec3f sub3f(vec3f _a, vec3f _b) { return (vec3f) {_a.x - _b.x, _a.y - _b.y, _a.z - _b.z}; }
vec4f sub4f(vec4f _a, vec4f _b) { return (vec4f) {_a.x - _b.x, _a.y - _b.y, _a.z - _b.z, _a.w - _b.w}; }

/* vector addition */
vec2f add2f(vec2f _a, vec2f _b) { return (vec2f) {_a.x + _b.x, _a.y + _b.y}; }
vec3f add3f(vec3f _a, vec3f _b) { return (vec3f) {_a.x + _b.x, _a.y + _b.y, _a.z + _b.z}; }
vec4f add4f(vec4f _a, vec4f _b) { return (vec4f) {_a.x + _b.x, _a.y + _b.y, _a.z + _b.z, _a.w + _b.w}; }

/* scalar multiplication */
vec2f mul2f(vec2f _v, double _c) { return (vec2f) {_v.x * _c, _v.y * _c}; }
vec3f mul3f(vec3f _v, double _c) { return (vec3f) {_v.x * _c, _v.y * _c, _v.z * _c}; }
vec4f mul4f(vec4f _v, double _c) { return (vec4f) {_v.x * _c, _v.y * _c, _v.z * _c, _v.w * _c}; }

/* vector negation */
vec2f neg2f(vec2f _v) { return (vec2f) {-_v.x, -_v.y}; }
vec3f neg3f(vec3f _v) { return (vec3f) {-_v.x, -_v.y, -_v.z}; }
vec4f neg4f(vec4f _v) { return (vec4f) {-_v.x, -_v.y, -_v.z, -_v.w}; }

/* cross product */
vec3f cross3f(vec3f _a, vec3f _b) {
	return (vec3f) {
		_a.y * _b.z - _a.z * _b.y,
		_a.z * _b.x - _a.x * _b.z,
		_a.x * _b.y - _a.y * _b.x,
	};
}

/* returns the resulting z conponent of the cross product after 2d
 * vectors are augmented with zero (notably useful in the world of
 * computer graphics) */
double cross2f(vec3f _a, vec3f _b) { return _a.x * _b.y - _a.y * _b.x; }

/* dot products */
double dot2f(vec2f _a, vec2f _b) { return _a.x * _b.x + _a.y * _b.y; }
double dot3f(vec3f _a, vec3f _b) { return _a.x * _b.x + _a.y * _b.y + _a.z * _b.z; }
double dot4f(vec4f _a, vec4f _b) { return _a.x * _b.x + _a.y * _b.y + _a.z * _b.z + _a.w * _b.w; }

/* magnitudes */
double mag2f(vec2f _v) { return sqrt(_v.x * _v.x + _v.y * _v.y); }
double mag3f(vec3f _v) { return sqrt(_v.x * _v.x + _v.y * _v.y + _v.z * _v.z); }
double mag4f(vec4f _v) { return sqrt(_v.x * _v.x + _v.y * _v.y + _v.z * _v.z + _v.w * _v.w); }

/* normalise */
vec2f norm2f(vec2f _v) {
	double mag = FISQRT(_v.x * _v.x + _v.y * _v.y);
	return (vec2f) {_v.x * mag, _v.y * mag};
}

vec3f norm3f(vec3f _v) {
	double mag = FISQRT(_v.x * _v.x + _v.y * _v.y + _v.z * _v.z);
	return (vec3f) {_v.x * mag, _v.y * mag, _v.z * mag};
}

vec4f norm4f(vec4f _v) {
	double mag = FISQRT(_v.x * _v.x + _v.y * _v.y + _v.z * _v.z + _v.w * _v.w);
	return (vec4f) {_v.x * mag, _v.y * mag, _v.z * mag, _v.w * mag};
}

// quaternions

typedef struct { double r, i, j, k; } qtrn;
typedef qtrn quat;

/* quaternion multiplication (a.k.a. the hamilton product) */
qtrn qmul(qtrn _a, qtrn _b) {
	return (qtrn) {
		_a.r * _b.r - _a.i * _b.i - _a.j * _b.j - _a.k * _b.k,
		_a.r * _b.i + _a.i * _b.r + _a.j * _b.k - _a.k * _b.j,
		_a.r * _b.j - _a.i * _b.k + _a.j * _b.r + _a.k * _b.i,
		_a.r * _b.k + _a.i * _b.j - _a.j * _b.i + _a.k * _b.r,
	};
}

/* returns a rotation quaternion representing a rotaion of
 * `_phi` degrees about the axis defined by `_v` */
qtrn make_qrot(vec3f _v, double _phi) {
	double sin_phi = sin(_phi * 0.5);
	double cos_phi = cos(_phi * 0.5);
	
	return (qtrn) {
		cos_phi,
		sin_phi * _v.x,
		sin_phi * _v.y,
		sin_phi * _v.z,
	};
}

/* rotates a vector `_p` by conjugation with the rotation
 * quaternion `_q` */
vec3f qrot(qtrn _q, vec3f _p) {
	qtrn q_inv = (qtrn) {_q.r, -_q.i, -_q.j, -_q.k};
	qtrn p = (qtrn) {0, _p.x, _p.y, _p.z};
	p = qmul(qmul(_q, p), q_inv);
	
	return (vec3f) {p.i, p.j, p.k};
}

// matrices

typedef struct { double get[16];    } mat4f;
typedef struct { double get[9];     } mat3f;
typedef struct { double get[4];     } mat2f;

vec4f mmul_vec4f(mat4f* _m, vec4f _v) {
	return (vec4f) {
		_m->get[0 ] * _v.x + _m->get[1 ] * _v.y + _m->get[2 ] * _v.z + _m->get[3 ] * _v.w,
		_m->get[4 ] * _v.x + _m->get[5 ] * _v.y + _m->get[6 ] * _v.z + _m->get[7 ] * _v.w,
		_m->get[8 ] * _v.x + _m->get[9 ] * _v.y + _m->get[10] * _v.z + _m->get[11] * _v.w,
		_m->get[12] * _v.x + _m->get[13] * _v.y + _m->get[14] * _v.z + _m->get[15] * _v.w,
	};
}

vec3f mmul_vec3f(mat3f _m, vec3f _v) {
	return (vec3f) {
		_m.get[0] * _v.x + _m.get[1] * _v.y + _m.get[2] * _v.z,
		_m.get[3] * _v.x + _m.get[4] * _v.y + _m.get[5] * _v.z,
		_m.get[6] * _v.x + _m.get[7] * _v.y + _m.get[8] * _v.z,
	};
}

vec2f mmul_vec2f(mat2f _m, vec2f _v) {
	return (vec2f) {
		_m.get[0] * _v.x + _m.get[1] * _v.y,
		_m.get[2] * _v.x + _m.get[3] * _v.y,
	};
}

mat4f mT4f(mat4f _m) {
	return (mat4f) {{
		_m.get[0], _m.get[4], _m.get[8 ], _m.get[12],
		_m.get[1], _m.get[5], _m.get[9 ], _m.get[13],
		_m.get[2], _m.get[6], _m.get[10], _m.get[14],
		_m.get[3], _m.get[7], _m.get[11], _m.get[15],
	}};
}

mat3f mT3f(mat3f _m) {
	return (mat3f) {{
		_m.get[0], _m.get[3], _m.get[6],
		_m.get[1], _m.get[4], _m.get[7],
		_m.get[2], _m.get[5], _m.get[8],
	}};
}

mat2f mT2f(mat2f _m) {
	return (mat2f) {{
		_m.get[0], _m.get[2],
		_m.get[1], _m.get[3],
	}};
}

void mmul4f(mat4f* _r, mat4f* _a, mat4f* _b) {
	*_r = (mat4f) {{
		_a->get[0] * _b->get[0 ] + _a->get[4] * _b->get[1 ] + _a->get[8 ] * _b->get[2 ] + _a->get[12] * _b->get[3 ],
		_a->get[1] * _b->get[0 ] + _a->get[5] * _b->get[1 ] + _a->get[9 ] * _b->get[2 ] + _a->get[13] * _b->get[3 ],
		_a->get[2] * _b->get[0 ] + _a->get[6] * _b->get[1 ] + _a->get[10] * _b->get[2 ] + _a->get[14] * _b->get[3 ],
		_a->get[3] * _b->get[0 ] + _a->get[7] * _b->get[1 ] + _a->get[11] * _b->get[2 ] + _a->get[15] * _b->get[3 ],
		_a->get[0] * _b->get[4 ] + _a->get[4] * _b->get[5 ] + _a->get[8 ] * _b->get[6 ] + _a->get[12] * _b->get[7 ],
		_a->get[1] * _b->get[4 ] + _a->get[5] * _b->get[5 ] + _a->get[9 ] * _b->get[6 ] + _a->get[13] * _b->get[7 ],
		_a->get[2] * _b->get[4 ] + _a->get[6] * _b->get[5 ] + _a->get[10] * _b->get[6 ] + _a->get[14] * _b->get[7 ],
		_a->get[3] * _b->get[4 ] + _a->get[7] * _b->get[5 ] + _a->get[11] * _b->get[6 ] + _a->get[15] * _b->get[7 ],
		_a->get[0] * _b->get[8 ] + _a->get[4] * _b->get[9 ] + _a->get[8 ] * _b->get[10] + _a->get[12] * _b->get[11],
		_a->get[1] * _b->get[8 ] + _a->get[5] * _b->get[9 ] + _a->get[9 ] * _b->get[10] + _a->get[13] * _b->get[11],
		_a->get[2] * _b->get[8 ] + _a->get[6] * _b->get[9 ] + _a->get[10] * _b->get[10] + _a->get[14] * _b->get[11],
		_a->get[3] * _b->get[8 ] + _a->get[7] * _b->get[9 ] + _a->get[11] * _b->get[10] + _a->get[15] * _b->get[11],
		_a->get[0] * _b->get[12] + _a->get[4] * _b->get[13] + _a->get[8 ] * _b->get[14] + _a->get[12] * _b->get[15],
		_a->get[1] * _b->get[12] + _a->get[5] * _b->get[13] + _a->get[9 ] * _b->get[14] + _a->get[13] * _b->get[15],
		_a->get[2] * _b->get[12] + _a->get[6] * _b->get[13] + _a->get[10] * _b->get[14] + _a->get[14] * _b->get[15],
		_a->get[3] * _b->get[12] + _a->get[7] * _b->get[13] + _a->get[11] * _b->get[14] + _a->get[15] * _b->get[15],
	}};
}

#endif