// (c) Simone Guggiari 2019

#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <string>
#include <climits>

using namespace std;

const double eps = 1e-5;
const double deg2rad = 0.0174532925199;


//_________________________________________________________________________________

struct vec {
	double x, y, z; // also used for rgb

	vec(double _x = 0, double _y = 0, double _z = 0) : x(_x), y(_y), z(_z) {}

	vec operator+(const vec& o) const { return vec(x + o.x, y + o.y, z + o.z); }
	vec operator-(const vec& o) const { return vec(x - o.x, y - o.y, z - o.z); }
	vec operator*(double s) const { return vec(x*s, y*s, z*s); }
	vec operator/(double s) const { return vec(x / s, y / s, z / s); }

	double operator*(const vec& o) const { return x * o.x + y * o.y + z * o.z; } // dot product
	vec operator%   (const vec& o) const { return vec(y*o.z - z * o.y, z*o.x - x * o.z, x*o.y - y * o.x); } // cross product

	double norm()  const { return sqrt(x*x + y * y + z * z); }
	double norm2() const { return x * x + y * y + z * z; }
	void normalize() { double n = sqrt(x*x + y * y + z * z); x /= n, y /= n, z /= n; }

	friend ostream& operator<<(ostream& os, const vec& v);
	double r() const { return x; } double g() const { return y; } double b() const { return z; }
};

ostream& operator<<(ostream& os, const vec& v) { os << "(" << v.x << ", " << v.y << ", " << v.z << ")"; return os; }



struct  sph {
	vec p, c; // position, color
	double r; // radius
	sph() : p(), c(), r(0) {}
	sph(vec _p, double _r, vec _c = {}) : p(_p), r(_r), c(_c) { c = c / 255; }
};



struct ray {
	vec o, d;
	ray() : o(), d() {}
	ray(vec _o, vec _d) : o(_o), d(_d) { d.normalize(); }

	vec p(double t) { return o + d * t; } // point along ray

	double intersect(const sph& s) const { // assumes d is unit
		double b = d * (o - s.p);
		double c = (o - s.p)*(o - s.p) - (s.r*s.r);
		if (b*b - c < 0) return -1.0; // no intersection
		return -b - sqrt(b*b - c);
	}

	double intersect(const vector<sph>& ss) const {
		double ans = INT_MAX;
		for (const sph& s : ss) {
			double t = intersect(s);
			if (t >= 0 - eps) ans = min(ans, t);
		}
		return ans == INT_MAX ? -1 : ans;
	}
};




struct cam {
	vec o, d, r, u; // origin, direction, right, up (transform directions)
	double fov, a, s; // fov, aspect ratio, scaling factor (based on fov)

	cam(vec _o, vec _d, double _f, double _a) : o(_o), d(_d), fov(_f), a(_a) {
		d.normalize();
		r = d % vec(0, 0, 1);
		u = r % d;
		s = tan(fov / 2 * deg2rad); // scaling factor to compute ray
	}
	ray getRay(double x, double y) { // x,y in [-1, 1]
		vec dir = d + r * (x*s) + u * (y*s*a);
		return ray(o, dir);
	}
};


//_________________________________________________________________________________


const vector<sph> scene = {
	{ { 0, 0, 1e5 + 2.65 }, 1e5,{ 255, 255, 255 } },//U
	{ { 0, 0, -1e5 }, 1e5,{ 255, 255, 255 } },//D
	{ { 0, 1e5 + 1.4 , 0 }, 1e5,{ 255, 255, 255 } },//B
	{ { -1e5 - 1.6, 0, 0 }, 1e5,{ 183, 108, 115 } },//L
	{ { 1e5 + 1.6 , 0, 0 }, 1e5,{ 114, 108, 182 } },//R
	
	{ { -0.75, 0.25, 0.5 }, 0.5,{ 255, 255, 255 } },//sph1 (mirror)
	{ { 0.75, -0.5, 0.5 }, 0.5, { 255, 255, 255 } },//sph2 (glass)
	
	{ { 10 + 2.65, 0, 0 }, 10,{ 255, 255, 255 } },//light
};

cam mycam = cam({ 0, -5.75, 1.75 }, { 0, 0.9976, -0.06976 }, 50, 0.75); // cos(-4), sin(-4);


//_________________________________________________________________________________


//goal 1: save image done
//goal 2: save depth buffer done

double clamp(double x, double a, double b) { return max(a, min(x, b)); } // x in [a,b]

int toCol(double d) {
	int v = static_cast<int>(round(d * 255));
	return clamp(v, 0, 255);
}

void writePpmImage(const string& fileName, int W, int H, const vector<vec>& pixels) {
	ofstream img(fileName);
	img << "P3" << endl << W << " " << H << endl << "255" << endl; // header

	for (int y = 0; y<H; ++y) {
		for (int x = 0; x<W; ++x) {
			const vec& p = pixels[y*W + x];
			img << toCol(p.r()) << " " << toCol(p.g()) << " " << toCol(p.b()) << endl;
		}
	}
	//system("open picture.ppm"); 
}


//_________________________________________________________________________________


vec raytracer(const ray& r) { // HERE IS THE RAYTRACER
	//computes the color for a given ray
	//white = far, black = close
	const double mind = -1, maxd = 10;//3.4, maxd = 7.6; // based on the scene

	//compute first intersection
	double t = r.intersect(scene);

	double d = 1-clamp((t - mind) / (maxd - mind), 0, 1);
	return vec(d, d, d);
}



//_________________________________________________________________________________

int main() {

	cout << "Simple Raytracer" << endl << "(c) Simone Guggiari 2019" << endl << endl;
	cout << "rendering..." << endl;

	const int W = 1024 / 2, H = 768 / 2;
	vector<vec> pixels(W*H);


	for (int y = 0; y<H; ++y) {
		for (int x = 0; x<W; ++x) {
			//compute pixel
			ray r = mycam.getRay(double(x - W / 2) / (W / 2), -double(y - H / 2) / (H / 2));
			pixels[y*W + x] = raytracer(r);
		}
	}
	writePpmImage("picture.ppm", W, H, pixels);

	cout << "done" << endl;
	return 0;
}