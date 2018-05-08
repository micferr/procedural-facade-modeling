#include <cstdio>
#include <iostream>
#include <string>
#include <ctime>

#include "yocto\yocto_image.h"
#include "yocto\yocto_math.h"
#include "yocto\yocto_obj.h"
#include "yocto\yocto_scene.h"
#include "yocto\yocto_shape.h"
#include "yocto\yocto_utils.h"

#include "geom_utils.h"
#include "prob_utils.h"
#include "yocto_utils.h"

long fact(long n) {
	long res = 1;
	for (int i = 2; i <= n; i++) {
		res *= i;
	}
	return res;
}

long binomial(long n, long k) {
	if (n >= k && k >= 0) {
		return fact(n) / (fact(k) * fact(n - k));
	}
	else return 0;
}

template<class Point>
class bezier {
public:
	std::vector<Point> control_points;

	Point compute(float t) {
		auto n = control_points.size();
		Point p;
		// Raw Bezier formula - This is bad! :'(
		for (int i = 0; i <= n; i++) {
			p += (control_points[i] * binomial(n, i) * powf(1.f - t, (float)n - i) * powf(t, (float)i));
		}
		return p;
	}
};

int main(int argc, char** argv) {
	bezier<ygl::vec2f> my_bezier;
	for (auto i = 0; i <= 6; i++) {
		auto j = float(i)*ygl::pi/6;
		my_bezier.control_points.push_back({ j, sin(j) });
	}
	for (auto i = 0; i <= 30; i++) {
		auto j = float(i)*ygl::pi / 30;
		printf("%d\t| %f\t%f\t%f\t%f\n", i, j, sin(j), my_bezier.compute(i/30.f).x, my_bezier.compute(i/30.f).y);
	}
	return 0;
}