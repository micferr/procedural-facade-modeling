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

template<class Point>
class bezier {
public:
	std::vector<Point> control_points;

	Point compute(float t) {
		auto n = control_points.size();
		if (n == 0) return { 0,0 };

		// De Casteljau
		auto points = control_points;
		std::vector<Point> points2;
		while (points.size() > 1) {
			for (int i = 0; i < points.size() - 1; i++) {
				points2.push_back({ points[i] + (points[i + 1] - points[i])*t });
			}
			points = points2;
			points2.clear();
		}
		return points[0];
	}
};

int main(int argc, char** argv) {
	bezier<ygl::vec2f> my_bezier;
	my_bezier.control_points.push_back({ 0,0 });
	my_bezier.control_points.push_back({ 2,4 });
	my_bezier.control_points.push_back({ 4,-4 });
	my_bezier.control_points.push_back({ 6,0 });
	for (int i = 0; i < 20; i++) {
		float perc = float(i) / 20.f;
		printf("%i\t%f\t%f\n", i + 1, my_bezier.compute(perc).x, my_bezier.compute(perc).y);
	}

	ygl::shape sphere_shp;
	ygl::make_cube(sphere_shp.quads, sphere_shp.pos, 5, .1f);
	ygl::material* sphere_mat = ygl::make_material("sphere_mat");
	std::vector<ygl::instance*> sphere_insts;
	for (int i = 0; i < 20; i++) {
		ygl::frame3f sphere_frame;
		sphere_frame.o = yb::to_3d(my_bezier.compute(float(i) / 20.f));
		sphere_insts.push_back(ygl::make_instance(
			"sphere_inst" + std::to_string(i),
			&sphere_shp,
			sphere_mat,
			sphere_frame
		));
	}
	ygl::scene scene;
	for (auto inst : sphere_insts) yb::add_to_scene(&scene, inst);
	ygl::save_scene("ciao.obj", &scene);

	return 0;
}