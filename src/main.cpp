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
struct bezier {
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

template<class Point>
class bezier_sides {
	std::vector<bezier<Point>> sides;

public:
	bezier_sides(const std::vector<Point>& points, const std::vector<size_t> corner_ids) {
		// Check: corners' IDs have to be strictly increasing
		for (int i = 0; i < corner_ids.size() - 1; i++) {
			if (corner_ids[i] >= corner_ids[i + 1]) throw std::runtime_error("Invalid corner ids");
		}

		// Check: corners' IDs in valid range
		if (corner_ids.front() < 0 || corner_ids.back() >= points.size())
			throw std::runtime_error("Invalid corner ids");

		for (int i = 0; i < corner_ids.size()-1 ;i++) {
			bezier<Point> b;
			for (int j = corner_ids[i]; j <= corner_ids[i + 1]; j++) {
				b.control_points.push_back(points[j]);
			}
			sides.push_back(b);
		}
		bezier<Point> b;
		for (int i = corner_ids.back(); i < points.size(); i++)
			b.control_points.push_back(points[i]);
		for (int i = 0; i <= corner_ids.front(); i++)
			b.control_points.push_back(points[i]);
		sides.push_back(b);
	}

	int num_sides() {
		return sides.size();
	}

	Point compute(size_t side, float t) {
		return sides[side].compute(t);
	}
};

int main(int argc, char** argv) {
	ygl::log_info("creating beziers");
	bezier_sides<ygl::vec2f> bs(
	{
		{0,0},{2,4},{4,-4},{6,0},
		{5,3},{6,6},
		{3,10},{0,6}
	},
	{
		0,3,5,7
	}
	);

	ygl::shape cube_shp;
	cube_shp.name = "cube_shp";
	ygl::make_cube(cube_shp.quads, cube_shp.pos, 5, .1f);
	ygl::material* cube_mat = ygl::make_material("cube_mat");
	std::vector<ygl::instance*> cube_insts;
	ygl::log_info("placing cubes");
	for (int y = 0; y < 10; y++) {
		for (int j = 0; j < bs.num_sides(); j++) {
			for (int i = 0; i < 20; i++) {
				ygl::frame3f cube_frame;
				cube_frame.o = yb::to_3d(bs.compute(j, float(i) / 20.f), float(2*y));
				cube_insts.push_back(ygl::make_instance(
					"cube_inst" + std::to_string(j) + "_" + std::to_string(i),
					&cube_shp,
					cube_mat,
					cube_frame
				));
			}
		}
	}
	ygl::scene scene;
	ygl::log_info("saving scene");
	for (auto inst : cube_insts) yb::add_to_scene(&scene, inst);
	ygl::save_scene("out.obj", &scene);

	return 0;
}