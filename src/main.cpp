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

template<class point>
struct bezier {
	std::vector<point> control_points;

	point compute(float t) {
		auto n = control_points.size();
		if (n == 0) return { 0,0 };

		// De Casteljau
		auto points = control_points;
		std::vector<point> points2;
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

template<class point>
class bezier_sides {
	std::vector<bezier<point>> sides;

public:
	bezier_sides(const std::vector<point>& points, const std::vector<size_t> corner_ids) {
		// Check: corners' IDs have to be strictly increasing
		for (int i = 0; i < corner_ids.size() - 1; i++) {
			if (corner_ids[i] >= corner_ids[i + 1]) throw std::runtime_error("Invalid corner ids");
		}

		// Check: corners' IDs in valid range
		if (corner_ids.front() < 0 || corner_ids.back() >= points.size())
			throw std::runtime_error("Invalid corner ids");

		for (int i = 0; i < corner_ids.size()-1 ;i++) {
			bezier<point> b;
			for (int j = corner_ids[i]; j <= corner_ids[i + 1]; j++) {
				b.control_points.push_back(points[j]);
			}
			sides.push_back(b);
		}
		bezier<point> b;
		for (int i = corner_ids.back(); i < points.size(); i++)
			b.control_points.push_back(points[i]);
		for (int i = 0; i <= corner_ids.front(); i++)
			b.control_points.push_back(points[i]);
		sides.push_back(b);
	}

	int num_sides() {
		return sides.size();
	}

	point compute(size_t side, float t) {
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

	const int SEGMENTS_PER_BEZIER = 100;
	const float SEGMENTS_PER_BEZIER_F = float(SEGMENTS_PER_BEZIER);

	// Building 
	ygl::shape building_shp;
	building_shp.name = "building_shp";
	for (int side = 0; side < bs.num_sides(); side++) {
		for (int i = 0; i < SEGMENTS_PER_BEZIER; i++) {
			// NB: duplicate points for simplicity
			building_shp.pos.push_back(yb::to_3d(bs.compute(side, float(i) / SEGMENTS_PER_BEZIER_F)));
			building_shp.pos.push_back(yb::to_3d(bs.compute(side, float(i+1) / SEGMENTS_PER_BEZIER_F)));
			building_shp.pos.push_back(yb::to_3d(bs.compute(side, float(i+1) / SEGMENTS_PER_BEZIER_F), 20.f));
			building_shp.pos.push_back(yb::to_3d(bs.compute(side, float(i) / SEGMENTS_PER_BEZIER_F), 20.f));
			int base = side * 4 * SEGMENTS_PER_BEZIER_F + i * 4;
			building_shp.quads.push_back({ base, base + 1, base + 2, base + 3 });
		}
	}
	yb::to_triangle_mesh(&building_shp);
	// Floor
	std::vector<ygl::vec3f> floor_border;
	for (int i = 0; i < building_shp.pos.size(); i += 4) {
		floor_border.push_back(building_shp.pos[i]);
	}
	std::vector<ygl::vec3f> floor_pos;
	std::vector<ygl::vec3i> floor_triangles;
	std::tie(floor_triangles, floor_pos) = yb::triangulate_opposite(floor_border);
	int bps = building_shp.pos.size();
	building_shp.pos.insert(building_shp.pos.end(), floor_pos.begin(), floor_pos.end());
	for (const auto& t : floor_triangles)
		building_shp.triangles.push_back({ t.x + bps, t.y + bps, t.z + bps });
	ygl::material* building_mat = ygl::make_matte_material("building_mat", { 1.f,0.3f,0.3f });
	ygl::instance* building_inst = ygl::make_instance("building_inst", &building_shp, building_mat);
	// Roof
	for (const auto& p : floor_pos)
		building_shp.pos.push_back(p + ygl::vec3f{0, 20.f, 0});
	bps += floor_pos.size();
	for (const auto& t : floor_triangles)
		building_shp.triangles.push_back({t.z + bps, t.y + bps, t.x + bps});

	ygl::compute_normals(building_shp.triangles, building_shp.pos, building_shp.norm);

	// Windows
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
				cube_frame.o = yb::to_3d(bs.compute(j, float(i) / 20.f), 0.5f + float(2*y));
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
	yb::add_to_scene(&scene, building_inst);
	ygl::save_scene("out.obj", &scene);

	return 0;
}