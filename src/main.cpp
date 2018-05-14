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

	point compute(float t) const {
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
bezier<point> bezier_derivative(const bezier<point>& bez) {
	bezier<point> b;

	auto n = bez.control_points.size();
	for (auto i = 0; i < n - 1; i++) {
		const auto& curr = bez.control_points[i];
		const auto& next = bez.control_points[i + 1];
		const auto& cp = next - curr;
		b.control_points.push_back(n*cp);
	}

	return b;
}

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
	// Number of segments to split each bezier in when rendering
	const int SEGMENTS_PER_BEZIER = 40;
	const float SEGMENTS_PER_BEZIER_F = float(SEGMENTS_PER_BEZIER); // Utility

	// Absolute value, in world coordinates, of the building
	const float BUILDING_HEIGHT = 20.f;

	// Whether the floors' extrusion follows a bezier path
	const bool DO_VERTICAL_BEZIER = true;

	// Assuming DO_VERTICAL_BEZIER == true, whether floors are perpendicular
	// to the bezier's derivative
	const bool INCLINED_FLOORS = true;

	// Whether the building rotates around itself
	const bool DO_VERTICAL_ROTATION = true;

	// Whether to merge overlapping vertices
	const bool DO_MERGE_SAME_POINTS = false;

	ygl::log_info("creating beziers");
	bezier_sides<ygl::vec2f> bs(
	{
		{-4,-4},{-2,0},{0,-8},{2,-4},
		{1,-1},{2,2},
		{-1,6},{-4,2}
	},
	{
		0,3,5,7
	}
	);

	bezier<ygl::vec3f> vertical_path;
	vertical_path.control_points = {
		{0,0,0},
		{8,5,8},
		{16,10,0},
		{-20,15,-8},
		{0,BUILDING_HEIGHT,0}
	};
	auto vert_derivative = bezier_derivative(vertical_path);

	// Rotation of x and z coordinates, in radiants, around y-axis
	auto rotation_path = [](float t){return 3.f*t;};

	auto compute_position = [&](int side, float x_t, float y_t)->ygl::vec3f {
		auto xz = bs.compute(side, x_t);
		if (DO_VERTICAL_ROTATION) {
			auto dist = ygl::length(xz);
			auto angle = yb::get_angle(xz);
			angle += rotation_path(y_t);
			xz.x = dist*cosf(angle);
			xz.y = dist*sinf(angle);
		}
		auto pos = yb::to_3d(xz);
		if (DO_VERTICAL_BEZIER) {
			if (INCLINED_FLOORS) {
				// Taken with appreciation ( <3 ) from:
				// https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
				auto y_axis = ygl::vec3f{ 0,1,0 };
				auto incl = ygl::normalize(vert_derivative.compute(y_t));

				auto v = ygl::cross(y_axis, incl);
				auto s = ygl::length(v);
				auto c = ygl::dot(y_axis, incl);

				if (c != 0.f) {
					auto vx = ygl::mat3f{ {0,v.z,-v.y},{-v.z,0,v.x},{v.y,-v.x,0} };
					auto R = ygl::identity_mat3f + vx + ((vx*vx)*(1 / (1 + c)));
					pos = R*pos;
				}
			}
			auto offset = vertical_path.compute(y_t);
			pos += offset;
		}
		else {
			pos.y = BUILDING_HEIGHT*y_t;
		}

		return pos;
	};

	// Building 
	ygl::log_info("creating building mesh");
	ygl::shape building_shp;
	building_shp.name = "building_shp";

	for (int y = 0; y < SEGMENTS_PER_BEZIER; y++) {
		auto y_t = float(y) / SEGMENTS_PER_BEZIER_F;
		auto next_y_t = float(y + 1) / SEGMENTS_PER_BEZIER_F;

		for (int i = 0; i < SEGMENTS_PER_BEZIER; i++) {
			auto x_t = float(i) / SEGMENTS_PER_BEZIER_F;
			auto next_x_t = float(i + 1) / SEGMENTS_PER_BEZIER_F;

			for (int side = 0; side < bs.num_sides(); side++) {
				// NB: duplicate points for simplicity
				building_shp.pos.push_back(compute_position(side, x_t, y_t));
				building_shp.pos.push_back(compute_position(side, next_x_t, y_t));
				building_shp.pos.push_back(compute_position(side, next_x_t, next_y_t));
				building_shp.pos.push_back(compute_position(side, x_t, next_y_t));

				int base = building_shp.pos.size() - 4;
				building_shp.quads.push_back({ base, base + 1, base + 2, base + 3 });
			}
		}
	}
	ygl::log_info("converting to triangle mesh");
	yb::to_triangle_mesh(&building_shp);

	// Floor
	std::vector<ygl::vec3f> floor_border;
	for (int side = 0; side < bs.num_sides(); side++) {
		for (int i = 0; i < SEGMENTS_PER_BEZIER; i++) {
			floor_border.push_back(compute_position(side, float(i)/SEGMENTS_PER_BEZIER_F, 0.f));
		}
	}
	std::vector<ygl::vec3f> floor_pos;
	std::vector<ygl::vec3i> floor_triangles;
	std::tie(floor_triangles, floor_pos) = yb::triangulate_opposite(floor_border);
	int bps = building_shp.pos.size();
	building_shp.pos.insert(building_shp.pos.end(), floor_pos.begin(), floor_pos.end());
	for (const auto& t : floor_triangles)
		building_shp.triangles.push_back({ t.x + bps, t.y + bps, t.z + bps });

	// Roof
	floor_border.clear();
	for (int side = 0; side < bs.num_sides(); side++) {
		for (int i = 0; i < SEGMENTS_PER_BEZIER; i++) {
			floor_border.push_back(compute_position(side, float(i) / SEGMENTS_PER_BEZIER_F, 1.f));
		}
	}
	floor_pos.clear();
	floor_triangles.clear();
	std::tie(floor_triangles, floor_pos) = yb::triangulate(floor_border);
	bps = building_shp.pos.size();
	building_shp.pos.insert(building_shp.pos.end(), floor_pos.begin(), floor_pos.end());
	for (int i = bps; i < building_shp.pos.size(); i++) 
		building_shp.pos[i].y = compute_position(0,0,1.f).y;
	for (const auto& t : floor_triangles)
		building_shp.triangles.push_back({ t.x + bps, t.y + bps, t.z + bps });

	if (DO_MERGE_SAME_POINTS) {
		ygl::log_info("merging overlapping points");
		yb::merge_same_points(&building_shp);
	}
	ygl::log_info("recomputing normals");
	ygl::compute_normals(building_shp.triangles, building_shp.pos, building_shp.norm);
	ygl::material* building_mat = ygl::make_matte_material("building_mat", { 1.f,0.3f,0.3f });
	ygl::instance* building_inst = ygl::make_instance("building_inst", &building_shp, building_mat);

	// Windows
	ygl::shape cube_shp;
	cube_shp.name = "cube_shp";
	ygl::make_cube(cube_shp.quads, cube_shp.pos, 5, .1f);
	ygl::material* cube_mat = ygl::make_material("cube_mat");
	std::vector<ygl::instance*> cube_insts;
	ygl::log_info("placing cubes");
	for (int y = 0; y < 10; y++) {
		auto y_t = float(y) / 10.f + 0.05f;

		for (int i = 0; i < 20; i++) {
			auto x_t = float(i) / 20.f;

			for (int side = 0; side < bs.num_sides(); side++) {
				ygl::frame3f cube_frame;
				cube_frame.o = compute_position(side, x_t, y_t);
				cube_insts.push_back(ygl::make_instance(
					"cube_inst" + std::to_string(side) + "_" + std::to_string(i),
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