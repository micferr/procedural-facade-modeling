//
// LICENSE:
//
// Copyright (c) 2016 -- 2018 Fabio Pellacini
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

/// Includes

#include <cstdio>
#include <iostream>
#include <string>
#include <ctime>
#include <set>

#include "yocto_utils.h"
#include "apps/yapp_ui.h"
using namespace std::literals;

#include "geom_bool.h"
#include "geom_utils.h"
#include "tagged_shape.h"

#include <gurobi_c++.h>
#include "optimizer.h"

/// Globals

GRBEnv env = GRBEnv();

/// App settings

// Whether to print debug info
bool DEBUG_PRINT = true;

// Number of segments to split each bezier in when rendering
int SEGMENTS_PER_BEZIER;
// Utility
#define SEGMENTS_PER_BEZIER_F (float(SEGMENTS_PER_BEZIER))

// Building's height when DO_VERTICAL_BEZIER == false
float BUILDING_HEIGHT;

// Whether the floors' extrusion follows a bezier path
bool DO_VERTICAL_BEZIER;

// Assuming DO_VERTICAL_BEZIER == true, whether floors are perpendicular
// to the bezier's derivative
// (DEPRECATED)
bool INCLINED_FLOORS;

// Whether the building rotates around itself
bool DO_VERTICAL_ROTATION;

// Whether to merge overlapping vertices
bool DO_MERGE_SAME_POINTS;

/// Struct, classes and functions

template<class point>
struct bezier {
	std::vector<point> control_points;

	bezier() = default;
	bezier(const std::vector<point>& points) : control_points(points) {}

	// Approximation, the higher the number of segments, the better the result.
	// Compute (n+1) points of the bezier (equidistant t's) and sum the distances
	// between each consecutive pair of points
	float length(int num_segments = SEGMENTS_PER_BEZIER) {
		auto l = 0.f;
		auto p = compute(0.f);
		for (auto i = 1; i <= num_segments; i++) {
			auto p2 = compute(float(i) / num_segments);
			l += ygl::length(p2 - p);
			p = p2;
		}
		return l;
	}

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
public:
	std::vector<bezier<point>> sides;

	bezier_sides(const std::vector<point>& points, const std::vector<size_t> corner_ids) {
		// Check: corners' IDs have to be strictly increasing
		for (int i = 0; i < corner_ids.size() - 1; i++) {
			if (corner_ids[i] >= corner_ids[i + 1]) throw std::runtime_error("Invalid corner ids");
		}

		// Check: corners' IDs in valid range
		if (corner_ids.front() < 0 || corner_ids.back() >= points.size())
			throw std::runtime_error("Invalid corner ids");

		for (int i = 0; i < corner_ids.size() - 1;i++) {
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

/// Building data

yb::tagged_shape building_shp;

bezier<ygl::vec3f> vertical_path({
	{ 0,0,0 },
	{ 20,10,0 },
	{ 20,20,20 },
	{ 10,35,-10 }
});
auto vert_derivative = bezier_derivative(vertical_path);

bezier_sides<ygl::vec2f> bs(
{ 
	{-5,-5},{0,-2},{5,-5},{8,0},{5,5},{0,2},{-5,5}//,{-8,0}
}, 
{ 0,2,4,6 }
);

// Rotation of x and z coordinates, in radiants, around y-axis
auto rotation_path = [](float t) {return 3.f*t;};

// Computes the position, in world coordinates, of a layout element on the
// building described by bs, BUILDING_HEIGHT, DO_VERTICAL_[BEZIER/ROTATION], INCLINED_FLOORS
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
			// Taken from:
			// https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
			auto y_axis = ygl::vec3f{ 0,1,0 };
			auto incl = ygl::normalize(vert_derivative.compute(y_t));

			auto v = ygl::cross(y_axis, incl);
			auto s = ygl::length(v);
			auto c = ygl::dot(y_axis, incl);

			if (c != 0.f) {
				auto vx = ygl::mat3f{ { 0,v.z,-v.y },{ -v.z,0,v.x },{ v.y,-v.x,0 } };
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

// Computes the world coordinates and face normal of a layout position for a given building.
// Returned values are operation success (it may fail e.g. if there's no face with 
// the required face id), world coordinates and shape normal on said position.
//
// todo: use optionals? (needs C++17)
std::tuple<bool, ygl::vec3f, ygl::vec3f> compute_layout_position(
	yb::tagged_shape& ts,
	int face_id,
	ygl::vec2f face_coord
) {
	if (ts.face_ids().count(face_id) == 0) return { false,{},{} };

	// todo: more efficient algorithm
	for (const auto& t : ts.triangles) { // Assuming a triangle mesh
		const auto& tag1 = ts.vertex_tags[t.x];
		const auto& tag2 = ts.vertex_tags[t.y];
		const auto& tag3 = ts.vertex_tags[t.z];

		if (tag1.face_id == face_id) { 
			const auto& c1 = tag1.face_coord;
			const auto& c2 = tag2.face_coord;
			const auto& c3 = tag3.face_coord;

			// see http://blackpawn.com/texts/pointinpoly/
			const auto v2 = face_coord - c1;
			const auto v0 = c2 - c1;
			const auto v1 = c3 - c1;
			using ygl::dot;
			auto u =
				(dot(v1, v1)*dot(v2, v0) - dot(v1, v0)*dot(v2, v1)) /
				(dot(v0, v0)*dot(v1, v1) - dot(v0, v1)*dot(v1, v0));
			auto v =
				(dot(v0, v0)*dot(v2, v1) - dot(v0, v1)*dot(v2, v0)) /
				(dot(v0, v0)*dot(v1, v1) - dot(v0, v1)*dot(v1, v0));
			if (u >= 0 && v >= 0 && u + v <= 1.f) {
				const auto& p1 = ts.pos[t.x];
				const auto& p2 = ts.pos[t.y];
				const auto& p3 = ts.pos[t.z];

				const auto pos = p1 + (p2 - p1)*u + (p3 - p1)*v;
				const auto norm = ygl::cross(p2 - p1, p3 - p2);
				return { true, pos, norm };
			}
		}
	}

	return { false,{},{} };
}

// Interactive viewer options
std::map<int, bool> is_face_highlighted;
bool show_facecoord_heatmap = false;
bool show_pattern_2 = false;
bool show_border_highlighting = false;

void inflate_by_facecoord_heatmap(float bump_factor) {
	for (auto i = 0; i < building_shp.pos.size(); i++) {
		auto intensity = 1.f - std::max(
			fabs(0.5f - building_shp.vertex_tags[i].face_coord.x),
			fabs(0.5f - building_shp.vertex_tags[i].face_coord.y)
		)*2.f;
		building_shp.pos[i] += building_shp.norm[i] * intensity * bump_factor;
	}
}

void reset_colors() {
	for (auto& kv : is_face_highlighted) kv.second = false;
	show_facecoord_heatmap = false;
	show_pattern_2 = false;
	for (auto& c : building_shp.color) c = ygl::vec4f{ 1,1,1,1 };
}

// Application state
struct app_state {
	ygl::scene* scn = nullptr;
	ygl::camera* cam = nullptr;
	std::string filename;
	std::string imfilename;
	std::string outfilename;

	int resolution = 512;       // image resolution
	bool wireframe = false;     // wireframe drawing
	bool edges = false;         // draw edges
	float edge_offset = 0.01f;  // offset for edges
	bool cutout = false;        // draw with binary transparency
	bool eyelight = false;      // camera light mode
	float exposure = 0;         // exposure
	float gamma = 2.2f;         // gamma
	ygl::vec4b background = { 222, 222, 222, 255 };  // background
	ygl::vec3f ambient = { 0, 0, 0 };              // ambient lighting
	bool cull_backface = false;                  // culling back face

	ygl::glsurface_program prog;

	bool navigation_fps = false;
	ygl::scene_selection selection = {};
	std::vector<ygl::scene_selection> update_list;
	float time = 0;
	std::string anim_group = "";
	ygl::vec2f time_range = ygl::zero2f;
	bool animate = false;
	bool quiet = false;
	bool screenshot_and_exit = false;
	bool no_glwidgets = false;
	std::unordered_map<std::string, std::string> inspector_highlights;

	~app_state() {
		if (scn) delete scn;
	}
};

// draw with shading
inline void draw(ygl::glwindow* win, app_state* app) {
	auto framebuffer_size = get_glwindow_framebuffer_size(win);
	app->resolution = framebuffer_size.y;

	static auto last_time = 0.0f;
	for (auto& sel : app->update_list) {
		if (sel.as<ygl::texture>()) {
			ygl::update_gldata(sel.as<ygl::texture>());
		}
		if (sel.as<ygl::shape>()) { ygl::update_gldata(sel.as<ygl::shape>()); }
		if (sel.as<ygl::node>() || sel.as<ygl::animation>() ||
			app->time != last_time) {
			ygl::update_transforms(app->scn, app->time, app->anim_group);
			last_time = app->time;
		}
		if (sel.as<ygl::shape>() || sel.as<ygl::material>() ||
			sel.as<ygl::node>()) {
			ygl::update_lights(app->scn);
			if (app->scn->lights.empty()) app->eyelight = true;
		}
	}
	app->update_list.clear();

	ygl::clear_glbuffers(app->background);
	ygl::enable_gldepth_test(true);
	ygl::enable_glculling(app->cull_backface);
	ygl::draw_glscene(app->scn, app->cam, app->prog, framebuffer_size,
		app->selection.ptr, app->eyelight, app->wireframe, app->edges,
		app->cutout, app->exposure, app->gamma, app->cull_backface);

	if (app->no_glwidgets) {
		ygl::swap_glwindow_buffers(win);
		return;
	}

	if (ygl::begin_glwidgets_frame(win, "yiview")) {
		ygl::draw_glwidgets_text(win, "scene", app->filename);
		if (ygl::draw_glwidgets_button(win, "new")) {
			delete app->scn;
			ygl::clear_gldata(app->scn);
			app->scn = new ygl::scene();
			ygl::update_gldata(app->scn);
		}
		ygl::continue_glwidgets_line(win);
		if (ygl::draw_glwidgets_button(win, "load")) {
			ygl::clear_gldata(app->scn);
			delete app->scn;
			app->scn = ygl::load_scene(app->filename, {});
			ygl::update_gldata(app->scn);
		}
		ygl::continue_glwidgets_line(win);
		if (ygl::draw_glwidgets_button(win, "save")) {
			ygl::save_scene(app->filename, app->scn, {});
		}
		ygl::continue_glwidgets_line(win);
		if (ygl::draw_glwidgets_button(win, "screenshot")) {
			auto width = 0, height = 0;
			auto img = std::vector<ygl::vec4b>();
			ygl::take_glwindow_screenshot4b(win, width, height, img);
			ygl::save_image4b(app->imfilename, width, height, img);
		}
		ygl::continue_glwidgets_line(win);
		if (ygl::draw_glwidgets_button(win, "print stats"))
			ygl::print_stats(app->scn);
		if (app->time_range != ygl::zero2f) {
			ygl::draw_glwidgets_dragbox(
				win, "time", app->time, app->time_range.x, app->time_range.y);
			ygl::draw_glwidgets_text(win, "anim group", app->anim_group);
			ygl::draw_glwidgets_checkbox(win, "animate", app->animate);
		}
		if (ygl::begin_glwidgets_tree(win, "render settings")) {
			ygl::draw_glwidgets_combobox(
				win, "camera", app->cam, app->scn->cameras);
			draw_glwidgets_dragbox(
				win, "resolution", app->resolution, 256, 4096);
			draw_glwidgets_checkbox(win, "eyelight", app->eyelight);
			draw_glwidgets_checkbox(win, "wireframe", app->wireframe);
			ygl::continue_glwidgets_line(win);
			draw_glwidgets_checkbox(win, "edges", app->edges);
			ygl::continue_glwidgets_line(win);
			draw_glwidgets_checkbox(win, "cutout", app->cutout);
			ygl::continue_glwidgets_line(win);
			draw_glwidgets_checkbox(win, "cull", app->cull_backface);
			ygl::end_glwidgets_tree(win);
		}
		if (ygl::begin_glwidgets_tree(win, "view settings")) {
			draw_glwidgets_dragbox(win, "exposure", app->exposure, -10, 10);
			draw_glwidgets_dragbox(win, "gamma", app->gamma, 0.1f, 4);
			draw_glwidgets_colorbox(win, "background", app->background);
			ygl::draw_glwidgets_checkbox(win, "fps", app->navigation_fps);
			ygl::end_glwidgets_tree(win);
		}
		if (ygl::begin_glwidgets_tree(win, "scene tree")) {
			ygl::draw_glwidgets_scene_tree(win, "", app->scn, app->selection,
				app->update_list, app->inspector_highlights);
			ygl::end_glwidgets_tree(win);
		}
		if (ygl::begin_glwidgets_tree(win, "scene object")) {
			ygl::draw_glwidgets_scene_inspector(win, "", app->scn,
				app->selection, app->update_list, app->inspector_highlights);
			ygl::end_glwidgets_tree(win);
		}
		if (ygl::begin_glwidgets_tree(win, "face highlighting")) {
			// init as not highlighted
			if (is_face_highlighted.size() == 0) {
				for (auto i : building_shp.face_ids()) {
					is_face_highlighted[i] = false;
				}
			}

			for (auto& kv : is_face_highlighted) {
				if (ygl::draw_glwidgets_checkbox(win, "face " + std::to_string(kv.first), kv.second)) {
					auto color = kv.second ?
						ygl::vec4f{ 1.f, 0.f, 0.f, 1.f } : ygl::vec4f{ 1.f, 1.f, 1.f, 1.f };
					for (int i = 0; i < building_shp.pos.size(); i++) {
						auto fid = building_shp.vertex_tags[i].face_id;
						building_shp.color[i] =
							building_shp.vertex_tags[i].face_id == kv.first ? 
							color : ygl::vec4f{1.f, 1.f, 1.f, 1.f};
					}
					for (auto& kv2 : is_face_highlighted) {
						if (kv.first != kv2.first) {
							kv2.second = false;
						}
					}
					ygl::update_gldata(app->scn);
				}
			}
			ygl::end_glwidgets_tree(win);
		}
		if (ygl::begin_glwidgets_tree(win, "windows")) {
			static bool windows_computed = false;
			static yb::bool_operation win_op = yb::bool_operation::DIFFERENCE;

			ygl::draw_glwidgets_combobox(
				win, "win mode", win_op,
				{
					{yb::bool_operation::DIFFERENCE, "difference"},
					{yb::bool_operation::UNION, "union"}
				}
			);
			if (ygl::draw_glwidgets_button(win, "compute windows") && !windows_computed) {
				windows_computed = true;
				ygl::log_info("carving windows...");

				yb::tagged_shape all_windows;

				const float cube_size = 0.5f;

				optimization_problem prob_h;
				prob_h.add_parameter(parameter("nw", 1.f, 20.f));
				prob_h.add_parameter(parameter("sb", 2.f, 10.f));
				prob_h.add_objective([&](const auto& params) {
					auto nw = params[0].get_rval();
					auto sb = params[1].get_rval();
					auto excess_penalty = BUILDING_HEIGHT*0.9f - (nw*cube_size + (nw - 1)*sb);
					excess_penalty = excess_penalty < 0 ? excess_penalty * 100000.f : 0.f;
					return nw * 100 + sb - fmodf(nw, 1.0)*100.0 + excess_penalty;
				});
				particle_swarm(prob_h, 0.8f, 2.f, 2.f, 100, 100);

				for (int s = 0; s < bs.num_sides(); s++) {
					optimization_problem prob_w;
					prob_w.add_parameter(parameter("nw", 1.f, 20.f));
					prob_w.add_parameter(parameter("sb", 1.f, 10.f));
					prob_w.add_objective([&](const auto& params) {
						auto nw = params[0].get_rval();
						auto sb = params[1].get_rval();
						auto excess_penalty = std::max((nw*cube_size + (nw - 1)*sb) - bs.sides[s].length()*0.9f, 0.f)*-100'000;
						return nw * 100 + sb - fmodf(nw, 1.0)*1000.0 + excess_penalty;
					});
					particle_swarm(prob_w, 0.8f, 2.f, 2.f, 300, 300);

					auto num_windows = int(prob_w.get_params()[0].get_rval());
					auto num_floors = int(prob_h.get_params()[0].get_rval());
					auto space_between_windows = prob_w.get_params()[1].get_rval();
					auto space_between_floors = prob_h.get_params()[1].get_rval();
					auto space_from_edges =
						(bs.sides[s].length() - num_windows*cube_size - (num_windows - 1)*space_between_windows)/2.f;
					auto space_from_floor_ceiling =
						(BUILDING_HEIGHT - num_floors*cube_size - (num_floors - 1)*space_between_floors) / 2.f;

					if (DEBUG_PRINT) {
						std::cout
							<< "----------------------------------------\n"
							<< "X:\n"
							<< "  Num windows: " << num_windows << std::endl
							<< "  Space between windows: " << space_between_windows << std::endl
							<< "  Space from edges: " << space_from_edges << std::endl
							<< "  Facade width: " << bs.sides[s].length() << std::endl
							<< "Y:\n"
							<< "  Num floors: " << num_floors << std::endl
							<< "  Space between floors: " << space_between_floors << std::endl
							<< "  Space from top and bottom: " << space_from_floor_ceiling << std::endl
							<< "  Building height: " << BUILDING_HEIGHT << std::endl
							<< "----------------------------------------\n";
					}

					for (int i = 0; i < num_windows; i++) {
						for (int j = 0; j < num_floors; j++) {
							if (s == 0) continue;
							auto x = (space_from_edges + cube_size / 2.f + (cube_size + space_between_windows)*i) / bs.sides[s].length();
							auto y = (space_from_floor_ceiling + cube_size / 2.f + (cube_size + space_between_floors)*j) / BUILDING_HEIGHT;
							auto clp = compute_layout_position(building_shp, s, { x,y });
							if (!std::get<0>(clp)) {
								continue;
							} 
							auto p = std::get<1>(clp);
							ygl::shape c = yb::make_cube(c.quads, c.pos, cube_size);

							// todo: generalize rotation (needs up vector)
							auto n = std::get<2>(clp);
							yb::rotate_y(c.pos, -yb::get_angle({n.x,n.z}));
							for (auto& pos : c.pos) pos += p;

							// pos, triangles, tags
							std::tie(all_windows.pos, all_windows.triangles) = yb::mesh_boolean_operation(
								all_windows.pos, all_windows.triangles,
								c.pos, c.triangles,
								yb::bool_operation::UNION
							);
						}
					}
				}

				//s = 0, two windows with balcony and a door
				{
					// random params
					random_generator rng;
					float door_width_ratio = rng.rand(0.2f, 0.4f);
					float door_width = bs.sides[0].length()*door_width_ratio;
					float door_height_ratio = rng.rand(0.4f, 0.5f);
					float door_height = BUILDING_HEIGHT*door_height_ratio;
					float window_width = rng.rand(1.5f, 2.f);
					float window_height = rng.rand(1.5f, 2.f);
					float w_from_b = rng.rand(0.5f, 1.25f);

					optimization_problem s0_op;
					s0_op.add_parameter(parameter("doorx", door_width_ratio/2.f, 1.f-door_width_ratio/2.f));
					s0_op.add_parameter(parameter("balcony_height", 0.2f, 1.f));
					s0_op.add_parameter(parameter("balcony_width", window_width*1.5f, 2.5f*window_width));
					s0_op.add_parameter(parameter("win_dist_from_center", 0.f, 0.3f));
					s0_op.add_parameter(parameter("win y", door_height_ratio+0.15f, 0.9f));
					s0_op.add_objective([&](const auto& params) {
						auto doorx = params[0].get_rval();
						auto bh = params[1].get_rval();
						auto bw = params[2].get_rval();
						auto wd = params[3].get_rval();
						auto wy = params[4].get_rval();

						auto obj = 0;
						auto excess = (bw+0.1) / 2 - (wd*bs.sides[0].length());
						if (excess > 0) {
							obj -= excess*100.f; // Penalty on overlapping balconies
						}
						return obj;
					});
					particle_swarm(s0_op, 0.8, 1.4, 1.4, 200, 200);

					auto pars = s0_op.get_params();
					auto doorx = pars[0].get_rval();
					auto bh = pars[1].get_rval();
					auto bw = pars[2].get_rval();
					auto wd = pars[3].get_rval();
					auto wy = pars[4].get_rval();

					yb::tagged_shape door = yb::make_cube(door.quads, door.pos, 1.f, 7);
					for (auto& pos : door.pos) {
						pos.x *= door_width;
						pos.y *= door_height;
					}
					auto p = compute_position(0, doorx, 0);
					auto b_der = bezier_derivative(bs.sides[0]);
					auto rot_xz = ygl::normalize(b_der.compute(doorx));
					auto rot_angle = yb::get_angle(rot_xz) /*- yb::pi/2.f*/;
					yb::rotate_y(door.pos, rot_angle);
					for (auto& pos : door.pos) { 
						pos += p; 
						pos.y += door_height / 2.f; 
					};
					std::tie(building_shp.pos, building_shp.triangles, building_shp.vertex_tags) =
						yb::mesh_boolean_operation(building_shp, door, yb::bool_operation::UNION);

					for (int wi = 0; wi < 2; wi++) {
						yb::tagged_shape w = yb::make_cube(w.quads, w.pos, 1.f, 7);
						for (auto& pos : w.pos) {
							pos.x *= window_width;
							pos.y *= window_height;
						}
						auto sign = wi ? +1 : -1;
						auto p = compute_position(0, 0.5f + wd*sign, 0);
						auto b_der = bezier_derivative(bs.sides[0]);
						auto rot_xz = ygl::normalize(b_der.compute(0.5f + wd*sign));
						auto rot_angle = yb::get_angle(rot_xz) /*- yb::pi/2.f*/;
						yb::rotate_y(w.pos, rot_angle);
						for (auto& pos : w.pos) {
							pos += p;
							pos.y += wy*BUILDING_HEIGHT;
						};
						std::tie(building_shp.pos, building_shp.triangles, building_shp.vertex_tags) =
							yb::mesh_boolean_operation(building_shp, w, yb::bool_operation::UNION);

						// Balconies
						w = yb::make_cube(w.quads, w.pos, 1.f, 7);
						for (auto& pos : w.pos) {
							pos.x *= bw;
							pos.y *= bh;
						}
						sign = wi ? +1 : -1;
						p = compute_position(0, 0.5f + wd*sign, 0);
						b_der = bezier_derivative(bs.sides[0]);
						rot_xz = ygl::normalize(b_der.compute(0.5f + wd*sign));
						rot_angle = yb::get_angle(rot_xz) /*- yb::pi/2.f*/;
						yb::rotate_y(w.pos, rot_angle);
						for (auto& pos : w.pos) {
							pos += p;
							pos.y += wy*BUILDING_HEIGHT - window_height/2.f - w_from_b;
						};
						std::tie(building_shp.pos, building_shp.triangles, building_shp.vertex_tags) =
							yb::mesh_boolean_operation(building_shp, w, yb::bool_operation::UNION);
					}
				}

				all_windows.vertex_tags = std::vector<yb::tagged_shape::tag>(all_windows.pos.size(), { 7,{0,0} });
				std::tie(building_shp.pos, building_shp.triangles, building_shp.vertex_tags) =
					yb::mesh_boolean_operation(building_shp, all_windows, win_op);
				ygl::log_info("    converting to faceted");
				yb::convert_tagged_to_faceted(&building_shp);
				building_shp.color.resize(building_shp.pos.size(), { 1,1,1,1 });
				ygl::update_gldata(app->scn);
				ygl::log_info("    done");
			}
			ygl::end_glwidgets_tree(win);
		}
		if (ygl::begin_glwidgets_tree(win, "other")) {
			if (ygl::draw_glwidgets_button(win, "convert to faceted")) {
				ygl::log_info("converting to faceted...");
				yb::convert_to_faceted(&building_shp);
				building_shp.color.resize(building_shp.pos.size(), { 1,1,1,1 });
				ygl::log_info("    done");
			}
			if (ygl::draw_glwidgets_button(win, "merge overlapping vertices")) {
				ygl::log_info("merging overlapping points...");
				yb::merge_same_points(&building_shp);
				ygl::update_gldata(app->scn);
				ygl::log_info("    done");
			}
			if (ygl::draw_glwidgets_checkbox(win, "highlight borders", show_border_highlighting)) {
				auto newval = show_border_highlighting;
				reset_colors();
				show_border_highlighting = newval;
				if (show_border_highlighting) {
					for (auto i = 0; i < building_shp.pos.size(); i++) {
						if (building_shp.is_edge_vertex(i)) {
							building_shp.color[i] = { 1,0,0,1 };
						}
					}
				}
				ygl::update_gldata(app->scn);
			}
			if (ygl::draw_glwidgets_checkbox(win, "show face coord heatmap", show_facecoord_heatmap)) {
				auto newval = show_facecoord_heatmap;
				reset_colors();
				show_facecoord_heatmap = newval;
				if (show_facecoord_heatmap) {
					for (int i = 0; i < building_shp.pos.size(); i++) {
						auto intensity =
							1.f - std::max(
								fabs(0.5f - building_shp.vertex_tags[i].face_coord.x),
								fabs(0.5f - building_shp.vertex_tags[i].face_coord.y)
							)*2.f;
						building_shp.color[i] = { intensity, intensity, intensity, 1.f };
					}
				}
				ygl::update_gldata(app->scn);
			}
			if (ygl::draw_glwidgets_checkbox(win, "pattern 2", show_pattern_2)) {
				auto newval = show_pattern_2;
				reset_colors();
				show_pattern_2 = newval;
				if (show_pattern_2) {
					for (int i = 0; i < building_shp.pos.size(); i++) {
						const auto& tag = building_shp.vertex_tags[i];
						auto intensity = std::max(tag.face_coord.x,tag.face_coord.y);
						building_shp.color[i] = { intensity/2.f,intensity/2.f,intensity,1.f };
					}
				}
				ygl::update_gldata(app->scn);
			}
			if (ygl::draw_glwidgets_button(win, "inflate faces by face coord heatmap")) {
				inflate_by_facecoord_heatmap(1.f);
				ygl::update_gldata(app->scn);
			}
			if (ygl::draw_glwidgets_button(win, "deflate faces by face coord heatmap")) {
				inflate_by_facecoord_heatmap(-1.f);
				ygl::update_gldata(app->scn);
			}
			if (ygl::draw_glwidgets_button(win, "inflate along normals")) {
				for (int i = 0; i < building_shp.pos.size(); i++) {
					building_shp.pos[i] += building_shp.norm[i];
				}
				ygl::update_gldata(app->scn);
			}
			if (ygl::draw_glwidgets_button(win, "deflate along normals")) {
				for (int i = 0; i < building_shp.pos.size(); i++) {
					building_shp.pos[i] -= building_shp.norm[i];
				}
				ygl::update_gldata(app->scn);
			}
		}
	}
	ygl::end_glwidgets_frame(win);

	ygl::swap_glwindow_buffers(win);
}

inline void refresh(ygl::glwindow* win) {
	return draw(win, (app_state*)ygl::get_glwindow_user_pointer(win));
}

// run ui loop
void run_ui(app_state* app) {
	// window
	auto win =
		ygl::make_glwindow((int)std::round(app->cam->aspect * app->resolution) +
			ygl::default_glwidgets_width,
			app->resolution, "yiview", app);
	ygl::set_glwindow_callbacks(win, nullptr, nullptr, refresh);

	// load textures and vbos
	app->prog = ygl::make_glsurface_program();
	ygl::update_gldata(app->scn);
	ygl::update_transforms(app->scn, app->time);
	ygl::update_lights(app->scn);
	if (app->scn->lights.empty()) app->eyelight = true;

	// init widget
	ygl::init_glwidgets(win);

	// loop
	while (!should_glwindow_close(win)) {
		// handle mouse and keyboard for navigation
		ygl::handle_glcamera_navigation(win, app->cam, app->navigation_fps);

		// animation
		if (app->animate) {
			app->time += 1 / 60.0f;
			if (app->time < app->time_range.x || app->time > app->time_range.y)
				app->time = app->time_range.x;
			ygl::update_transforms(app->scn, app->time);
		}

		// draw
		draw(win, app);

		// check if exiting is needed
		if (app->screenshot_and_exit) {
			ygl::log_info("taking screenshot and exiting...");
			auto width = 0, height = 0;
			auto img = std::vector<ygl::vec4b>();
			ygl::take_glwindow_screenshot4b(win, width, height, img);
			ygl::save_image4b(app->imfilename, width, height, img);
			break;
		}

		// event hadling
		if (ygl::get_glwindow_mouse_button(win) ||
			ygl::get_glwidgets_active(win) || app->animate) {
			ygl::poll_glwindow_events(win);
		}
		else {
			ygl::wait_glwindow_events(win);
		}
	}

	// cleanup
	delete win;
}

int main(int argc, char* argv[]) {

	// create empty scene
	auto app = new app_state();
	app->navigation_fps = true;

	// parse command line
	auto parser =
		ygl::make_parser(argc, argv, "yiview", "views scenes interactively");

	SEGMENTS_PER_BEZIER = ygl::parse_opt(
		parser, "--bezier-segments", "-bs", "Number of segments for each bezier curve.", 40
	);
	BUILDING_HEIGHT = ygl::parse_opt(
		parser, "--building-height", "-bh", "Height of the straight building.", 20.f
	);
	DO_VERTICAL_BEZIER = ygl::parse_flag(
		parser, "--vertical-bezier", "-vb", "The building is extruded along a bezier curve."
	);
	INCLINED_FLOORS = ygl::parse_flag(
		parser, "--inclined floors", "-if", "The floors are perpendicular to the extrusion path."
	);
	DO_VERTICAL_ROTATION = ygl::parse_flag(
		parser, "--vertical-rotation", "-vr", "The building rotates around itself during extrusion."
	);
	DO_MERGE_SAME_POINTS = ygl::parse_flag(
		parser, "--no-merge-points", "-nm", "Do not merge overlapping vertices after building generation.", true
	);

	app->eyelight = ygl::parse_flag(
		parser, "--eyelight", "-c", "Eyelight rendering.", false);
	auto double_sided = ygl::parse_flag(
		parser, "--double-sided", "-D", "Force double sided rendering.", false);
	app->quiet =
		ygl::parse_flag(parser, "--quiet", "-q", "Print only errors messages");
	app->screenshot_and_exit = ygl::parse_flag(
		parser, "--screenshot-and-exit", "", "Take a screenshot and exit");
	app->no_glwidgets =
		ygl::parse_flag(parser, "--no-widgets", "", "Disable widgets");
	auto highlight_filename =
		ygl::parse_opt(parser, "--highlights", "", "Highlight filename", ""s);
	app->imfilename = ygl::parse_opt(
		parser, "--output-image", "-o", "Image filename", "out.png"s);

	if (ygl::should_exit(parser)) {
		printf("%s\n", get_usage(parser).c_str());
		exit(1);
	}

	// Building creation 
	ygl::log_info("creating building mesh");
	building_shp.name = "building_shp";

	auto facade_face_ids = building_shp.get_new_ids(bs.num_sides());
	for (int y = 0; y < SEGMENTS_PER_BEZIER; y++) {
		auto y_t = float(y) / SEGMENTS_PER_BEZIER_F;
		auto next_y_t = float(y + 1) / SEGMENTS_PER_BEZIER_F;

		for (int i = 0; i < SEGMENTS_PER_BEZIER; i++) {
			auto x_t = float(i) / SEGMENTS_PER_BEZIER_F;
			auto next_x_t = float(i + 1) / SEGMENTS_PER_BEZIER_F;

			for (int side = 0; side < bs.num_sides(); side++) {
				auto face_id = facade_face_ids[side];
				building_shp.vertex_tags.push_back({ face_id,{ x_t, y_t } });
				building_shp.vertex_tags.push_back({ face_id,{ next_x_t, y_t } });
				building_shp.vertex_tags.push_back({ face_id,{ next_x_t, next_y_t } });
				building_shp.vertex_tags.push_back({ face_id,{ x_t, next_y_t } });

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
	ygl::log_info("floor and roof");
	std::vector<ygl::vec3f> floor_border;
	for (int side = 0; side < bs.num_sides(); side++) {
		for (int i = 0; i < SEGMENTS_PER_BEZIER; i++) {
			floor_border.push_back(compute_position(side, float(i) / SEGMENTS_PER_BEZIER_F, 0.f));
		}
	}
	std::vector<ygl::vec3f> floor_pos;
	std::vector<ygl::vec3i> floor_triangles;
	std::tie(floor_triangles, floor_pos) = yb::triangulate_opposite(floor_border);
	int bps = building_shp.pos.size();
	building_shp.pos.insert(building_shp.pos.end(), floor_pos.begin(), floor_pos.end());
	for (int i = 0, id = building_shp.get_new_id(); i < floor_pos.size(); i++)
		building_shp.vertex_tags.push_back({ id,{ 0.f, 0.f } });
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
	for (int i = 0, id = building_shp.get_new_id(); i < floor_pos.size(); i++)
		building_shp.vertex_tags.push_back({ id,{ 0.f, 0.f } });
	for (const auto& t : floor_triangles)
		building_shp.triangles.push_back({ t.x + bps, t.y + bps, t.z + bps });

	// Final settings
	ygl::material* building_mat = ygl::make_matte_material("building_mat", { 0.95f,0.95f,0.85f });
	ygl::instance* building_inst = ygl::make_instance("building_inst", &building_shp, building_mat);

	if (DO_MERGE_SAME_POINTS) {
		ygl::log_info("merging overlapping points");
		building_shp.merge_same_points();
	}
	building_shp.color.resize(building_shp.pos.size(), { 1,1,1,1 });

	ygl::log_info("computing normals"); 
	ygl::compute_normals(building_shp.triangles, building_shp.pos, building_shp.norm);

	ygl::scene scene;
	ygl::log_info("generating scene");
	yb::add_to_scene(&scene, building_inst);

	// setup logger
	if (app->quiet) ygl::log_verbose() = false;

	// scene loading
	app->scn = &scene;

	// tesselate input shapes
	ygl::tesselate_shapes(app->scn, true, false, false, false);

	// fix scene
	ygl::update_bbox(app->scn);
	ygl::add_missing_camera(app->scn);
	ygl::camera top_cam = *app->scn->cameras.front();
	top_cam.name = "top camera";
	top_cam.frame = ygl::lookat_frame({ 0,BUILDING_HEIGHT*2.f,0 }, { 0,0,0 }, { 0,0,-1 });
	app->scn->cameras.push_back(&top_cam);
	ygl::camera right_cam = *app->scn->cameras.front();
	right_cam.name = "right camera";
	right_cam.frame = ygl::lookat_frame({ 20,0,0 }, { 0,0,0 }, { 0,0,0 });
	app->scn->cameras.push_back(&right_cam);
	ygl::camera left_cam = *app->scn->cameras.front();
	left_cam.name = "left camera";
	left_cam.frame = ygl::lookat_frame({ -20,0,0 }, { 0,0,0 }, { 0,1,0 });
	app->scn->cameras.push_back(&left_cam);
	ygl::camera back_cam = *app->scn->cameras.front();
	back_cam.name = "back camera";
	back_cam.frame = ygl::lookat_frame({ 0,0,-20 }, { 0,0,0 }, { 0,1,0 });
	app->scn->cameras.push_back(&back_cam);
	ygl::camera tr_cam = *app->scn->cameras.front();
	tr_cam.name = "topright camera";
	tr_cam.frame = ygl::lookat_frame({ 20,BUILDING_HEIGHT*2.f,20 }, { 0,20,0 }, { 0,1,0 });
	app->scn->cameras.push_back(&tr_cam);
	ygl::add_missing_names(app->scn);
	ygl::add_missing_tangent_space(app->scn);
	app->cam = app->scn->cameras[0];
	if (double_sided) {
		for (auto mat : app->scn->materials) mat->double_sided = true;
	}

	// validate
	for (auto err : ygl::validate(app->scn)) ygl::log_warning(err);

	// animation
	app->time_range = ygl::compute_animation_range(app->scn);
	app->time = app->time_range.x;

	// lights
	ygl::update_bbox(app->scn);
	ygl::update_lights(app->scn);

	// run ui
	run_ui(app);

	// cleanup
	delete app;

	// done
	return 0;
}
