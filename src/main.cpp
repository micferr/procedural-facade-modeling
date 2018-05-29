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

/// App constants

// Number of segments to split each bezier in when rendering
const int SEGMENTS_PER_BEZIER = 40;
// Utility
const float SEGMENTS_PER_BEZIER_F = float(SEGMENTS_PER_BEZIER);

// Building's height when DO_VERTICAL_BEZIER == false
const float BUILDING_HEIGHT = 20.f;

// Whether the floors' extrusion follows a bezier path
const bool DO_VERTICAL_BEZIER = false;

// Assuming DO_VERTICAL_BEZIER == true, whether floors are perpendicular
// to the bezier's derivative
const bool INCLINED_FLOORS = false;

// Whether the building rotates around itself
const bool DO_VERTICAL_ROTATION = true;

// Whether to merge overlapping vertices
const bool DO_MERGE_SAME_POINTS = true;

// Number of windows per side and number of floors
const int NUM_WINDOWS_PER_SIDE = 5;
const int NUM_FLOORS = 6;

/// Struct, classes and functions

struct tagged_shape : public ygl::shape {
	struct tag {
		int face_id; // Which face the vertex belongs to
		ygl::vec2f face_coord; // Usually a [0;1]x[0;1] coordinate relative to the face
	};

	std::vector<tag> vertex_tags;

	std::set<int> face_ids() {
		std::set<int> ids;
		for (const auto& t : vertex_tags)
			ids.insert(t.face_id);
		return ids;
	}

	int num_faces() {
		return face_ids().size();
	}

	// Returns an unused id for a new face
	int get_new_id() {
		return get_new_ids(1)[0];
	}

	// Returns n unused ids for new faces
	std::vector<int> get_new_ids(int n) {
		auto ids = face_ids();
		std::vector<int> new_ids;
		auto count = 0;
		for (auto i = 0; count < n; i++) {
			if (ids.count(i) == 0) {
				new_ids.push_back(i);
				count++;
			}
		}
		return new_ids;
	}

	bool is_edge_vertex(int i, float max_zero = 0.0001f, float min_one = 0.9999f) {
		auto tag = vertex_tags[i];
		return tag.face_coord.x <= max_zero || tag.face_coord.x >= min_one ||
			tag.face_coord.y <= max_zero || tag.face_coord.y >= min_one;
	}

	void merge_same_points(float eps = 0.0001f, float max_zero = 0.0001f, float min_one = 0.9999f) {
		for (int i = 0; i < pos.size(); i++) {
			for (int j = i + 1; j < pos.size(); j++) {
				// If a vertex is part of a face border, don't merge it or onto it
				if (is_edge_vertex(i) || is_edge_vertex(j)) continue;

				// Check within epsilon
				if (
					ygl::length(pos[i] - pos[j]) < eps &&
					(color.size() == 0 || ygl::length(color[i] - color[j]) < eps)
					) {
					// Assuming a triangle mesh
					if (triangles.size() > 0) {
						for (auto& t : triangles) {
							if (t.x == j) t.x = i;
							if (t.y == j) t.y = i;
							if (t.z == j) t.z = i;
						}
					}
				}
			}
		}
		if (triangles.size() > 0) {
			ygl::compute_normals(triangles, pos, norm);
		}
		else if (quads.size() > 0) {
			ygl::compute_normals(quads, pos, norm);
		}
	}
};


template<class point>
struct bezier {
	std::vector<point> control_points;

	bezier() = default;
	bezier(const std::vector<point>& points) : control_points(points) {}

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

tagged_shape building_shp;

bezier<ygl::vec3f> vertical_path({
	{ 0,0,0 },
	{ 20,10,0 },
	{ 20,20,20 },
	{ 10,35,-10 }
});
auto vert_derivative = bezier_derivative(vertical_path);

bezier_sides<ygl::vec2f> bs(
{
	{ -4,-4 },{ -2,0 },{ 0,-8 },{ 2,-4 },
	{ 1,-1 },{ 2,2 },
	{ -1,6 },{ -4,2 }
},
{
	0,3,5,7
}
);

// Rotation of x and z coordinates, in radiants, around y-axis
auto rotation_path = [](float t) {return 3.f*t;};

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
	ygl::vec4b background = { 222, 222, 222, 0 };  // background
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

	if (ygl::begin_glwidgets_frame(win, "yview")) {
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

				for (int j = 0; j < NUM_FLOORS; j++) {
					for (int i = 0; i < NUM_WINDOWS_PER_SIDE; i++) {
						for (int s = 0; s < bs.num_sides(); s++) {
							auto dx = 1 / float(NUM_WINDOWS_PER_SIDE);
							auto dy = 1 / float(NUM_FLOORS);
							auto p = compute_position(s, i*dx + dx/2.f, j*dy + dy/2.f);
							ygl::shape c;
							ygl::make_cube(c.quads, c.pos, 0, 0.5f);
							yb::to_triangle_mesh(&c);

							auto b_der = bezier_derivative(bs.sides[s]);
							auto rot_xz = ygl::normalize(b_der.compute(i*dx + dx/2.f));
							auto rot_angle = yb::get_angle(rot_xz)-yb::pi/2.f;
							if (DO_VERTICAL_ROTATION) {
								rot_angle += rotation_path(j*dy + dy/2.f);
							}
							yb::rotate_y(c.pos, rot_angle);
							for (auto& pos : c.pos) pos += p;

							auto pt = yb::mesh_boolean_operation(
								building_shp.pos, building_shp.triangles,
								c.pos, c.triangles, win_op
							);
							std::tie(building_shp.pos, building_shp.triangles) = pt;
							ygl::log_info(
								"    floor windows carved: {}/{}",
								int(i*bs.num_sides() + s + 1),
								int(NUM_WINDOWS_PER_SIDE*bs.num_sides())
							);
						}
					}
					ygl::log_info("    Floors carved: {}/{}", j + 1, NUM_FLOORS);
				}
				ygl::log_info("    converting to faceted");
				yb::convert_to_faceted(&building_shp);
				building_shp.color.resize(building_shp.pos.size(), { 1,1,1,1 });
				ygl::update_gldata(app->scn);
				ygl::log_info("    done");
			}
			ygl::end_glwidgets_tree(win);
		}
		if (ygl::begin_glwidgets_tree(win, "other")) {
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
			app->resolution, "yview | " + app->filename, app);
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
	ygl::material* building_mat = ygl::make_matte_material("building_mat", { 1.f,0.3f,0.3f });
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
