#ifndef BUILDING_UTILS_H
#define BUILDING_UTILS_H

#include "geom_utils.h"
#include "prob_utils.h"
#include "yocto_utils.h"

/**
 * A collection of utilities to deal with procedural building generation.
 *
 * It contains functions to generate most common architectural elements (e.g roofs,
 * belt courses, railings, ...)
 *
 * Many routines use the notion of a building's floor's main point: for buildings
 * with floor shape generated from widening a segmented line, the line's vertexes
 * are that building's floor's main points.
 * See make_wide_line and make_wide_line_border functions in geom_utils.h for further
 * information.
 *
 * A building's floor can also be described by a (center, arbitrary vertex, number of sides)
 * triple (these are referred to as regular buildings), or directly from all its vertexes. 
 * These mode may however offer less customizability.
 */

namespace yb {

	/// Parameter enums and structs

	enum class roof_type {
		none = 0,
		crossgabled,
		crosshipped,
		pyramid
	};

	struct building_params;

	// Roof parameters, all types of roof combined for simplicity.
	// Only relevant params have to be set, the others can be ignored
	struct roof_params {
		roof_type type = roof_type::none;
		ygl::vec3f color1 = { 1,1,1 };
		float roof_angle = yb::pi / 2.f;

		// CrossGabled
		float thickness = -1.f; // Ignored if negative
		ygl::vec3f color2 = { 1,1,1 };
		float rake_overhang = 0.f;
		float roof_overhang = 0.f;

		// CrossHipped
		float hip_depth = 0.f;

		// Pyramid
		float roof_height = 5.f; // We can't use angle since it's different for each edge

		// None
		float recursive_prob = 0.7f; // Probability to build new buildings on top of the current one
		                            // (The probablity is among building with roof_pars.type == none)
		building_params* recursive_params = nullptr; // If nullptr, see make_rand_building_params
		float keep_prob = 0.9f; // Keep probability for main points 
		                        // (see random_substrings in prob_utils.h)
		float continue_prob = 0.85f; // Continue probability for main points
									// (see random_substrings in prob_utils.h)
	};

	// Groups the parameters relative to windows's generation
	struct windows_params {
		std::string name = "";
		float windows_distance = 0.f; // Avg distance between windows
		float windows_distance_from_edges = 0.f;
		ygl::shape* closed_window_shape = nullptr;
		ygl::shape* open_window_shape = nullptr;
		float open_windows_ratio = 0.5f; // Percentage of open windows
		float filled_spots_ratio = 1.f; // Ratio of spots that actually have a window
	};

	enum class building_type {
		main_points = 0,
		border,
		regular
	};

	struct building_params {
		building_type type;

		// Type == Main points
		std::vector<ygl::vec2f> floor_main_points = {};
		float floor_width = 1.f;
		// Type == Border
		std::vector<ygl::vec2f> floor_border = {};
		// Type == Regular
		ygl::vec2f floor_center = { 0,0 };
		unsigned num_sides = 3;
		float radius = 1.f;
		float reg_base_angle = 0.f;

		unsigned num_floors = 1;
		float floor_height = 1.f;
		float belt_height = 0.1f;
		float belt_additional_width = 0.1f;
		float width_delta_per_floor = 0.f; // How much to expand or shrink 
		                                   // each consecutive floor,
		                                   // as a polygon offsetting size
		std::string id = "";
		ygl::vec3f color1 = { 1,1,1 };
		ygl::vec3f color2 = { 1,1,1 };
		ygl::rng_pcg32* rng = nullptr;

		// Roof
		roof_params roof_pars;

		// Windows
		windows_params win_pars;

		// Tower
		float tower_prob; // Probability to build a tower
		bool tower_on_shrinking_building; // If false, no tower is built near an expanding 
		                                  // building (otherwise they would collide)
		std::function<int(const std::vector<ygl::vec2f>&)>
			tower_placement = [](const std::vector<ygl::vec2f>& border)->int {return 0;};
		int tower_num_floors = 2;
		int tower_num_sides = 4;
		float tower_radius = 1.f;

		// Wall
		// -- nothing for now --
	};

	// Roofs

	ygl::shape* make_roof_crossgabled_simple(
		const std::vector<ygl::vec2f>& floor_main_points,
		float floor_width,
		float roof_angle,
		float base_height = 0.f
	) {
		if (roof_angle <= 0.f || roof_angle >= yb::pi / 2.f) {
			throw std::runtime_error("Invalid roof angle");
		}
		float center_height = tanf(roof_angle)*floor_width / 2.f;
		auto _floor_main_points = to_3d(floor_main_points);
		ygl::shape* shp = new ygl::shape();
		shp->pos = yb::make_wide_line_border(floor_main_points, floor_width);
		auto mid_point = (shp->pos[0] + shp->pos.back()) / 2.f;
		mid_point.y += center_height;
		shp->pos.push_back(mid_point);
		// i goes along the right side of the widenened segmented line, j is the opposite point
		for (int i = 0, j = shp->pos.size() - 2; i < j-2; i+=1, j-=1) {
			auto mid_point_next = (shp->pos[i+1] + shp->pos[j-1]) / 2.f;
			mid_point_next.y += center_height;
			shp->pos.push_back(mid_point_next);
			shp->quads.push_back({ j,j - 1,i + 1,i });
			shp->quads.push_back({ j - 1,j,int(shp->pos.size() - 2),int(shp->pos.size() - 1) });
			shp->quads.push_back({ i,i + 1,int(shp->pos.size() - 1),int(shp->pos.size() - 2) });
			
			mid_point = mid_point_next;
		}
		int floor_points = 2 * floor_main_points.size();
		shp->triangles.push_back({ 
			0,floor_points,floor_points-1
		});
		shp->triangles.push_back({ 
			floor_points/2, int(shp->pos.size()-1), floor_points/2-1
		});
		if (base_height != 0.f) { // Check purely for performance
			displace(shp->pos, { 0,base_height,0 });
		}
		shp->norm = ygl::compute_normals({}, shp->triangles, shp->quads, shp->pos);
		return shp; 
	}

	/**
	 * Hip depth must be strictly less than both the first and last 
	 * floor's main segments' length + floor_width/2 (floor_width/2 because of
	 * lengthened ends)
	 */
	ygl::shape* make_roof_crosshipped_simple(
		const std::vector<ygl::vec2f>& floor_main_points,
		float floor_width,
		float roof_angle,
		float hip_depth,
		float base_height = 0.f
	) {
		auto shp = make_roof_crossgabled_simple(
			floor_main_points, floor_width, roof_angle, base_height
		);
		auto fr = 2*floor_main_points.size(); // First roof point
		auto lr = shp->pos.size() - 1; // Last roof point
		shp->pos[fr] += ygl::normalize(shp->pos[fr + 1] - shp->pos[fr])*hip_depth;
		shp->pos[lr] -= ygl::normalize(shp->pos[lr] - shp->pos[lr - 1])*hip_depth;
		shp->norm = ygl::compute_normals({}, shp->triangles, shp->quads, shp->pos);
		return shp;
	}

	ygl::shape* make_roof_pyramid_from_border(
		const std::vector<ygl::vec2f>& border,
		float roof_height,
		float base_height = 0.f
	) {
		auto shp = new ygl::shape();
		auto t = triangulate_opposite(to_3d(border), {});
		shp->triangles = std::get<0>(t);
		shp->pos = std::get<1>(t);
		auto c = centroid(border);
		auto top = to_3d(c, roof_height);
		shp->pos.push_back(top);
		int sz = shp->pos.size();
		shp->pos.push_back(to_3d(border[0]));
		for (int i = 0; i < border.size()-1; i++) {
			shp->pos.push_back(to_3d(border[i + 1]));
			shp->triangles.push_back({ sz + i, sz + i + 1, sz - 1 });
		}
		shp->triangles.push_back({ sz + int(border.size()) - 1, sz, sz - 1 });
		if (base_height > 0.f) displace(shp->pos, { 0,base_height,0 });
		merge_same_points(shp);
		set_shape_normals(shp);
		return shp;
	}

	ygl::shape* make_roof_pyramid_from_regular(
		const ygl::vec2f& floor_center,
		const ygl::vec2f& floor_vertex,
		unsigned num_sides,
		float roof_angle,
		float base_height = 0.f
	) {
		auto radius_segment = floor_vertex - floor_center;
		auto base_angle = get_angle(radius_segment);
		auto points = make_regular_polygon(num_sides, ygl::length(radius_segment), base_angle);
		for (auto& p : points) p += floor_center;
		return make_roof_pyramid_from_border(
			points, tanf(roof_angle)*ygl::length(radius_segment), base_height
		);
	}

	// Usually not recommended
	ygl::shape* make_roof_pyramid_from_main_points(
		const std::vector<ygl::vec2f>& floor_main_points,
		float floor_width,
		float roof_height,
		float base_height = 0.f
	) {
		auto border = to_2d(make_wide_line_border(floor_main_points, floor_width));
		return make_roof_pyramid_from_border(
			border, roof_height, base_height
		);
	}

	ygl::shape* make_roof_crossgabled_thickness(
		const std::vector<ygl::vec2f>& floor_main_points,
		float floor_width,
		float roof_angle,
		float thickness,
		float rake_overhang,
		float roof_overhang,
		float base_height = 0.f
	) {
		if (rake_overhang < 0.f || roof_overhang < 0.f) {
			throw std::runtime_error("Invalid arguments.");
		}
		float center_height = tanf(roof_angle)*floor_width / 2.f;
		auto floor_border = make_wide_line_border(floor_main_points, floor_width);
		auto shp = new ygl::shape();
		
		// Law of sines: 
		//		a/sin(A) = b/sin(B) = c/sin(C), where a,b and c are the lengths
		//		of the sides and A,B and C are the opposite angles
		// We calculate the thickened roof's top height as follows:
		//		Let ABC be a triangle with:
		//		- a : thickness segment
		//		- b : height segment
		//		- c : segment between a and b
		//		Then:
		//		- C = roof_angle
		//		- B = pi/2
		//		- A = pi - C - A = pi - pi/2 - roof_angle = pi/2 - roof_angle
		//		For the law of sines, a/sin(A) = b/sin(B) = b/sin(pi/2) = b
		//		-> b = a/sin(A) = thickness / sin(pi/2 - roof_angle)
		auto thick_height = thickness / sinf(yb::pi / 2.f - roof_angle);
		auto thick_width = thickness / sinf(roof_angle); // Same calculations

		// For each main point we generate six vertexes:
		// - The original right, left and top, with overhangs
		// - As above, but including thickness
		shp->pos.push_back(floor_border[0]);
		shp->pos.push_back(floor_border.back());
		shp->pos.push_back(
			(floor_border[0] + floor_border.back()) / 2.f + 
			ygl::vec3f{0, center_height, 0}
		);
		auto to_right = ygl::normalize(floor_border[0] - floor_border.back());
		shp->pos.push_back(shp->pos[0] + to_right*thick_width);
		shp->pos.push_back(shp->pos[1] - to_right*thick_width);
		shp->pos.push_back(shp->pos[2] + ygl::vec3f{0, thick_height, 0});

		for (int i = 0, j = floor_border.size()-1; i < j-2; i++, j--) {
			// Six new points again, for the next main point
			auto midpoint_next = (floor_border[i + 1] + floor_border[j - 1]) / 2.f;
			midpoint_next += {0, center_height, 0};
			auto ps = shp->pos.size(); // For readability
			shp->pos.push_back(floor_border[i+1]);
			shp->pos.push_back(floor_border[j-1]);
			shp->pos.push_back(midpoint_next);
			to_right = ygl::normalize(floor_border[i + 1] - floor_border[j - 1]);
			shp->pos.push_back(shp->pos[ps] + to_right*thick_width);
			shp->pos.push_back(shp->pos[ps + 1] - to_right*thick_width);
			shp->pos.push_back(shp->pos[ps + 2] + ygl::vec3f{0, thick_height, 0});

			//Faces
			int bi = ps - 6; // First index to put in the following quads
			                 // floor_border[i]
			// Internal quads (they're seen from below)
			shp->quads.push_back({ bi,bi + 2,bi + 8,bi + 6 });
			shp->quads.push_back({ bi + 1,bi + 2,bi + 8,bi + 7 });
			// Upper quads (seen from above)
			shp->quads.push_back({ bi + 3, bi + 9, bi + 11, bi + 5 });
			shp->quads.push_back({ bi + 4,bi + 5,bi + 11,bi + 10 });
			// Bottom flat quads (seen from below)
			shp->quads.push_back({ bi + 1,bi + 4,bi + 10,bi + 7 });
			shp->quads.push_back({ bi,bi + 6,bi + 9,bi + 3 });
			// Vertical, front-facing
			if (i == 0) { // Skip hidden faces
				shp->quads.push_back({ bi + 1,bi + 2,bi + 5,bi + 4 });
				shp->quads.push_back({ bi,bi + 3,bi + 5,bi + 2 });
			}
			// Vertical, rear-facing
			if (i == j - 3) {
				shp->quads.push_back({ bi + 6,bi + 8,bi + 11,bi + 9 });
				shp->quads.push_back({ bi + 7,bi + 10,bi + 11,bi + 8 });
			}
		}
		if (base_height != 0.f) {
			yb::displace(shp->pos, { 0,base_height,0 });
		}
		if (rake_overhang > 0.f) {
			auto dir = ygl::normalize(shp->pos[6] - shp->pos[0]);
			for (int i = 0; i < 6; i++) {
				shp->pos[i] -= dir*rake_overhang;
			}
			dir = ygl::normalize(shp->pos[shp->pos.size() - 1] - shp->pos[shp->pos.size() - 7]);
			for (int i = shp->pos.size() - 6; i < shp->pos.size(); i++) {
				shp->pos[i] += dir*rake_overhang;
			}
		}
		if (roof_overhang > 0.f) {
			for (int i = 0; i < shp->pos.size(); i+=6) {
				auto to_top = ygl::normalize(shp->pos[i + 2] - shp->pos[i]);
				//Law of sines again
				auto length = roof_overhang / sinf(yb::pi / 2.f - roof_angle);
				shp->pos[i] -= to_top*length;
				shp->pos[i + 3] -= to_top*length;
				to_top = { -to_top.x, to_top.y, -to_top.z };
				shp->pos[i + 1] -= to_top*length;
				shp->pos[i + 4] -= to_top*length;
			}
		}
		shp->norm = ygl::compute_normals({}, {}, shp->quads, shp->pos);
		return shp;
	}

	// Floors

	float get_building_height(unsigned num_floors, float floor_height, float belt_height) {
		return num_floors*floor_height + (num_floors - 1)*belt_height;
	}

	std::vector<ygl::vec2f> __make_floor_border_from_main_points(
		const std::vector<ygl::vec2f>& floor_main_points,
		float floor_width
	) {
		return to_2d(make_wide_line_border(floor_main_points, floor_width));
	}

	std::vector<ygl::vec2f> __make_floor_border_from_regular(
		const ygl::vec2f& floor_center,
		const ygl::vec2f& floor_vertex,
		unsigned num_sides
	) {
		if (num_sides < 3) {
			throw std::runtime_error("Invalid arguments");
		}
		auto segment = floor_vertex - floor_center;
		auto angle = get_angle(segment);
		auto radius = ygl::length(segment);
		std::vector<ygl::vec2f> points;
		for (int i = 0; i < num_sides; i++) {
			auto a = angle + 2.f*pi / num_sides*i;
			points.push_back(ygl::vec2f{ cos(a),sin(a) }*radius + floor_center);
		}
		return points;
	}

	// Just for consistency :)
	std::vector<ygl::vec2f> __make_floor_border_from_border(
		const std::vector<ygl::vec2f>& border
	) {
		return border;
	}

	std::tuple<ygl::shape*, ygl::shape*> __make_floors_from_border(
		const std::vector<ygl::vec2f>& floor_border,
		unsigned num_floors,
		float floor_height,
		float belt_height,
		float belt_additional_width,
		float width_delta_per_floor = 0.f
	) {
		if (num_floors <= 0 || floor_height <= 0.f || 
			belt_height < 0.f || belt_additional_width <= 0.f) {
			throw std::runtime_error("Invalid arguments");
		}
		auto floor = thicken_polygon(floor_border, floor_height);
		ygl::shape* belt = nullptr;
		if (belt_height <= 0.f) { // Belt is optional
			belt = new ygl::shape();
		}
		else {
			belt = thicken_polygon(
				expand_polygon(floor_border, belt_additional_width), 
				belt_height
			);
			displace(belt->pos, { 0,floor_height,0 });
		}
		auto floor_shp = new ygl::shape();
		auto belt_shp = new ygl::shape();
		merge_shapes(floor_shp, floor);
		for (int i = 1; i < num_floors; i++) {
			merge_shapes(belt_shp, belt);
			if (width_delta_per_floor == 0.f) {
				displace(belt->pos, { 0,floor_height + belt_height,0 });
				displace(floor->pos, { 0,floor_height + belt_height,0 });
			}
			else {
				delete floor;
				delete belt;
				floor = thicken_polygon(
					offset_polygon(floor_border, width_delta_per_floor*i)[0],
					floor_height
				);
				belt = thicken_polygon(
					offset_polygon(
						floor_border,
						belt_additional_width + width_delta_per_floor*i
					)[0],
					belt_height
				);
				displace(floor->pos, { 0,(floor_height + belt_height)*i,0 });
				displace(belt->pos, { 0,(floor_height + belt_height)*i + floor_height,0 });
			}
			merge_shapes(floor_shp, floor);
		}

		delete floor;
		delete belt;
		return { floor_shp, belt_shp };
	}

	std::tuple<ygl::shape*, ygl::shape*> make_floors_from_main_points(
		const std::vector<ygl::vec2f>& floor_main_points,
		float floor_width,
		unsigned num_floors,
		float floor_height,
		float belt_height,
		float belt_additional_width,
		float width_delta_per_floor = 0.f
	) {
		if (floor_width <= 0.f || num_floors <= 0 || floor_height <= 0.f || 
			belt_height < 0.f || belt_additional_width < 0.f) {
			throw std::runtime_error("Invalid arguments");
		}

		// This function is mostly copy-pasted from __make_floors_from_border
		// Will fix later :)

		auto floor_border = to_2d(make_wide_line_border(floor_main_points, floor_width));
		auto floor = thicken_polygon(floor_border, floor_height);
		ygl::shape* belt = nullptr;
		if (belt_height <= 0.f) { // Belt is optional
			belt = new ygl::shape();
		}
		else {
			belt = thicken_polygon(
				expand_polygon(floor_border, belt_additional_width),
				belt_height
			);
			displace(belt->pos, { 0,floor_height,0 });
		}

		auto floor_shp = new ygl::shape();
		auto belt_shp = new ygl::shape();
		merge_shapes(floor_shp, floor);
		for (int i = 1; i < num_floors; i++) {
			merge_shapes(belt_shp, belt);
			if (width_delta_per_floor == 0.f) {
				displace(belt->pos, { 0,floor_height + belt_height,0 });
				displace(floor->pos, { 0,floor_height + belt_height,0 });
			}
			else {
				delete floor;
				delete belt;
				auto floor_border = to_2d(make_wide_line_border(
					floor_main_points, 
					floor_width + width_delta_per_floor*i
				));
				floor = thicken_polygon(
					floor_border,
					floor_height
				);
				belt = thicken_polygon(
					offset_polygon(
						floor_border,
						belt_additional_width
					)[0],
					belt_height
				);
				displace(floor->pos, { 0,(floor_height + belt_height)*i,0 });
				displace(belt->pos, { 0,(floor_height + belt_height)*i + floor_height,0 });
			}
			merge_shapes(floor_shp, floor);
		}

		delete floor;
		delete belt;
		return { floor_shp, belt_shp };
	}

	std::tuple<ygl::shape*, ygl::shape*> make_floors_from_regular(
		const ygl::vec2f& floor_center,
		unsigned num_sides,
		float radius,
		float base_angle,
		unsigned num_floors,
		float floor_height,
		float belt_height,
		float belt_additional_width,
		float width_delta_per_floor = 0.f
	) {
		auto floor_border = make_regular_polygon(
			num_sides,
			radius,
			base_angle
		);
		for (auto& p : floor_border) p += floor_center;

		// This function is mostly copy-pasted from __make_floors_from_border
		// Will fix later :)

		auto floor = thicken_polygon(floor_border, floor_height);
		ygl::shape* belt = nullptr;
		if (belt_height <= 0.f) { // Belt is optional
			belt = new ygl::shape();
		}
		else {
			belt = thicken_polygon(
				expand_polygon(floor_border, belt_additional_width),
				belt_height
			);
			displace(belt->pos, { 0,floor_height,0 });
		}

		auto floor_shp = new ygl::shape();
		auto belt_shp = new ygl::shape();
		merge_shapes(floor_shp, floor);
		for (int i = 1; i < num_floors; i++) {
			merge_shapes(belt_shp, belt);
			if (width_delta_per_floor == 0.f) {
				displace(belt->pos, { 0,floor_height + belt_height,0 });
				displace(floor->pos, { 0,floor_height + belt_height,0 });
			}
			else {
				delete floor;
				delete belt;
				auto floor_border = make_regular_polygon(
					num_sides, radius+width_delta_per_floor*i, base_angle
				);
				for (auto& p : floor_border) p += floor_center;
				floor = thicken_polygon(
					floor_border,
					floor_height
				);
				belt = thicken_polygon(
					offset_polygon(
						floor_border,
						belt_additional_width
					)[0],
					belt_height
				);
				displace(floor->pos, { 0,(floor_height + belt_height)*i,0 });
				displace(belt->pos, { 0,(floor_height + belt_height)*i + floor_height,0 });
			}
			merge_shapes(floor_shp, floor);
		}

		delete floor;
		delete belt;
		return { floor_shp, belt_shp };
	}

	auto& make_floors_from_border = __make_floors_from_border;

	// Walls

	/**
	 * Makes a wall.
	 *
	 * If closed is true, the last input point is connected to the first.
	 */
	ygl::shape* make_wall(
		const std::vector<ygl::vec2f> points,
		float thickness,
		float height,
		bool closed = false
	) {
		ygl::shape* shp;
		if (!closed) {
			shp = thicken_polygon(
				make_wide_line_border(points, thickness),
				height
			);
		}
		else {
			auto ext_border = expand_polygon(points, thickness / 2.f);
			// Assuming only one output polygon
			auto int_border = offset_polygon(points, -thickness / 2.f)[0];
			std::reverse(int_border.begin(), int_border.end());
			shp = thicken_polygon(
				ext_border, height, { int_border }
			);
		}
		return shp;
	}

	// Windows

	void check_win_info(const windows_params& win_info) {
		const auto& w = win_info;
		if (w.windows_distance < 0.f ||
			w.closed_window_shape == nullptr || w.open_window_shape == nullptr ||
			w.open_windows_ratio < 0.f || w.open_windows_ratio > 1.f ||
			w.filled_spots_ratio < 0.f || w.filled_spots_ratio > 1.f
			) {
			throw std::runtime_error("Invalid windows parameters");
		}
	}

	/**
	 * Makes all the windows for a building.
	 *
	 * Since it needs a different material than the rest of the building,
	 * the shape (material included) is taken as input and a vector of instances
	 * (one for each window) is returned. The shapes are assumed to be centered around
	 * {0,0,0} (on all three dimensions!).
	 */
	std::vector<ygl::instance*> make_windows(
		const building_params& params
	) {
		check_win_info(params.win_pars);

		int win_id = 0; // Windows's unique id for instances' names
		std::vector<ygl::instance*> windows;
		for (int i = 0; i < params.num_floors; i++) {
			std::vector<ygl::vec2f> border;
			switch (params.type) {
			case building_type::main_points:
				border = to_2d(make_wide_line_border(
					params.floor_main_points, 
					params.floor_width + params.width_delta_per_floor*i
					)
				);
				break;
			case building_type::border: 
				border = offset_polygon(params.floor_border, params.width_delta_per_floor*i)[0]; 
				break;
			case building_type::regular:
				border = make_regular_polygon(
					params.num_sides, 
					params.radius + params.width_delta_per_floor*i, 
					params.reg_base_angle
				);
				for (auto& p : border) p += params.floor_center;
				break;
			default:
				throw std::runtime_error("Invalid building type");
			}
			for_sides(border, [&](const ygl::vec2f& p1, const ygl::vec2f& p2) {
				auto side = p2 - p1;
				auto eps = params.win_pars.windows_distance_from_edges; // Min distance between window and corner
				auto W = ygl::length(side); // Side length

				auto w = get_size(params.win_pars.closed_window_shape).x; // Window's width
				if (get_size(params.win_pars.open_window_shape).x > w)
					w = get_size(params.win_pars.open_window_shape).x;

				auto s = params.win_pars.windows_distance;
				int n; // Number of windows fitting on this side within the given constraints
				if (w + 2 * eps >= W) n = 0;
				else n = int((W - w - 2 * eps) / (w + s)) + 1;
				if (n <= 0) return;

				// Having found the number of windows, distribute them more uniformly
				s = (W - 2 * eps - n*w) / (n - 1);
				if (n == 1) eps = W / 2.f; // Special case: 1 window is on the center

				auto dir = ygl::normalize(side); // Side versor
				for (int j = 0; j < n; j++) { // n windows per size
					auto win_center_xz = p1 + dir*(eps + w / 2.f + (w + s)*j);
					// Keep around f_p_s percent of windows
					if (!bernoulli(*params.rng, params.win_pars.filled_spots_ratio))
						continue;

					auto win_center_y =
						params.floor_height / 2.f +
						(params.floor_height + params.belt_height)*i;

					auto win_inst = new ygl::instance();
					win_inst->name = params.win_pars.name + "_" + std::to_string(win_id++);
					win_inst->shp =
						bernoulli(*params.rng, params.win_pars.open_windows_ratio) ?
						params.win_pars.open_window_shape :
						params.win_pars.closed_window_shape;
					win_inst->frame.o = to_3d(win_center_xz, win_center_y);
					rotate_y(win_inst->frame.x, get_angle(side));
					rotate_y(win_inst->frame.z, get_angle(side));
					windows.push_back(win_inst);
				}
			});
		}
		return windows;
	}

	std::tuple<ygl::shape*, ygl::shape*> make_test_windows(
		const std::string& name_open, const std::string& name_closed
	) {
		auto regwnd_shp = new ygl::shape();
		std::tie(regwnd_shp->quads, regwnd_shp->pos) = make_parallelepidedon(1.6f, 1.f, .10f);
		set_shape_normals(regwnd_shp);
		center_points(regwnd_shp->pos);
		regwnd_shp->mat = make_material(name_open + "_mat", { 0.8f,0.8f,1.f }, nullptr, { 0.8f,0.8f,0.8f });
		regwnd_shp->name = name_open + "_shape";

		auto regwnd_close_shp = new ygl::shape();
		std::tie(regwnd_close_shp->quads, regwnd_close_shp->pos) =
			make_parallelepidedon(1.f, 1.f, .10f);
		center_points(regwnd_close_shp->pos);
		set_shape_normals(regwnd_close_shp);
		regwnd_close_shp->mat = make_material(name_closed + "_mat", { 0.3f,0.1f,0.f }, nullptr, { 0,0,0 });
		regwnd_close_shp->name = name_closed + "_shape";

		return { regwnd_shp, regwnd_close_shp };
	}

	building_params* make_rand_building_params(
		ygl::rng_pcg32& rng,
		ygl::shape *open_window_shape,
		ygl::shape *closed_window_shape,
		std::string id
	) {
		building_params *params = new building_params();
		params->type = choose_random_weighted(
			rng,
			std::vector<building_type>{
				building_type::main_points,
				building_type::regular
			},
			{ 90.f, 10.f }
		);
		int num_segments = choose_random(rng, std::vector<int>{ 3,4,5,6,7 });
		params->floor_main_points = make_segmented_line(
		{ 0,0 }, num_segments, yb::pi / 2.f,
			[&rng]() {
				float v = 0.f;
				while (v == 0.f) v = yb::uniform(rng, -yb::pi / 5.f, yb::pi / 5.f);
				return v;
			},
			[&rng]() {return yb::gaussian(rng, 10.f, 1.f);}
		);
		params->floor_width = yb::uniform(rng, 10.f, 25.f);
		params->floor_border = { {10,10},{0,5},{-10,10},{-5,0},{-10,-10},{0,-5},{10,-10},{5,0} };
		params->floor_center = { 0,0 };
		params->num_sides = ygl::next_rand1i(rng, 2) + 3;
		params->radius = uniform(rng, 10.f, 15.f);
		params->reg_base_angle = uniform(rng, 0.f, pi);

		params->num_floors = ygl::next_rand1i(rng, 6) + 3;
		params->floor_height = yb::uniform(rng, 2.5f, 5.f);
		params->belt_height = yb::uniform(rng, 0.25f, 0.45f);
		params->belt_additional_width = yb::uniform(rng, 0.25f, 0.45f);
		params->id = id + "_building";
		params->color1 = yb::rand_color3f(rng);
		params->color2 = yb::rand_color3f(rng);
		params->width_delta_per_floor = uniform(rng, -0.15f, .25f);
		params->rng = &rng;

		if (params->type == building_type::main_points) {
			params->roof_pars.type = yb::choose_random_weighted(
				rng,
				std::vector<roof_type>{ 
					roof_type::crossgabled,
					roof_type::crosshipped,
					roof_type::pyramid,
					roof_type::none
				},
				std::vector<float>{75.f, 10.f, 10.f, 5.f}
			);
			params->roof_pars.type = roof_type::none;
		}
		else {
			params->roof_pars.type = yb::choose_random_weighted(
				rng,
				std::vector<roof_type>{roof_type::pyramid, roof_type::none},
				std::vector<float>{85.f, 15.f}
			);
		}
		params->roof_pars.color1 = yb::rand_color3f(rng);
		params->roof_pars.roof_angle = uniform(rng, pi / 10.f, pi/3.f);
		params->roof_pars.thickness = uniform(rng, 0.25f, 0.75f);
		params->roof_pars.color2 = yb::rand_color3f(rng);
		params->roof_pars.rake_overhang = uniform(rng, 0.1f, 2.f);
		params->roof_pars.roof_overhang = uniform(rng, 0.1f, 1.f);
		auto max_hip_depth = std::max<float>(std::min<float>(
			ygl::length(params->floor_main_points[1] - params->floor_main_points[0]),
			ygl::length(
				params->floor_main_points.back() -
				params->floor_main_points[params->floor_main_points.size() - 2]
			)
		)*0.9f, 0.f);
		params->roof_pars.hip_depth = uniform(rng, 0.f, max_hip_depth);
		params->roof_pars.roof_height = uniform(rng, 3.f, 13.f);

		params->win_pars.name = id + "_wnd";
		params->win_pars.windows_distance = uniform(rng, .1f, .5f);
		params->win_pars.windows_distance_from_edges = uniform(rng, .2f, .5f);
		params->win_pars.closed_window_shape = closed_window_shape;
		params->win_pars.open_window_shape = open_window_shape;
		params->win_pars.open_windows_ratio = uniform(rng, 0, 1);
		params->win_pars.filled_spots_ratio = uniform(rng, 0, 1);

		params->tower_prob = 0.6f;
		params->tower_on_shrinking_building = false;
		params->tower_num_floors = uniform(rng, 2, params->num_floors+10);
		params->tower_num_sides = ygl::next_rand1i(rng, 20) + 3;
		params->tower_radius = uniform(rng, 4, 10);
		// Returns the longest side
		// In a building generated from main points, the longest side will precede
		// a left turn (i.e. the tower will *probably* be tangent to the building)
		params->tower_placement = [&](const std::vector<ygl::vec2f>& border)->int{
			auto max_length = 0.f;
			auto side = 0;
			auto i = 0;
			for_sides(border, [&](const ygl::vec2f& p1, const ygl::vec2f& p2){
				auto length = ygl::length(p2 - p1);
				if (length > max_length) {
					max_length = length;
					side = i;
				}
				i++;
			});
			return side;
		};

		return params;
	}

	// Whole house

	std::tuple<ygl::shape*, ygl::shape*> make_floors_from_params(
		const building_params& params
	) {
		switch (params.type) {
		case building_type::main_points:
			return make_floors_from_main_points(
				params.floor_main_points, params.floor_width, params.num_floors,
				params.floor_height, params.belt_height, params.belt_additional_width,
				params.width_delta_per_floor
			);
		case building_type::border:
			return make_floors_from_border(
				params.floor_border, params.num_floors, params.floor_height,
				params.belt_height, params.belt_additional_width,
				params.width_delta_per_floor
			);
		case building_type::regular:
			return make_floors_from_regular(
				params.floor_center,
				params.num_sides, params.radius, params.reg_base_angle,
				params.num_floors, params.floor_height, params.belt_height,
				params.belt_additional_width, params.width_delta_per_floor
			);
		default:
			throw std::runtime_error("Invalid building type");
		}
	}

	std::tuple<ygl::shape*, ygl::shape*> make_roof_from_params(const building_params& params) {
		const auto& r_pars = params.roof_pars; // Shorter alias
		auto base_height = get_building_height(
			params.num_floors, params.floor_height, params.belt_height
		);
		auto floor_width =
			params.floor_width +
			params.width_delta_per_floor * (params.num_floors - 1);
		
		ygl::shape* r_shp = nullptr; // Main body of the roof
		ygl::shape* t_shp = nullptr; // Roof thickness with overhangs

		switch (r_pars.type) {
		case roof_type::none:
			r_shp = new ygl::shape(); // Empty shape
			break;
		case roof_type::crossgabled: 
			if (params.type != building_type::main_points) {
				throw std::runtime_error("Invalid parameters");
			}
			r_shp = make_roof_crossgabled_simple(
				params.floor_main_points,
				floor_width,
				r_pars.roof_angle,
				base_height
			);
			if (r_pars.thickness > 0.f) {
				t_shp = make_roof_crossgabled_thickness(
					params.floor_main_points, floor_width, r_pars.roof_angle,
					r_pars.thickness, r_pars.rake_overhang, r_pars.roof_overhang,
					base_height
				);
			}
			break;
		case roof_type::crosshipped:
			if (params.type != building_type::main_points) {
				throw std::runtime_error("Invalid parameters");
			}
			r_shp = make_roof_crosshipped_simple(
				params.floor_main_points, floor_width, r_pars.roof_angle,
				r_pars.hip_depth, base_height
			);
			break;
		case roof_type::pyramid:
			switch (params.type) {
			case building_type::main_points:
				r_shp = make_roof_pyramid_from_main_points(
					params.floor_main_points, floor_width, r_pars.roof_height,
					base_height
				);
				break;
			case building_type::border:
				r_shp = make_roof_pyramid_from_border(
					offset_polygon(
						params.floor_border,
						params.width_delta_per_floor * (params.num_floors - 1)
					)[0],
					r_pars.roof_height,
					base_height
				);
				break;
			case building_type::regular: {
				auto reg_border = make_regular_polygon(
					params.num_sides,
					params.radius + params.width_delta_per_floor*(params.num_floors - 1),
					params.reg_base_angle
				);
				for (auto& p : reg_border) p += params.floor_center;
				r_shp = make_roof_pyramid_from_border(
					reg_border,
					r_pars.roof_height,
					base_height
				);
				break;
			}
			default:
				throw std::runtime_error("Invalid building type");
			}
			break;
		default:
			throw std::runtime_error("Invalid roof type");
		}
		return { r_shp, t_shp != nullptr ? t_shp : new ygl::shape() };
	}

	// Definition later
	std::vector<ygl::instance*> make_building(
		const building_params& params,
		float base_height = 0.f
	);

	std::vector<ygl::instance*> make_tower_from_params(
		const building_params& params
	) {
		std::vector<ygl::vec2f> border;
		switch (params.type) {
		case building_type::main_points:
			border = to_2d(make_wide_line_border(params.floor_main_points, params.floor_width));
			break;
		case building_type::border:
			border = params.floor_border;
			break;
		case building_type::regular:
			border = make_regular_polygon(params.num_sides, params.radius, params.reg_base_angle);
			break;
		}
		auto side = params.tower_placement(border);
		ygl::vec2f b1, b2; // Building's side
		if (side != border.size() - 1) {
			b1 = border[side];
			b2 = border[side + 1];
		}
		else {
			b1 = border.back();
			b2 = border[0];
		}
		auto building_border_center = (b1+b2)/2;
		auto alpha = (2*pi) / params.tower_num_sides; // Angle between two vertices of the tower's floor
		auto gamma = pi / 2 - (alpha/2); // Last angle of the triangle between vertex, border center and tower center
		auto half_side_length = params.tower_radius*sin(alpha / 2);
		auto dist_center_from_border = sqrtf(powf(params.tower_radius, 2) - powf(half_side_length, 2)); // Pitagora's theorem
		auto b2t_angle = get_angle(b2 - b1) - pi / 2;  // Border side's center to tower center
		auto tower_floor_center =
			building_border_center + ygl::vec2f(cos(b2t_angle), sin(b2t_angle))*dist_center_from_border;
		auto reg_base_angle = get_angle(b2 - b1) + (pi - gamma);

		auto tower_params = params;
		// Manually fix parameters
		tower_params.id = params.id + "_tower";
		tower_params.type = building_type::regular;
		tower_params.num_floors = params.tower_num_floors;
		tower_params.num_sides = params.tower_num_sides;
		tower_params.radius = params.tower_radius;
		tower_params.roof_pars.type = roof_type::pyramid;
		tower_params.tower_prob = 0.f;
		 
		auto tower_insts = make_building(tower_params);
		for (auto ti : tower_insts) {
			translate(ti, to_3d(tower_floor_center));
		}
		return tower_insts;
	}

	std::vector<std::vector<ygl::vec2f>> recursive_main_points(
		const building_params& params
	) {
		auto mainpts_subs = random_substrings(
			*params.rng,
			params.floor_main_points,
			params.roof_pars.keep_prob,
			params.roof_pars.continue_prob,
			1
		);

		if (mainpts_subs.size() == 1 &&
			mainpts_subs.front().size() != 1 && // Recursive towers allowed
			// Recursive building spans whole roof
			mainpts_subs.front().size() == params.floor_main_points.size() 
			) {
			if (bernoulli(*params.rng, 0.5f)) {
				mainpts_subs.front().erase(mainpts_subs.front().begin());
			}
			else {
				mainpts_subs.front().pop_back();
			}
		}
		for (auto& sub : mainpts_subs) {
			if (sub.size() >= 2) {
				sub[0] += (sub[1] - sub[0]) / 2.f;
				sub.back() -= (sub.back() - sub[sub.size() - 1]) / 2.f;
			}
		}
		return mainpts_subs;
	}

	std::vector<ygl::instance*> make_building(
		const building_params& params,
		float base_height
	) {
		std::vector<ygl::instance*> instances;
		
		auto h_shp = make_floors_from_params(params);
		instances += make_instance(
			params.id + "_h1",
			std::get<0>(h_shp),
			make_material("", params.color1, nullptr, { 0,0,0 })
		);
		instances += make_instance(
			params.id + "_h2",
			std::get<1>(h_shp),
			make_material("", params.color2, nullptr, { 0,0,0 })
		);

		auto r_shp = make_roof_from_params(params);
		instances += make_instance(
			params.id + "_rr",
			std::get<0>(r_shp),
			make_material("", params.roof_pars.color1, nullptr, { 0,0,0 })
		);
		instances += make_instance(
			params.id + "_rt",
			std::get<1>(r_shp),
			make_material("", params.roof_pars.color2, nullptr, { 0,0,0 })
		);
		
		auto w_insts = make_windows(params);
		instances.insert(instances.end(), w_insts.begin(), w_insts.end());

		for (auto i : instances) {
			translate(i, ygl::vec3f{ 0, base_height, 0 });
		}

		// Recursive buildings
		if (params.type == building_type::main_points &&
			//params.roof_pars.type == roof_type::none &&
			bernoulli(*params.rng, params.roof_pars.recursive_prob)
		) {
			auto rec_params =
				params.roof_pars.recursive_params != nullptr ?
				params.roof_pars.recursive_params :
				make_rand_building_params(
					*params.rng,
					params.win_pars.open_window_shape,
					params.win_pars.closed_window_shape,
					params.id + "_rec"
				);
			auto mainpts_subs = recursive_main_points(params);

			// Necessary parameter adjustments
			auto total_width_offset = params.width_delta_per_floor*(params.num_floors - 1);
			rec_params->floor_width = std::min(
				rec_params->floor_width, 
				(params.floor_width+total_width_offset)*0.85f
			);
			rec_params->num_sides = params.num_sides;
			rec_params->radius = std::min(
				rec_params->radius, 
				(params.floor_width+total_width_offset)/2.f*0.75f
			);
			rec_params->reg_base_angle = params.reg_base_angle;
			rec_params->tower_prob = 0.f;
			
			// Color homogeneity
			rec_params->color1 = params.color1;
			rec_params->color2 = params.color2;
			rec_params->roof_pars.color1 = params.roof_pars.color1;
			rec_params->roof_pars.color2 = params.roof_pars.color2;
			
			for (int i = 0; i < mainpts_subs.size(); i++) {
				rec_params->id += std::to_string(i);
				const auto& mainpts = mainpts_subs[i];
				if (mainpts.size() > 1) {
					rec_params->type = building_type::main_points;
					rec_params->floor_main_points = mainpts;
					rec_params->roof_pars.type = roof_type::crossgabled;
				}
				else {
					rec_params->type = building_type::regular;
					rec_params->floor_center = mainpts[0];
					rec_params->roof_pars.type = roof_type::pyramid;
				}
				auto rec_insts = make_building(
					*rec_params,
					base_height + get_building_height(params.num_floors, params.floor_height, params.belt_height)
				);
				for (auto ri : rec_insts) {
					instances.push_back(ri);
				}
			}
		}

		if ((params.width_delta_per_floor <= 0.f || params.tower_on_shrinking_building) &&
			bernoulli(*params.rng, params.tower_prob)
			) {
			for (auto ti : make_tower_from_params(params)) {
				instances.push_back(ti);
			}
		}

		return instances;
	}
}

#endif // BUILDING_UTILS_H