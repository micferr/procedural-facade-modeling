#ifndef GEOM_UTILS_H
#define GEOM_UTILS_H

#include <vector>

#include "yocto\yocto_gl.h"
#include "poly2tri\poly2tri\poly2tri.h"
#include "clipper\clipper.hpp"
#include "yocto_utils.h"

namespace yb {
	const float pi = 3.14159265359f;

	ygl::vec2f centroid(const std::vector<ygl::vec2f>& points) {
		// Average of the input points
		ygl::vec2f c = { 0,0 };
		for (auto& p : points) c += p;
		return c / points.size();
	}

	ygl::vec3f centroid(const std::vector<ygl::vec3f>& points) {
		ygl::vec3f c = { 0,0,0 };
		for (auto& p : points) c += p;
		return c / points.size();
	}

	/**
	* Discards the y component of 3d points
	*/
	ygl::vec2f to_2d(const ygl::vec3f& point, bool flip_y = true) {
		return { point.x, flip_y ? -point.z : point.z };
	}

	std::vector<ygl::vec2f> to_2d(
		const std::vector<ygl::vec3f>& points, bool flip_y = true
	) {
		std::vector<ygl::vec2f> res;
		for (const auto& p : points) {
			res.push_back(to_2d(p,flip_y));
		}
		return res;
	}

	/**
	 * Transforms 2D points (x,z) to 3D points (x,y,-z), with y in input.
	 * z is flipped by default to preserve clockwiseness
	 */
	ygl::vec3f to_3d(const ygl::vec2f& point, float y = 0.f, bool flip_z = true) {
		return { point.x, y, flip_z ? -point.y : point.y };
	}

	std::vector<ygl::vec3f> to_3d(
		const std::vector<ygl::vec2f>& points, 
		float y = 0.f,
		bool flip_z = true
	) {
		std::vector<ygl::vec3f> res;
		for (const auto& p : points) {
			res.push_back(to_3d(p, y, flip_z));
		}
		return res;
	}

	/**
	 * Swaps the Y and Z components of the input points
	 */
	void swap_yz(std::vector<ygl::vec3f>& points) {
		for (auto& p : points) p = { p.x, p.z, p.y };
	}

	/**
	 * Return the points of a 2D polygon.
	 * The first point is set at (radius,0) if the polygon is not aligned,
	 * else it's at radius*(cos(a),sin(a)), a = 2*pi/(2*num_sides)
	 */
	std::vector<ygl::vec2f> make_regular_polygon(
		int num_sides, 
		float radius = 1.f,
		float base_angle = 0.f
	) {
		if (num_sides <= 2) {
			throw std::exception("A polygon must have at least 3 sides.");
		}
		std::vector<ygl::vec2f> res;
		float angle_step = 2 * pi / num_sides;
		for (int i = 0; i < num_sides; i++) {
			float angle = i*angle_step + base_angle;
			res.push_back({ cos(angle)*radius, sin(angle)*radius });
		}
		return res;
	}

	/**
	 * Returns a 2D regular polygon in 3D space, with a fixed
	 * value of 0 for the y component of the points.
	 * Face normal is (0,1,0)
	 */
	std::vector<ygl::vec3f> make_regular_polygonXZ(
		int num_sides,
		float radius = 1.f,
		float base_angle = 0.f
	) {
		return to_3d(make_regular_polygon(num_sides, radius, base_angle));
	}

	std::vector<ygl::vec2f> make_quad(float side_length = 1.f) {
		float s = side_length/2.f;
		return { {s,s},{-s,s},{-s,-s},{s,-s} };
	}

	std::vector<ygl::vec3f> make_quadXZ(float side_length = 1.f) {
		auto quad = to_3d(make_quad(side_length));
		return quad;
	}

	/**
	 * Returns the point of a line made of 'steps' consecutive, connected
	 * segments.
	 * 
	 * Arguments:
	 *		start : Position of the first vertex of the line
	 *		steps : Number of segments in the line
	 *		start_alpha : Angle of the first generated segment
	 *		alpha_delta_per_step : Generator of subsequent angle changes
	 *			(the returned angle is a delta to add to the previous value).
	 *		segment_length_per_step : Generator of subsequent segment lengths
	 */
	std::vector<ygl::vec2f> make_segmented_line(
		const ygl::vec2f& start,
		unsigned steps,
		float start_alpha,
		const std::function<float()>& alpha_delta_per_step,
		const std::function<float()>& segment_length_per_step
	) {
		std::vector<ygl::vec2f> points;
		points.push_back(start);
		float alpha = start_alpha;
		for (int i = 0; i < steps; i++) {
			alpha += !i ? 0.f : alpha_delta_per_step();
			const ygl::vec2f& last_pos = points.back();
			float segment_length = segment_length_per_step();
			points.push_back(
				last_pos + ygl::vec2f{cos(alpha), sin(alpha)}*segment_length
			);
		}
		return points;
	}

	/**
	 * Overridden version where the starting angle is also
	 * randomly generated
	 */
	std::vector<ygl::vec2f> make_segmented_line(
		const ygl::vec2f& start,
		unsigned steps,
		const std::function<float()>& alpha_delta_per_step,
		const std::function<float()>& segment_length_per_step
	) {
		return make_segmented_line(
			start,
			steps,
			alpha_delta_per_step(),
			alpha_delta_per_step,
			segment_length_per_step
		);
	}

	/**
	 * Returns a 2d surface obtained from widening the input line.
	 * It is currently assumed that the resulting polygon will be simple.
	 */
	std::tuple<std::vector<ygl::vec4i>, std::vector<ygl::vec3f>> make_wide_line(
		const std::vector<ygl::vec2f>& points,
		float width,
		bool lengthen_ends = true
	) {
		auto half_width = width / 2.f;
		auto _points = points; // Work-around to keep code cleaner :)
		std::vector<ygl::vec2f> vertexes;
		if (lengthen_ends) {
			_points[0] -= ygl::normalize(points[1] - points[0]) * half_width;
			_points.back() += ygl::normalize(points[points.size() - 1] - points[points.size() - 2]) * half_width;
		}
		_points.push_back(2*_points[_points.size()-1] - _points[_points.size()-2]); //p.back() + to(p[p.size()-2], p.back())
		_points.insert(_points.begin(), _points[0] - (_points[1]-_points[0]));
		for (int i = 1; i < _points.size() - 1; i++) {
			const auto& p1 = _points[i - 1];
			const auto& p2 = _points[i];
			const auto& p3 = _points[i + 1];
			auto p1_to_p3 = p3 - p1;
			auto alpha = atan2f(p1_to_p3.y, p1_to_p3.x); // Angle from p1 to p3
			ygl::vec2f delta = ygl::vec2f{ cos(alpha + pi / 2.f), sin(alpha + pi / 2.f) }*half_width;
			vertexes.push_back(p2 - delta); // Right side of segment
			vertexes.push_back(p2 + delta); // Left side of segment
		}
		std::vector<ygl::vec4i> quads;
		for (int i = 0; i < vertexes.size()-4; i += 2) {
			quads.push_back({ i,i + 1,i + 3,i + 2 });
		}
		return { quads, to_3d(vertexes) };
	}

	/**
	 * Merges collinear consecutive sides in a polygon.
	 *
	 * It is assumed that the polygon is well-formed (e.g. the vertexes are not all
	 * collinear)
	 */
	void clean_collinear_vertexes(std::vector<ygl::vec2f>& points, float alpha_eps = 0.001f) {
		points.push_back(points[0]); // Easier to loop on all sides
		for (int i = 1; i < points.size()-1; i++) {
			auto segment1 = points[i] - points[i - 1];
			auto segment2 = points[i + 1] - points[i];
			auto segment_tot = points[i + 1] - points[i - 1];
			float alpha1 = atan2f(segment1.y, segment1.x);
			float alpha2 = atan2f(segment2.y, segment2.x);
			float alpha_tot = atan2f(segment_tot.y, segment_tot.x);
			if (fabs(alpha_tot-alpha2) <= alpha_eps) {
				points.erase(points.begin() + i);
			}
			else {
				i++;
			}
		}
		points.pop_back(); // Take out the point we added at the beginning
	}

	std::vector<ygl::vec3f> make_wide_line_border(
		const std::vector<ygl::vec2f>& points,
		float width,
		bool lengthen_ends = true
	) {
		auto pos = std::get<1>(make_wide_line(points, width, lengthen_ends));
		std::vector<ygl::vec3f> res;
		// Go forward along the right side of the line...
		for (int i = 0; i < pos.size(); i += 2) res.push_back(pos[i]);
		// ...then back along the left side
		for (int i = pos.size() - 1; i >= 0; i -= 2) res.push_back(pos[i]);
		return res;
	}

	/**
	 * Offsets a polygon's vertexes to uniformly expand/shrink it.
	 */
	std::vector<std::vector<ygl::vec2f>> offset_polygon(
		const std::vector<ygl::vec2f>& polygon,
		float delta,
		unsigned _scale_factor = 100000
	) {
		// Clipper only uses integer, thus we scale twice (first expand,
		// then shrink) to preserve the decimal part.
		int d = delta*_scale_factor;
		ClipperLib::Path poly;
		for (const auto& p : polygon) 
			poly << ClipperLib::IntPoint(int(p.x*_scale_factor), int(p.y*_scale_factor));
		ClipperLib::Paths result;
		ClipperLib::ClipperOffset co;
		co.AddPath(poly, ClipperLib::jtMiter, ClipperLib::etClosedPolygon);
		co.Execute(result, d);
		std::vector<std::vector<ygl::vec2f>> newpolys;
		for (const auto& path : result) {
			std::vector<ygl::vec2f> poly;
			for (const auto& p : path) {
				poly.push_back(ygl::vec2f(float(p.X), float(p.Y)) / float(_scale_factor));
			}
			newpolys.push_back(poly);
		}
		return newpolys;
	}

	/**
	 * Simplified version of offset_polygon, it can only expand (no shrinking),
	 * so we're sure to only have one output polygon
	 */
	std::vector<ygl::vec2f> expand_polygon(
		const std::vector<ygl::vec2f>& polygon,
		float delta,
		unsigned _scale_factor = 10000
	) {
		if (delta < 0.f) throw std::runtime_error("Invalid arguments");
		return offset_polygon(polygon, delta, _scale_factor)[0];
	}

	/**
	* Triangulates an arbitrary shape.
	* Holes must be given in clockwise order
	*/
	std::tuple<std::vector<ygl::vec3i>, std::vector<ygl::vec2f>>
		triangulate(
			const std::vector<ygl::vec2f>& border,
			const std::vector<std::vector<ygl::vec2f>>& holes = {}
		) {
		// Code adapted from https://github.com/greenm01/poly2tri/blob/master/testbed/main.cc
		std::vector<p2t::Point*> polyline;
		for (const auto& p : border) polyline.push_back(new p2t::Point(p.x, p.y));

		std::vector<std::vector<p2t::Point*>> p2t_holes;
		for (const auto& hole : holes) {
			p2t_holes.push_back(std::vector<p2t::Point*>());
			for (const auto& p : hole) p2t_holes.back().push_back(new p2t::Point(p.x, p.y));
		}

		p2t::CDT* cdt = new p2t::CDT(polyline);
		for (const auto& hole : p2t_holes) cdt->AddHole(hole);

		cdt->Triangulate();
		auto p2t_triangles = cdt->GetTriangles();
		auto map = cdt->GetMap();

		std::vector<ygl::vec2f> pos;
		std::vector<ygl::vec3i> triangles;
		int i = 0;
		for (auto t : p2t_triangles) {
			auto p1 = *t->GetPoint(0),
				p2 = *t->GetPoint(1),
				p3 = *t->GetPoint(2);
			pos.push_back({ float(p1.x), float(p1.y) });
			pos.push_back({ float(p2.x), float(p2.y) });
			pos.push_back({ float(p3.x), float(p3.y) });
			triangles.push_back({ i,i + 1,i + 2 });
			i += 3;
		}

		//Cleanup
		for (auto p : polyline) delete p;
		for (const auto& hole : p2t_holes) {
			for (auto p : hole) {
				delete p;
			}
		}

		return { triangles, pos };
	}

	// NB: The points' y coordinate is discarded and then assumed to be 0
	std::tuple<std::vector<ygl::vec3i>, std::vector<ygl::vec3f>>
		triangulate(
			const std::vector<ygl::vec3f>& border,
			const std::vector<std::vector<ygl::vec3f>>& holes = {}
		) {
		std::vector<std::vector<ygl::vec2f>> _holes;
		for (const auto& h : holes) _holes.push_back(to_2d(h));
		auto tt = triangulate(to_2d(border), _holes);
		return { std::get<0>(tt), to_3d(std::get<1>(tt)) };
	}

	/**
	 * Triangulates the shape and inverts the triangles' orientation
	 */
	std::tuple<std::vector<ygl::vec3i>, std::vector<ygl::vec3f>>
		triangulate_opposite(
			const std::vector<ygl::vec3f>& border,
			const std::vector<std::vector<ygl::vec3f>>& holes = {}
		) {
		auto tt = triangulate(border, holes);
		for (auto& t : std::get<0>(tt)) t = { t.z,t.y,t.x };
		return tt;
	}

	/**
	 * Moves all points by the specified value
	 */
	void displace(std::vector<ygl::vec2f>& points, const ygl::vec2f& disp) {
		for (auto& p : points) p += disp;
	}

	void displace(std::vector<ygl::vec3f>& points, const ygl::vec3f& disp) {
		for (auto& p : points) p += disp;
	}

	template<typename T>
	void scale(std::vector<T>& points, float scale) {
		for (auto& p : points) p *= scale;
	}

	/**
	 * Moves the points so that they're centered on the specified axes
	 */
	void center_points(
		std::vector<ygl::vec3f>& points, 
		bool x = true, bool y = true, bool z = true,
		bool weighted = true
	) {
		if (!points.size()) return;

		ygl::vec3f disp = { 0,0,0 };
		if (weighted) {
			disp = -centroid(points);
		}
		else {
			auto min = points[0], max = points[0];
			for (const auto& p : points) {
				if (p.x < min.x) min.x = p.x;
				if (p.x > max.x) max.x = p.x;
				if (p.y < min.y) min.y = p.y;
				if (p.y > max.y) max.y = p.y;
				if (p.z < min.z) min.z = p.z;
				if (p.z > max.z) max.z = p.z;
			}
			disp = -(max - min);
		}
		if (!x) disp.x = 0.f;
		if (!y) disp.y = 0.f;
		if (!z) disp.z = 0.f;
		displace(points, disp);
	}

	/**
	* Rotates the point by 'angle' radiants, counter-clockwise relative to (0,0)
	*/
	void rotate(ygl::vec2f& point, float angle) {
		auto new_angle = atan2f(point.y, point.x) + angle;
		auto length = ygl::length(point);
		point = ygl::vec2f( cos(new_angle), sin(new_angle) )*length;
	}

	void rotate(std::vector<ygl::vec2f>& points, float angle) {
		for (auto& p : points) rotate(p, angle);
	}

	/**
	 * Rotates the point by 'angle' radiants around the y axis
	 * counter-clockwise
	 */
	void rotate_y(ygl::vec3f& point, float angle) {
		auto p2d = to_2d(point);
		rotate(p2d, angle);
		point = to_3d(p2d, point.y);
	}

	void rotate_y(std::vector<ygl::vec3f>& points, float angle) {
		for (auto& p : points) rotate_y(p, angle);
	}

	/**
	 * Executes func on all sides of the polygon (where a side is
	 * two consecutive vertexes)
	 */
	void for_sides(
		const std::vector<ygl::vec2f>& poly,
		const std::function<void(const ygl::vec2f&, const ygl::vec2f&)>& func
	) {
		if (poly.size() < 3) return;
		for (int i = 0; i < poly.size()-1; i++) {
			func(poly[i], poly[i + 1]);
		}
		func(poly.back(), poly.front());
	}

	ygl::vec3f get_size(const ygl::shape* shp) {
		if (shp->pos.size() == 0) return ygl::zero3f;
		auto pmin = shp->pos[0];
		auto pmax = pmin;
		for (const auto& p : shp->pos) {
			if (p.x < pmin.x) pmin.x = p.x;
			if (p.x > pmax.x) pmax.x = p.x;
			if (p.y < pmin.y) pmin.y = p.y;
			if (p.y > pmax.y) pmax.y = p.y;
			if (p.z < pmin.z) pmin.z = p.z;
			if (p.z > pmax.z) pmax.z = p.z;
		}
		return pmax - pmin;
	}

	/**
	* Merges duplicates points in a shape
	*
	* TODO: Remove duplicates from shp->pos
	*/
	void merge_same_points(ygl::shape* shp, float eps = 0.0001f) {
		for (int i = 0; i < shp->pos.size(); i++) {
			for (int j = i + 1; j < shp->pos.size(); j++) {
				if (
					ygl::length(shp->pos[i] - shp->pos[j]) < eps &&
					(shp->color.size() == 0 || ygl::length(shp->color[i]-shp->color[j]) < eps)
				) {
					for (auto& t : shp->triangles) {
						for (auto& p : t) if (p == j) p = i;
					}
					for (auto& q : shp->quads) {
						for (auto& p : q) if (p == j) p = i;
					}
					for (auto& l : shp->lines) {
						for (auto& p : l) if (p == j) p = i;
					}
					for (auto& p : shp->points) if (p == j) p = i;
				}
			}
		}
		shp->norm = ygl::compute_normals(shp->lines, shp->triangles, shp->quads, shp->pos);
	}

	/**
	* Merges the second shape into the first
	*/
	void merge_shapes(
		ygl::shape* s1,
		ygl::shape* s2,
		bool merge_points = true
	) {
		std::tie(s1->lines, s1->triangles, s1->quads) = ygl::merge_elems(
			s1->pos.size(),
			s1->lines, s1->triangles, s1->quads,
			s2->lines, s2->triangles, s2->quads
		);
		s1->pos.insert(s1->pos.end(), s2->pos.begin(), s2->pos.end());
		s1->texcoord.insert(s1->texcoord.end(), s2->texcoord.begin(), s2->texcoord.end());
		s1->norm.insert(s1->norm.end(), s2->norm.begin(), s2->norm.end());
		s1->color.insert(s1->color.end(), s2->color.begin(), s2->color.end());
		if (merge_points) {
			merge_same_points(s1);
		}
	}

	/**
	 * Makes a thick solid from a polygon.
	 *
	 * Thickness is added along the face normal
	 */
	ygl::shape* thicken_polygon(
		const std::vector<ygl::vec3f>& border,
		float thickness,
		const std::vector<std::vector<ygl::vec3f>>& holes = {},
		bool smooth_normals = true
	) {
		// Triangulate floor surface
		auto triangles_data = triangulate(border, holes);
		const auto& triangles = std::get<0>(triangles_data);
		const auto& triangles_pos = std::get<1>(triangles_data);
		// Border vertexes are not reliable (e.g non-convex polygons)
		auto face_norm = ygl::normalize(ygl::cross(triangles_pos[1]-triangles_pos[0], triangles_pos[2]-triangles_pos[0]));

		// Outer walls
		auto shape = new ygl::shape();
		shape->pos = border;
		for (const auto& p : border) {
			shape->pos.push_back(p + face_norm*thickness);
		}
		int ps = border.size();
		for (int i = 0; i < border.size() - 1; i++) {
			shape->quads.push_back({ i,i + 1,ps + i + 1,ps + i });
		}
		shape->quads.push_back({ ps - 1,0,ps,2 * ps - 1 });

		// Hole walls
		for (const auto& hole : holes) {
			ps = shape->pos.size();
			int hs = hole.size();
			shape->pos.insert(shape->pos.end(), hole.begin(), hole.end()); // Floor points
			for (const auto& p : hole) shape->pos.push_back(p + face_norm*thickness); // Ceiling points
			for (int i = 0; i < hole.size() - 1; i++) {
				shape->quads.push_back({ ps + i,ps + i + 1,ps + hs + i + 1,ps + hs + i });
			}
			shape->quads.push_back({ps+hs-1,ps,ps+hs,ps+hs+hs-1});
		}

		// Floor and ceiling
		ps = shape->pos.size();
		ygl::vec3i psize3 = { ps,ps,ps };
		ygl::vec4i psize4 = { ps,ps,ps,ps };
		shape->pos.insert(shape->pos.end(), triangles_pos.begin(), triangles_pos.end()); // Floor
		for (const auto& p : triangles_pos) shape->pos.push_back(p + face_norm*thickness); // Ceiling
		for (const auto& t : triangles) {
			auto nt = t + psize3;
			shape->triangles.push_back({ nt.z, nt.y, nt.x }); // Original face, seen from the other side
			nt += {int(triangles_pos.size()), int(triangles_pos.size()), int(triangles_pos.size())};
			shape->triangles.push_back(nt); // New face
		}

		// Normals
		float ny = smooth_normals ? 1.f : 0.f;
		shape->norm = ygl::compute_normals({}, shape->triangles, shape->quads, shape->pos,false);
		for (int i = 0; i < border.size(); i++) { // Border
			auto& n = shape->norm[i];
			shape->norm[i] = ygl::normalize({ n.x, -ny, n.z });
		}
		for (int i = border.size(); i < 2*border.size(); i++) {
			auto& n = shape->norm[i];
			shape->norm[i] = ygl::normalize({ n.x, ny, n.z });
		}
		ps = 2 * border.size();
		for (const auto& hole : holes) { // Holes
			int hs = hole.size();
			for (int i = ps; i < ps + hs; i++) {
				shape->norm[i] = ygl::normalize({ shape->norm[i].x, -ny, shape->norm[i].z });
				shape->norm[i + hs] = ygl::normalize({ shape->norm[i + hs].x, ny, shape->norm[i + hs].z });
			}
			ps += 2*hs; // Top and bottom of the hole
		}
		for (int i = ps; i < ps + triangles_pos.size(); i++) { // Floor and ceiling
			// Naive all-points comparison to detect points on a border
			if (smooth_normals) {
				float eps = 0.001f;
				for (int j = 0; j < ps; j++) {
					if (ygl::length(shape->pos[i] - shape->pos[j]) <= eps) {
						// I don't know why it's {shape->norm[i],...} and not {shape->norm[j],...},
						// but it works :)
						shape->norm[i] = ygl::normalize({ shape->norm[i].x, -1.f, shape->norm[i].z });
						shape->norm[i + triangles_pos.size()] = ygl::normalize({ shape->norm[i].x, 1.f, shape->norm[i].z });
						break;
					}
				}
			}
			else {
				shape->norm[i] = { 0,-1,0 };
				shape->norm[i + triangles_pos.size()] = { 0,1,0 };
			}
		}
		/*merge_same_points(shape);
		set_shape_normals(shape);*/
		return shape;
	}

	ygl::shape* thicken_polygon(
		const std::vector<ygl::vec2f>& border,
		float thickness,
		const std::vector<std::vector<ygl::vec2f>>& holes = {},
		bool smooth_normals = true
	) {
		std::vector<std::vector<ygl::vec3f>> _holes;
		for (const auto& h : holes) _holes.push_back(to_3d(h));
		return thicken_polygon(to_3d(border), thickness, _holes, smooth_normals);
	}

	/**
	 * If origin_center is true, the model is centered on (0,0,0),
	 * else its center is (width,height,depth)/2
	 */
	std::tuple<std::vector<ygl::vec4i>, std::vector<ygl::vec3f>> 
		make_parallelepidedon(
			float width, float height, float depth,
			float x = 0.f, float y = 0.f, float z = 0.f,
			bool origin_center = false
		) {
		auto t = ygl::make_cube();
		auto w = width / 2.f,
			h = height / 2.f,
			d = depth / 2.f;
		for (auto& p : std::get<1>(t)) {
			p.x *= w;
			p.y *= h;
			p.z *= d;
			if (!origin_center) {
				p += {w, h, d};
			}
			p += {x, y, z};
		}
		return t;
	}

	/**
	 * Adds equilateral triangles on each side of the given polygon.
	 * 
	 * Arguments:
	 *     - polygon : the base polygon
	 *     - external : whether the fractal must be expanded outside or inside the polygon
	 *                  (assumes vertexes in counter-clockwise order)
	 *     - levels : number of recursion for the fractal expansion
	 */
	std::vector<ygl::vec2f> fractalize_triangle(
		const std::vector<ygl::vec2f>& polygon,
		bool outside = true,
		int levels = 1
	) {
		auto points = polygon;
		while (levels-- > 0) {
			std::vector<ygl::vec2f> newpos;
			for (int i = 0; i < polygon.size(); i++) {
				newpos.push_back(points[i]);
				ygl::vec2f mid1 = (points[i] * 2 / 3) + (points[(i + 1) % points.size()] * 1 / 3);
				ygl::vec2f mid2 = (points[i] * 1 / 3) + (points[(i + 1) % points.size()] * 2 / 3);
				ygl::vec2f mid =  (points[i] + points[(i + 1) % points.size()]) / 2;
				float side_normal_angle = 
					atan2f(mid1.x - mid2.x, mid1.y - mid2.y) + 
					((pi / 2) * (outside?+1.f:-1.f));
				// Height of an equilateral triangle is sqrt(3)/2 times the length of its side
				// sqrt(3)/2 roughly equals 0.866
				float triangle_height = ygl::length(mid2 - mid1)*0.866f;
				newpos.push_back(mid1);
				newpos.push_back(mid + ygl::vec2f{ sin(side_normal_angle)*triangle_height, cos(side_normal_angle)*triangle_height });
				newpos.push_back(mid2);
			}
			points = newpos;
		}
		return points;
	}

	/**
	 * Adds squares on each side of the given polygon.
	 * 
	 * Arguments:
	 *     - polygon : the base polygon
	 *     - external : whether the fractal must be expanded outside or inside the polygon
	 *                  (assumes vertexes in counter-clockwise order)
	 *     - levels : number of recursion for the fractal expansion
	 */
	std::vector<ygl::vec2f> fractalize_square(
		const std::vector<ygl::vec2f>& polygon,
		bool outside = true,
		int levels = 1
	) {
		auto points = polygon;
		while (levels-- > 0) {
			std::vector<ygl::vec2f> newpos;
			for (int i = 0; i < polygon.size(); i++) {
				newpos.push_back(points[i]);
				ygl::vec2f mid1 = (points[i] * 2 / 3) + (points[(i + 1) % points.size()] * 1 / 3);
				ygl::vec2f mid2 = (points[i] * 1 / 3) + (points[(i + 1) % points.size()] * 2 / 3);
				float side_normal_angle = 
					atan2f(mid1.x - mid2.x, mid1.y - mid2.y) + 
					((pi / 2) * (outside ? +1.f : -1.f));
				float quad_height = ygl::length(mid2 - mid1);
				auto add = ygl::vec2f{ sin(side_normal_angle)*quad_height, cos(side_normal_angle)*quad_height };
				newpos.push_back(mid1);
				newpos.push_back(mid1 + add);
				newpos.push_back(mid2 + add);
				newpos.push_back(mid2);
			}
			points = newpos;
		}
		return points;
	}
}

#endif // GEOM_UTILS_H