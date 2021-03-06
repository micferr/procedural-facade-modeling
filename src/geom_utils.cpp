#include <vector>

#include "poly2tri\poly2tri\poly2tri.h"
#include "clipper\clipper.hpp"
#include "geom_utils.h"
#include "yocto_utils.h"

namespace yb {
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

	ygl::vec2f to_2d(const ygl::vec3f& point, bool flip_y) {
		return { point.x, flip_y ? -point.z : point.z };
	}

	std::vector<ygl::vec2f> to_2d(
		const std::vector<ygl::vec3f>& points, bool flip_y
	) {
		std::vector<ygl::vec2f> res;
		for (const auto& p : points) {
			res.push_back(to_2d(p, flip_y));
		}
		return res;
	}

	ygl::vec3f to_3d(const ygl::vec2f& point, float y, bool flip_z) {
		return { point.x, y, flip_z ? -point.y : point.y };
	}

	std::vector<ygl::vec3f> to_3d(
		const std::vector<ygl::vec2f>& points,
		float y,
		bool flip_z
	) {
		std::vector<ygl::vec3f> res;
		for (const auto& p : points) {
			res.push_back(to_3d(p, y, flip_z));
		}
		return res;
	}

	void swap_yz(std::vector<ygl::vec3f>& points) {
		for (auto& p : points) p = { p.x, p.z, p.y };
	}

	std::vector<ygl::vec2f> make_regular_polygon(
		int num_sides,
		float radius,
		float base_angle
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

	std::vector<ygl::vec3f> make_regular_polygonXZ(
		int num_sides,
		float radius,
		float base_angle
	) {
		return to_3d(make_regular_polygon(num_sides, radius, base_angle));
	}

	std::vector<ygl::vec2f> make_quad(float side_length) {
		float s = side_length / 2.f;
		return { { s,s },{ -s,s },{ -s,-s },{ s,-s } };
	}

	std::vector<ygl::vec3f> make_quadXZ(float side_length) {
		auto quad = to_3d(make_quad(side_length));
		return quad;
	}

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
				last_pos + ygl::vec2f{ cos(alpha), sin(alpha) }*segment_length
			);
		}
		return points;
	}

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

	std::tuple<std::vector<ygl::vec4i>, std::vector<ygl::vec3f>> make_wide_line(
		const std::vector<ygl::vec2f>& points,
		float width,
		bool lengthen_ends
	) {
		auto half_width = width / 2.f;
		auto _points = points; // Work-around to keep code cleaner :)
		std::vector<ygl::vec2f> vertexes;
		if (lengthen_ends) {
			_points[0] -= ygl::normalize(points[1] - points[0]) * half_width;
			_points.back() += ygl::normalize(points[points.size() - 1] - points[points.size() - 2]) * half_width;
		}
		_points.push_back(2 * _points[_points.size() - 1] - _points[_points.size() - 2]); //p.back() + to(p[p.size()-2], p.back())
		_points.insert(_points.begin(), _points[0] - (_points[1] - _points[0]));
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
		for (int i = 0; i < vertexes.size() - 4; i += 2) {
			quads.push_back({ i,i + 1,i + 3,i + 2 });
		}
		return { quads, to_3d(vertexes) };
	}

	void clean_collinear_vertexes(std::vector<ygl::vec2f>& points, float alpha_eps) {
		points.push_back(points[0]); // Easier to loop on all sides
		for (int i = 1; i < points.size() - 1; i++) {
			auto segment1 = points[i] - points[i - 1];
			auto segment2 = points[i + 1] - points[i];
			auto segment_tot = points[i + 1] - points[i - 1];
			float alpha1 = atan2f(segment1.y, segment1.x);
			float alpha2 = atan2f(segment2.y, segment2.x);
			float alpha_tot = atan2f(segment_tot.y, segment_tot.x);
			if (fabs(alpha_tot - alpha2) <= alpha_eps) {
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
		bool lengthen_ends
	) {
		auto pos = std::get<1>(make_wide_line(points, width, lengthen_ends));
		std::vector<ygl::vec3f> res;
		// Go forward along the right side of the line...
		for (int i = 0; i < pos.size(); i += 2) res.push_back(pos[i]);
		// ...then back along the left side
		for (int i = pos.size() - 1; i >= 0; i -= 2) res.push_back(pos[i]);
		return res;
	}

	std::vector<std::vector<ygl::vec2f>> offset_polygon(
		const std::vector<ygl::vec2f>& polygon,
		float delta,
		unsigned _scale_factor
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
				poly.push_back(ygl::vec2f{ float(p.X), float(p.Y) } / float(_scale_factor));
			}
			newpolys.push_back(poly);
		}
		return newpolys;
	}

	std::vector<ygl::vec2f> expand_polygon(
		const std::vector<ygl::vec2f>& polygon,
		float delta,
		unsigned _scale_factor
	) {
		if (delta < 0.f) throw std::runtime_error("Invalid arguments");
		return offset_polygon(polygon, delta, _scale_factor)[0];
	}

	std::tuple<std::vector<ygl::vec3i>, std::vector<ygl::vec2f>>
		triangulate(
			const std::vector<ygl::vec2f>& border,
			const std::vector<std::vector<ygl::vec2f>>& holes
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

	std::tuple<std::vector<ygl::vec3i>, std::vector<ygl::vec3f>>
		triangulate(
			const std::vector<ygl::vec3f>& border,
			const std::vector<std::vector<ygl::vec3f>>& holes
		) {
		std::vector<std::vector<ygl::vec2f>> _holes;
		for (const auto& h : holes) _holes.push_back(to_2d(h));
		auto tt = triangulate(to_2d(border), _holes);

		// Find plane equation
		auto points = to_3d(std::get<1>(tt));
		// Already assumed non-collinear for triangulation
		auto p1 = border[0], p2 = border[1], p3 = border[2];
		auto p12 = p2 - p1, p13 = p3 - p1;
		auto pn = ygl::cross(p12, p13);
		auto d = ygl::dot(p1, pn);
		if (pn.y != 0.f) {
			for (auto& p : points) {
				p.y = (d - pn.x*p.x - pn.z*p.z) / pn.y;
			}
		}

		return { std::get<0>(tt), points };
	}

	std::tuple<std::vector<ygl::vec3i>, std::vector<ygl::vec3f>>
		triangulate_opposite(
			const std::vector<ygl::vec3f>& border,
			const std::vector<std::vector<ygl::vec3f>>& holes
		) {
		auto tt = triangulate(border, holes);
		for (auto& t : std::get<0>(tt)) t = { t.z,t.y,t.x };
		return tt;
	}

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

	void center_points(
		std::vector<ygl::vec3f>& points,
		bool x, bool y, bool z,
		bool weighted
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

	void rotate(ygl::vec2f& point, float angle) {
		auto new_angle = get_angle(point) + angle;
		auto length = ygl::length(point);
		point = { cosf(new_angle)*length, sinf(new_angle)*length };
	}

	void rotate(std::vector<ygl::vec2f>& points, float angle) {
		for (auto& p : points) rotate(p, angle);
	}

	void rotate_y(ygl::vec3f& point, float angle) {
		auto p2d = to_2d(point);
		rotate(p2d, angle);
		point = to_3d(p2d, point.y);
	}

	void rotate_y(std::vector<ygl::vec3f>& points, float angle) {
		for (auto& p : points) rotate_y(p, angle);
	}

	void for_sides(
		const std::vector<ygl::vec2f>& poly,
		const std::function<void(const ygl::vec2f&, const ygl::vec2f&)>& func
	) {
		if (poly.size() < 3) return;
		for (int i = 0; i < poly.size() - 1; i++) {
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

	void merge_same_points(ygl::shape* shp, float eps) {
		for (int i = 0; i < shp->pos.size(); i++) {
			for (int j = i + 1; j < shp->pos.size(); j++) {
				if (
					ygl::length(shp->pos[i] - shp->pos[j]) < eps &&
					(shp->color.size() == 0 || ygl::length(shp->color[i] - shp->color[j]) < eps)
					) {
					if (shp->triangles.size() > 0) {
						for (auto& t : shp->triangles) {
							if (t.x == j) t.x = i;
							if (t.y == j) t.y = i;
							if (t.z == j) t.z = i;
						}
					}
					else if (shp->quads.size() > 0) {
						for (auto& q : shp->quads) {
							if (q.x == j) q.x = i;
							if (q.y == j) q.y = i;
							if (q.z == j) q.z = i;
							if (q.w == j) q.w = i;
						}
					}
					else {
						// Putting lines and points together for now
						for (auto& l : shp->lines) {
							if (l.x == j) l.x = i;
							if (l.y == j) l.y = i;
						}
						for (auto& p : shp->points) if (p == j) p = i;
					}
				}
			}
		}
		if (shp->triangles.size() > 0) {
			ygl::compute_normals(shp->triangles, shp->pos, shp->norm);
		}
		else if (shp->quads.size() > 0) {
			ygl::compute_normals(shp->quads, shp->pos, shp->norm);
		}
	}

	ygl::shape* thicken_polygon(
		const std::vector<ygl::vec3f>& border,
		float thickness,
		const std::vector<std::vector<ygl::vec3f>>& holes,
		bool smooth_normals
	) {
		// TO BE FIXED

		/*// Triangulate floor surface
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
		set_shape_normals(shape);
		return shape;*/
		return {};
	}

	ygl::shape* thicken_polygon(
		const std::vector<ygl::vec2f>& border,
		float thickness,
		const std::vector<std::vector<ygl::vec2f>>& holes,
		bool smooth_normals
	) {
		std::vector<std::vector<ygl::vec3f>> _holes;
		for (const auto& h : holes) _holes.push_back(to_3d(h));
		return thicken_polygon(to_3d(border), thickness, _holes, smooth_normals);
	}

	std::tuple<std::vector<ygl::vec4i>, std::vector<ygl::vec3f>>
		make_parallelepidedon(
			float width, float height, float depth,
			float x, float y, float z,
			bool origin_center
		) {
		ygl::shape t;
		ygl::make_cube(t.quads, t.pos, 0, 1.f);
		auto w = width / 2.f,
			h = height / 2.f,
			d = depth / 2.f;
		for (auto& p : t.pos) {
			p.x *= w;
			p.y *= h;
			p.z *= d;
			if (!origin_center) {
				p += {w, h, d};
			}
			p += {x, y, z};
		}
		return std::tie(t.quads, t.pos);
	}

	void to_triangle_mesh(ygl::shape* shp) {
		for (const auto& q : shp->quads) {
			shp->triangles.push_back({ q.x,q.y,q.z });
			shp->triangles.push_back({ q.z,q.w,q.x });
		}
		ygl::compute_normals(shp->triangles, shp->pos, shp->norm);
		shp->quads.clear();
	}

	void convert_to_faceted(ygl::shape* shp) {
		std::vector<ygl::vec3f> pos;
		std::vector<ygl::vec3i> triangles;
		for (auto i = 0; i < shp->triangles.size(); i++) {
			pos.push_back(shp->pos[shp->triangles[i].x]);
			pos.push_back(shp->pos[shp->triangles[i].y]);
			pos.push_back(shp->pos[shp->triangles[i].z]);
			triangles.push_back({ i * 3, i * 3 + 1, i * 3 + 2 });
		}
		shp->pos = pos;
		shp->triangles = triangles;
		ygl::compute_normals(shp->triangles, shp->pos, shp->norm);
	}
}