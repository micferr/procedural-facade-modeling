#include "geom_utils.h"
#include "tagged_shape.h"

#include "yocto/yocto_shape.h"

namespace yb {

std::set<int> tagged_shape::face_ids() const {
	std::set<int> ids;
	for (const auto& t : vertex_tags)
		ids.insert(t.face_id);
	return ids;
}

int tagged_shape::num_faces() const {
	return face_ids().size();
}

int tagged_shape::get_new_id() const {
	return get_new_ids(1)[0];
}

std::vector<int> tagged_shape::get_new_ids(int n) const {
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

bool tagged_shape::is_edge_vertex(int i, float max_zero, float min_one) const {
	auto tag = vertex_tags[i];
	return tag.face_coord.x <= max_zero || tag.face_coord.x >= min_one ||
		tag.face_coord.y <= max_zero || tag.face_coord.y >= min_one;
}

void tagged_shape::merge_same_points(
	float eps, float max_zero, float min_one
) {
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

tagged_shape make_cube(
	std::vector<ygl::vec4i>& triangles,
	std::vector<ygl::vec3f>& pos,
	float size,
	int start_face_id
) {
	tagged_shape c;
	ygl::make_cube(c.quads, c.pos, 0, size);
	to_triangle_mesh(&c);
	for (auto& p : c.pos) {
		// todo
		c.vertex_tags.push_back({ start_face_id, { 0,0 } });
	}
	return c;
}

void convert_tagged_to_faceted(tagged_shape* shp) {
	std::vector<ygl::vec3f> pos;
	std::vector<ygl::vec3i> triangles;
	std::vector<tagged_shape::tag> tags;
	for (auto i = 0; i < shp->triangles.size(); i++) {
		pos.push_back(shp->pos[shp->triangles[i].x]);
		pos.push_back(shp->pos[shp->triangles[i].y]);
		pos.push_back(shp->pos[shp->triangles[i].z]);
		triangles.push_back({ i * 3, i * 3 + 1, i * 3 + 2 });
		tags.push_back(shp->vertex_tags[shp->triangles[i].x]);
		tags.push_back(shp->vertex_tags[shp->triangles[i].y]);
		tags.push_back(shp->vertex_tags[shp->triangles[i].z]);
	}
	shp->pos = pos;
	shp->triangles = triangles;
	shp->vertex_tags = tags;
	ygl::compute_normals(shp->triangles, shp->pos, shp->norm);
}

}