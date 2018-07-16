#include <Eigen/Core>
#include <Eigen/QR>

#include "rhea/simplex_solver.hpp"

#include "geom_bool.h"
#include "tagged_shape.h"

#include "wrap_igl.h"

namespace yb {

std::tuple<std::vector<ygl::vec3f>, std::vector<ygl::vec3i>, Eigen::VectorXi>
_mesh_boolean_operation(
	const std::vector<ygl::vec3f>& pos_a,
	const std::vector<ygl::vec3i>& triangles_a,
	const std::vector<ygl::vec3f>& pos_b,
	const std::vector<ygl::vec3i>& triangles_b,
	bool_operation op
) {
	Eigen::MatrixXd e_pa(pos_a.size(), 3), e_pb(pos_b.size(), 3);
	Eigen::MatrixXi e_ta(triangles_a.size(), 3), e_tb(triangles_b.size(), 3);

	for (auto i = 0; i < pos_a.size(); i++) {
		e_pa(i, 0) = pos_a[i].x;
		e_pa(i, 1) = pos_a[i].y;
		e_pa(i, 2) = pos_a[i].z;
	}
	for (auto i = 0; i < pos_b.size(); i++) {
		e_pb(i, 0) = pos_b[i].x;
		e_pb(i, 1) = pos_b[i].y;
		e_pb(i, 2) = pos_b[i].z;
	}
	for (auto i = 0; i < triangles_a.size(); i++) {
		e_ta(i, 0) = triangles_a[i].x;
		e_ta(i, 1) = triangles_a[i].y;
		e_ta(i, 2) = triangles_a[i].z;
	}
	for (auto i = 0; i < triangles_b.size(); i++) {
		e_tb(i, 0) = triangles_b[i].x;
		e_tb(i, 1) = triangles_b[i].y;
		e_tb(i, 2) = triangles_b[i].z;
	}

	Eigen::MatrixXd e_pr;
	Eigen::MatrixXi e_tr;
	Eigen::VectorXi J;
	w_igl_copyleft_cgal_mesh_boolean(e_pa, e_ta, e_pb, e_tb, op, e_pr, e_tr, J);
	
	std::vector<ygl::vec3f> pos_r;
	std::vector<ygl::vec3i> triangles_r;
	for (int i = 0; i < e_pr.rows(); i++) {
		pos_r.push_back({ float(e_pr(i, 0)), float(e_pr(i, 1)), float(e_pr(i, 2)) });
	}
	for (int i = 0; i < e_tr.rows(); i++) {
		triangles_r.push_back({ e_tr(i,0), e_tr(i,1), e_tr(i,2) });
	}
	return { pos_r, triangles_r, J };
}

std::tuple<std::vector<ygl::vec3f>, std::vector<ygl::vec3i>>
mesh_boolean_operation(
	const std::vector<ygl::vec3f>& pos_a,
	const std::vector<ygl::vec3i>& triangles_a,
	const std::vector<ygl::vec3f>& pos_b,
	const std::vector<ygl::vec3i>& triangles_b,
	bool_operation op
) {
	auto res = _mesh_boolean_operation(pos_a, triangles_a, pos_b, triangles_b, op);
	return { std::get<0>(res), std::get<1>(res) };
}

std::tuple<std::vector<ygl::vec3f>, std::vector<ygl::vec3i>, std::vector<tagged_shape::tag>>
mesh_boolean_operation(
	const tagged_shape& shp_a,
	const tagged_shape& shp_b,
	bool_operation op
) {
	auto ptj = _mesh_boolean_operation(
		shp_a.pos, shp_a.triangles, shp_b.pos, shp_b.triangles, op
	);
	const auto& pos = std::get<0>(ptj);
	const auto& triangles = std::get<1>(ptj);
	const auto& J = std::get<2>(ptj);

	std::vector<ygl::vec3f> rpos;
	std::vector<ygl::vec3i> rtriangles;
	std::vector<tagged_shape::tag> tags;
	for (int i = 0; i < triangles.size(); i++) {
		auto orig_shp = &shp_a;
		auto orig_triangle_index = J(i);
		if (J(i) >= shp_a.triangles.size()) {
			orig_shp = &shp_b;
			orig_triangle_index -= shp_a.triangles.size();
		}

		rpos.push_back(pos[triangles[i].x]);
		rpos.push_back(pos[triangles[i].y]);
		rpos.push_back(pos[triangles[i].z]);

		rtriangles.push_back({ i * 3,i * 3 + 1,i * 3 + 2 });

		const auto& orig_triangle = orig_shp->triangles[orig_triangle_index];
		auto tag1 = orig_shp->vertex_tags[orig_triangle.x];
		auto tag2 = orig_shp->vertex_tags[orig_triangle.y];
		auto tag3 = orig_shp->vertex_tags[orig_triangle.z];

		// Pos of the spawning triangle
		auto jpos1 = orig_shp->pos[orig_triangle.x];
		auto jpos2 = orig_shp->pos[orig_triangle.y];
		auto jpos3 = orig_shp->pos[orig_triangle.z];

		// todo:
		// use tagx and jposx to find bary coords of new rpos;
		// --> tags.face_coords
		ygl::vec2f face_coords[3] = { {0,0},{0,0},{0,0} };

		// Take the majority of face ids if they're not all equals
		auto face_tag = tag1.face_id == tag2.face_id || tag1.face_id == tag3.face_id ?
			tag1.face_id : tag2.face_id;
		for (int j = 0; j < 3; j++) {
			tags.push_back({ face_tag, face_coords[j] });
		}
	}

	return std::tie(rpos, rtriangles, tags);
}

}