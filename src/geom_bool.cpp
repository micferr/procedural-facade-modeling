#include "geom_bool.h"

#include <igl/copyleft/cgal/mesh_boolean.h>

namespace yb {

std::tuple<std::vector<ygl::vec3f>, std::vector<ygl::vec3i>>
mesh_boolean_operation(
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
	igl::MeshBooleanType bool_op;
	switch (op) {
	case bool_operation::INTERSECTION:
		bool_op = igl::MESH_BOOLEAN_TYPE_INTERSECT;
		break;
	case bool_operation::DIFFERENCE:
		bool_op = igl::MESH_BOOLEAN_TYPE_MINUS;
		break;
	case bool_operation::UNION:
		bool_op = igl::MESH_BOOLEAN_TYPE_UNION;
		break;
	case bool_operation::XOR:
		bool_op = igl::MESH_BOOLEAN_TYPE_XOR;
		break;
	default: break;
	}
	igl::copyleft::cgal::mesh_boolean(e_pa, e_ta, e_pb, e_tb, bool_op, e_pr, e_tr, J);
	
	std::vector<ygl::vec3f> pos_r;
	std::vector<ygl::vec3i> triangles_r;
	for (int i = 0; i < e_pr.rows(); i++) {
		pos_r.push_back({ float(e_pr(i, 0)), float(e_pr(i, 1)), float(e_pr(i, 2)) });
	}
	for (int i = 0; i < e_tr.rows(); i++) {
		triangles_r.push_back({ e_tr(i,0), e_tr(i,1), e_tr(i,2) });
	}
	return { pos_r, triangles_r };
}

}