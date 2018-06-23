#include <igl/copyleft/cgal/mesh_boolean.h>

#include "wrap_igl.h"

namespace yb {

	bool w_igl_copyleft_cgal_mesh_boolean(
		const Eigen::MatrixXd& pa,
		const Eigen::MatrixXi& ta,
		const Eigen::MatrixXd& pb,
		const Eigen::MatrixXi& tb,
		bool_operation op,
		Eigen::MatrixXd& pr,
		Eigen::MatrixXi& tr,
		Eigen::VectorXi& J
	) {
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
		return igl::copyleft::cgal::mesh_boolean(pa, ta, pb, tb, bool_op, pr, tr, J);
	}

}