#ifndef WRAP_IGL_H
#define WRAP_IGL_H

#include <Eigen\Core>
#include "geom_bool.h"

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
);

}

#endif // WRAP_IGL_H