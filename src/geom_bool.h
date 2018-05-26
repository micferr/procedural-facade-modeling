#ifndef YB_GEOM_BOOL
#define YB_GEOM_BOOL

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Nef_polyhedron_3.h>

#include "yocto_utils.h"

namespace yb {

typedef CGAL::Simple_cartesian<double> Kernel;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef CGAL::Nef_polyhedron_3<Kernel> Nef_polyhedron;

enum class bool_operation {
	INTERSECTION,
	UNION,
	DIFFERENCE,
	XOR
};

std::tuple<std::vector<ygl::vec3f>, std::vector<ygl::vec3i>>
geom_bool(
	const std::vector<ygl::vec3f>& pos_a,
	const std::vector<ygl::vec3i>& triangles_a,
	const std::vector<ygl::vec3f>& pos_b,
	const std::vector<ygl::vec3i>& triangles_b,
	bool_operation bool_op
) {
	return { {},{} };
}

}

#endif // YB_GEOM_BOOL