#ifndef YB_GEOM_BOOL
#define YB_GEOM_BOOL

#include <vector>
#include <tuple>

#include "tagged_shape.h"

#include "yocto\yocto_math.h"

namespace yb {

enum class bool_operation {
	INTERSECTION,
	UNION,
	DIFFERENCE,
	XOR
};

std::tuple<std::vector<ygl::vec3f>, std::vector<ygl::vec3i>>
mesh_boolean_operation(
	const std::vector<ygl::vec3f>& pos_a,
	const std::vector<ygl::vec3i>& triangles_a,
	const std::vector<ygl::vec3f>& pos_b,
	const std::vector<ygl::vec3i>& triangles_b,
	bool_operation op
);

std::tuple<std::vector<ygl::vec3f>, std::vector<ygl::vec3i>, std::vector<tagged_shape::tag>>
mesh_boolean_operation(
	const tagged_shape& shp_a,
	const tagged_shape& shp_b,
	bool_operation op
);

}

#endif // YB_GEOM_BOOL