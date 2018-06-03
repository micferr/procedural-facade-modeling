#ifndef TAGGED_SHAPE_H
#define TAGGED_SHAPE_H

#include "yocto\yocto_math.h"
#include "yocto\yocto_scene.h"

#include <set>
#include <vector>

namespace yb {

struct tagged_shape : public ygl::shape {
	struct tag {
		int face_id; // Which face the vertex belongs to
		ygl::vec2f face_coord; // Usually a [0;1]x[0;1] coordinate relative to the face
	};

	std::vector<tag> vertex_tags;
	std::set<int> face_ids();

	// Returns the number of unique assigned ids
	int num_faces();

	// Returns an unused id for a new face
	int get_new_id();

	// Returns n unused ids for new faces
	std::vector<int> get_new_ids(int n);

	// Checks whether a vertex lies on the edge of a face
	bool is_edge_vertex(int i, float max_zero = 0.0001f, float min_one = 0.9999f);

	// Customized merge_same_points (cfr. the one in geom_utils.h) for tagged shapes
	void merge_same_points(
		float eps = 0.0001f, float max_zero = 0.0001f, float min_one = 0.9999f
	);
};

}

#endif // TAGGED_SHAPE_H