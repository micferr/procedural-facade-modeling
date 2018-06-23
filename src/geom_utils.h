#ifndef GEOM_UTILS_H
#define GEOM_UTILS_H

#include "yocto\yocto_math.h"
#include "yocto\yocto_scene.h"
#include "yocto\yocto_shape.h"

namespace yb {
	const float pi = 3.14159265359f;

	ygl::vec2f centroid(const std::vector<ygl::vec2f>& points);
	ygl::vec3f centroid(const std::vector<ygl::vec3f>& points);

	/**
	* Discards the y component of 3d points
	*/
	ygl::vec2f to_2d(const ygl::vec3f& point, bool flip_y = true);
	std::vector<ygl::vec2f> to_2d(
		const std::vector<ygl::vec3f>& points, bool flip_y = true
	);

	/**
	 * Transforms 2D points (x,z) to 3D points (x,y,-z), with y in input.
	 * z is flipped by default to preserve clockwiseness
	 */
	ygl::vec3f to_3d(const ygl::vec2f& point, float y = 0.f, bool flip_z = true);
	std::vector<ygl::vec3f> to_3d(
		const std::vector<ygl::vec2f>& points,
		float y = 0.f,
		bool flip_z = true
	);

	/**
	 * Swaps the Y and Z components of the input points
	 */
	void swap_yz(std::vector<ygl::vec3f>& points);

	/**
	 * Return the points of a 2D polygon.
	 * The first point is set at (radius,0) if the polygon is not aligned,
	 * else it's at radius*(cos(a),sin(a)), a = 2*pi/(2*num_sides)
	 */
	std::vector<ygl::vec2f> make_regular_polygon(
		int num_sides,
		float radius = 1.f,
		float base_angle = 0.f
	);

	/**
	 * Returns a 2D regular polygon in 3D space, with a fixed
	 * value of 0 for the y component of the points.
	 * Face normal is (0,1,0)
	 */
	std::vector<ygl::vec3f> make_regular_polygonXZ(
		int num_sides,
		float radius = 1.f,
		float base_angle = 0.f
	);

	std::vector<ygl::vec2f> make_quad(float side_length = 1.f);

	std::vector<ygl::vec3f> make_quadXZ(float side_length = 1.f);

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
	);

	/**
	 * Overloaded version where the starting angle is also
	 * randomly generated
	 */
	std::vector<ygl::vec2f> make_segmented_line(
		const ygl::vec2f& start,
		unsigned steps,
		const std::function<float()>& alpha_delta_per_step,
		const std::function<float()>& segment_length_per_step
	);

	/**
	 * Returns a 2d surface obtained from widening the input line.
	 * It is currently assumed that the resulting polygon will be simple.
	 */
	std::tuple<std::vector<ygl::vec4i>, std::vector<ygl::vec3f>> make_wide_line(
		const std::vector<ygl::vec2f>& points,
		float width,
		bool lengthen_ends = true
	);

	/**
	 * Merges collinear consecutive sides in a polygon.
	 *
	 * It is assumed that the polygon is well-formed (e.g. the vertexes are not all
	 * collinear)
	 */
	void clean_collinear_vertexes(std::vector<ygl::vec2f>& points, float alpha_eps = 0.001f);

	std::vector<ygl::vec3f> make_wide_line_border(
		const std::vector<ygl::vec2f>& points,
		float width,
		bool lengthen_ends = true
	);

	/**
	 * Offsets a polygon's vertexes to uniformly expand/shrink it.
	 */
	std::vector<std::vector<ygl::vec2f>> offset_polygon(
		const std::vector<ygl::vec2f>& polygon,
		float delta,
		unsigned _scale_factor = 100000
	);

	/**
	 * Simplified version of offset_polygon, it can only expand (no shrinking),
	 * so we're sure to only have one output polygon
	 */
	std::vector<ygl::vec2f> expand_polygon(
		const std::vector<ygl::vec2f>& polygon,
		float delta,
		unsigned _scale_factor = 10000
	);

	/**
	* Triangulates an arbitrary shape.
	* Holes must be given in clockwise order
	*/
	std::tuple<std::vector<ygl::vec3i>, std::vector<ygl::vec2f>>
		triangulate(
			const std::vector<ygl::vec2f>& border,
			const std::vector<std::vector<ygl::vec2f>>& holes = {}
	);

	std::tuple<std::vector<ygl::vec3i>, std::vector<ygl::vec3f>>
		triangulate(
			const std::vector<ygl::vec3f>& border,
			const std::vector<std::vector<ygl::vec3f>>& holes = {}
	);

	/**
	 * Triangulates the shape and inverts the triangles' orientation
	 */
	std::tuple<std::vector<ygl::vec3i>, std::vector<ygl::vec3f>>
		triangulate_opposite(
			const std::vector<ygl::vec3f>& border,
			const std::vector<std::vector<ygl::vec3f>>& holes = {}
	);

	/**
	 * Moves all points by the specified value
	 */
	void displace(std::vector<ygl::vec2f>& points, const ygl::vec2f& disp);
	void displace(std::vector<ygl::vec3f>& points, const ygl::vec3f& disp);

	template<typename T>
	void scale(std::vector<T>& points, float scale);

	/**
	 * Moves the points so that they're centered on the specified axes
	 */
	void center_points(
		std::vector<ygl::vec3f>& points,
		bool x = true, bool y = true, bool z = true,
		bool weighted = true
	);

	/**
	* Rotates the point by 'angle' radiants, counter-clockwise relative to (0,0)
	*/
	void rotate(ygl::vec2f& point, float angle);
	void rotate(std::vector<ygl::vec2f>& points, float angle);

	/**
	 * Rotates the point by 'angle' radiants around the y axis
	 * counter-clockwise
	 */
	void rotate_y(ygl::vec3f& point, float angle);
	void rotate_y(std::vector<ygl::vec3f>& points, float angle);

	/**
	 * Executes func on all sides of the polygon (where a side is
	 * two consecutive vertexes)
	 */
	void for_sides(
		const std::vector<ygl::vec2f>& poly,
		const std::function<void(const ygl::vec2f&, const ygl::vec2f&)>& func
	);

	ygl::vec3f get_size(const ygl::shape* shp);

	/**
	* Merges duplicates points in a shape
	*
	* TODO: Remove duplicates from shp->pos
	*/
	void merge_same_points(ygl::shape* shp, float eps = 0.0001f);

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
	);

	ygl::shape* thicken_polygon(
		const std::vector<ygl::vec2f>& border,
		float thickness,
		const std::vector<std::vector<ygl::vec2f>>& holes = {},
		bool smooth_normals = true
	);

	/**
	 * If origin_center is true, the model is centered on (0,0,0),
	 * else its center is (width,height,depth)/2
	 */
	std::tuple<std::vector<ygl::vec4i>, std::vector<ygl::vec3f>>
		make_parallelepidedon(
			float width, float height, float depth,
			float x = 0.f, float y = 0.f, float z = 0.f,
			bool origin_center = false
		);

	/**
	 * Converts all quads of the shape into triangles.
	 * Note that, since Yocto/GL's more recent versions do not support
	 * mixed triangles+quads meshes, the quads vector is emptied after 
	 * this operation.
	 *
	 * Points and lines are not modified but should be kept empty anyway.
	 */
	void to_triangle_mesh(ygl::shape* shp);

	// Triangle mesh only
	void convert_to_faceted(ygl::shape* shp);
}

#endif // GEOM_UTILS_H