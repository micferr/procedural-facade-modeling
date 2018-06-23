#ifndef YOCTO_UTILS_H
#define YOCTO_UTILS_H

#include "yocto\yocto_image.h"
#include "yocto\yocto_math.h"
#include "yocto\yocto_obj.h"
#include "yocto\yocto_scene.h"
#include "yocto\yocto_shape.h"
#include "yocto\yocto_utils.h"

namespace ygl {
	inline vec3i operator+(const vec3i& a) { return a; }
	inline vec3i operator-(const vec3i& a) { return { -a.x, -a.y, -a.z }; }
	inline vec3i operator+(const vec3i& a, const vec3i& b) {
		return { a.x + b.x, a.y + b.y, a.z + b.z };
	}
	inline vec3i operator-(const vec3i& a, const vec3i& b) {
		return { a.x - b.x, a.y - b.y, a.z - b.z };
	}
	inline vec3i operator*(const vec3i& a, const vec3i& b) {
		return { a.x * b.x, a.y * b.y, a.z * b.z };
	}
	inline vec3i operator*(const vec3i& a, float b) {
		return { int(a.x * b), int(a.y * b), int(a.z * b) };
	}
	inline vec3i operator*(float a, const vec3i& b) {
		return { int(a * b.x), int(a * b.y), int(a * b.z) };
	}
	inline vec3i operator/(const vec3i& a, const vec3i& b) {
		return { a.x / b.x, a.y / b.y, a.z / b.z };
	}
	inline vec3i operator/(const vec3i& a, float b) {
		return { int(a.x / b), int(a.y / b), int(a.z / b) };
	}
	inline vec3i operator/(float a, const vec3i& b) {
		return { int(a / b.x), int(a / b.y), int(a / b.z) };
	}
	inline vec3i& operator+=(vec3i& a, const vec3i& b) { return a = a + b; }
	inline vec3i& operator-=(vec3i& a, const vec3i& b) { return a = a - b; }
	inline vec3i& operator*=(vec3i& a, const vec3i& b) { return a = a * b; }
	inline vec3i& operator*=(vec3i& a, float b) { return a = a * b; }
	inline vec3i& operator/=(vec3i& a, const vec3i& b) { return a = a / b; }
	inline vec3i& operator/=(vec3i& a, float b) { return a = a / b; }
}

namespace yb {

	ygl::material* make_material(const std::string& name, const ygl::vec3f& kd,
		ygl::texture* kd_txt = nullptr, const ygl::vec3f& ks = { 0.2f, 0.2f, 0.2f },
		float rs = 0.01f
	);

	void add_light(ygl::scene* scn, const ygl::vec3f& pos, const ygl::vec3f& ke, const std::string& name);

	void set_shape_color(ygl::shape* shp, const ygl::vec4f& color);

	void set_shape_color(ygl::shape* shp, const ygl::vec3f& color);

	void set_shape_normals(ygl::shape* shp);

	/**
	 * Returns the angle between the vector and the x axis
	 */
	float get_angle(const ygl::vec2f& v);

	void translate(ygl::instance* inst, const ygl::vec3f& d);

	void print_shape(const ygl::shape* shp);
	
	/**
	 * Adds an instance to a scene and, if needed, also add the instance's
	 * shape and material
	 *
	 * The comparison with elements already in the scene is done by checking
	 * for pointer equality rather than names for performance
	 */
	void add_to_scene(ygl::scene* scn, ygl::instance* inst);

	ygl::vec3f rand_color3f(ygl::rng_state& rng);

	ygl::vec4f rand_color4f(ygl::rng_state& rng, float a = 1.f);

	ygl::instance* make_instance(const std::string& name, ygl::shape* shp);

	ygl::instance* make_instance(
		const std::string& name,
		ygl::shape* shp,
		ygl::material* mat
	);
}

#endif // YOCTO_UTILS_H
