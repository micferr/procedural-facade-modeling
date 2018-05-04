#ifndef YOCTO_UTILS_H
#define YOCTO_UTILS_H

#include "yocto\yocto_gl.h"

namespace yb {

	ygl::material* make_material(const std::string& name, const ygl::vec3f& kd,
		ygl::texture* kd_txt = nullptr, const ygl::vec3f& ks = { 0.2f, 0.2f, 0.2f },
		float rs = 0.01f) {
		ygl::material* m = new ygl::material();
		m->name = name;
		m->kd = kd;
		m->kd_txt.txt = kd_txt;
		m->ks = ks;
		m->rs = rs;
		return m;
	}

	void add_light(ygl::scene* scn, const ygl::vec3f& pos, const ygl::vec3f& ke, const std::string& name) {
		ygl::shape* lshp = new ygl::shape{ name + "_shape" };
		lshp->pos.push_back(pos);
		lshp->points.push_back(0);
		lshp->radius.push_back(0.001f);
		lshp->norm.push_back({ 0,0,1 });
		lshp->color = { {1,1,1,1} };
		ygl::instance* linst = new ygl::instance();
		linst->frame = ygl::identity_frame3f;
		linst->name = name + "_instance";
		linst->shp = lshp;
		auto lmat = new ygl::material();
		lmat->name = name+"_material";
		lmat->ke = ke;
		lmat->kd = ygl::zero3f;
		lshp->mat = lmat;
		scn->materials.push_back(lmat);
		scn->shapes.push_back(lshp);
		scn->instances.push_back(linst);
	}

	void set_shape_color(ygl::shape* shp, const ygl::vec4f& color) {
		shp->color = std::vector<ygl::vec4f>(shp->pos.size(), color);
	}

	void set_shape_color(ygl::shape* shp, const ygl::vec3f& color) {
		set_shape_color(shp, { color.x, color.y, color.z, 1.f });
	}

	void set_shape_normals(ygl::shape* shp) {
		shp->norm = ygl::compute_normals(
			shp->lines, shp->triangles, shp->quads, shp->pos
		);
	}

	/**
	 * Returns the angle between the vector and the x axis
	 */
	float get_angle(const ygl::vec2f& v) {
		return atan2f(v.y, v.x);
	}

	void translate(ygl::instance* inst, const ygl::vec3f& d) {
		inst->frame.o = inst->frame.o + d;
	}

	void print_shape(const ygl::shape* shp) {
		printf("Vertexes: (pos and norm)\n");
		for (int i = 0; i < shp->pos.size(); i++) {
			ygl::println("{} | {} - {}", i, shp->pos[i], shp->norm[i]);
		}
		printf("Faces:\n");
		for (const auto& p : shp->points) ygl::println("{}", p);
		for (const auto& t : shp->triangles) ygl::println("{}", t);
		for (const auto& q : shp->quads) ygl::println("{}", q);
	}
	
	/**
	 * Adds an instance to a scene and, if needed, also add the instance's
	 * shape and material
	 *
	 * The comparison with elements already in the scene is done by checking
	 * for pointer equality rather than names for performance
	 */
	void add_to_scene(ygl::scene* scn, ygl::instance* inst) {
		scn->instances += inst;
		
		// Shape
		bool found = false;
		for (auto s : scn->shapes) if (s == inst->shp) {
			found = true;
			break;
		}
		if (!found) scn->shapes += inst->shp;

		// Material
		found = false;
		for (auto m : scn->materials) if (m == inst->shp->mat) {
			found = true;
			break;
		}
		if (!found) scn->materials += inst->shp->mat;
	}

	ygl::vec3f rand_color3f(ygl::rng_pcg32& rng) {
		return { ygl::next_rand1f(rng), ygl::next_rand1f(rng), ygl::next_rand1f(rng) };
	}

	ygl::vec4f rand_color4f(ygl::rng_pcg32& rng, float a = 1.f) {
		return { ygl::next_rand1f(rng), ygl::next_rand1f(rng), ygl::next_rand1f(rng), a };
	}

	ygl::instance* make_instance(const std::string& name, ygl::shape* shp) {
		auto inst = new ygl::instance();
		inst->name = name;
		inst->shp = shp;
		return inst;
	}

	ygl::instance* make_instance(
		const std::string& name,
		ygl::shape* shp,
		ygl::material* mat
	) {
		auto inst = new ygl::instance();
		inst->name = name + "_inst";
		inst->shp = shp;
		inst->shp->name = name + "_shp";
		inst->shp->mat = mat;
		if (mat->name.size() == 0) { // mat->name == ""
			mat->name = name + "_mat";
		}
		return inst;
	}
}

#endif // YOCTO_UTILS_H
