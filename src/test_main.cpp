#include <cstdio>
#include <iostream>
#include <string>
#include <ctime>

#include "geom_utils.h"
#include "grammar.h"
#include "prob_utils.h"
#include "yocto/yocto_gl.h"
#include "yocto_utils.h"
#include "building_utils.h"

int main(int argc, char** argv) {
	srand(time(NULL));
	ygl::scene* scn = new ygl::scene();
	
	ygl::rng_pcg32 rng = ygl::init_rng(rand());

	//Add floor
	auto floor_mat = yb::make_material("floor_mat", { 0.3f,0.3f,0.1f }, nullptr, { 0,0,0 });
	auto floor_shape = new ygl::shape();
	floor_shape->name = "floor_shape";
	floor_shape->pos = yb::make_quadXZ(5000.f);
	floor_shape->quads = { {0,1,2,3} };
	floor_shape->mat = floor_mat;
	floor_shape->texcoord = std::vector<ygl::vec2f>(4, { 0.f,0.f });
	floor_shape->norm = ygl::compute_normals({}, {}, floor_shape->quads, floor_shape->pos);
	floor_shape->color = std::vector<ygl::vec4f>(4, { 1,1,1,1 });
	auto floor_inst = new ygl::instance();
	floor_inst->name = "floor_inst";
	floor_inst->shp = floor_shape;
	for (int i = 0; i < 32; i++) ygl::tesselate_shape(floor_shape);
	scn->materials.push_back(floor_mat);
	scn->shapes.push_back(floor_shape);
	scn->instances.push_back(floor_inst);

	/// Building
	// Generation
	ygl::shape *open_window_shape, *closed_window_shape;
	std::tie(open_window_shape, closed_window_shape) =
		yb::make_test_windows("wnd_op", "wnd_cls");

	for (int i = 0; i < 1; i++) {
		yb::building_params *params = yb::make_rand_building_params(
			rng, open_window_shape, closed_window_shape, "building" + std::to_string(i)
		);
		params->type = yb::building_type::main_points;
		params->floor_main_points = yb::make_segmented_line(
			{ 0,0 },
			16,
			[]() {return yb::pi / 2.f;},
			[]() {static int k = 0; return (k++ / 2 + 1) * 30;}
		);
		params->floor_width = 20.f;
		params->win_pars.filled_spots_ratio = 1.f;
		params->width_delta_per_floor = -1.f;
		params->num_floors += 3 * i;
		params->roof_pars.type = yb::roof_type::crossgabled;
		params->tower_prob = 0.f;
		params->roof_pars.keep_prob = 0.f;
		params->roof_pars.color1 = params->roof_pars.color2;
		params->win_pars.filled_spots_ratio = std::max(params->win_pars.filled_spots_ratio, 0.1f);
		
		auto insts = yb::make_building(*params);
		for (auto inst : insts) {
			ygl::facet_shape(inst->shp);
			inst->shp->norm = ygl::compute_normals(
				inst->shp->lines, inst->shp->triangles, inst->shp->quads, inst->shp->pos
			);
		}
		for (auto inst : insts) yb::add_to_scene(scn, inst);
	}

	// Sky
	auto skyshape = new ygl::shape();
	auto sz = 2500.f;
	skyshape->pos = { {-sz,0,-sz},{sz,0,-sz},{sz,sz,-sz},{-sz,sz,-sz} };
	yb::displace(skyshape->pos, { 0,-sz / 2.f,0 });
	skyshape->quads = { {0,1,2,3} };
	skyshape->norm = ygl::compute_normals({}, {}, skyshape->quads, skyshape->pos);
	skyshape->color = std::vector<ygl::vec4f>(4, { 1,1,1,1 });
	skyshape->texcoord = { {0,1},{1,1},{1,0},{0,0} };
	skyshape->name = "skyshape";
	auto skytext = new ygl::texture();
	skytext->name = "sky.hdr";
	skytext->path = "sky.hdr";
	auto skymat = yb::make_material("skymat", { 1,1,1 }, skytext, ygl::zero3f);
	skyshape->mat = skymat;
	auto skyinst = new ygl::instance();
	skyinst->name = "skyinst";
	skyinst->shp = skyshape;
	scn->shapes.push_back(skyshape);
	scn->materials.push_back(skymat);
	scn->instances.push_back(skyinst);
	scn->textures.push_back(skytext);
	
	float lqs = 1000.f; // light quad size
	for (int i = 0; i < 3; i++) {
		for (int j = 2; j < 3; j++) {
			for (int k = 2; k < 3; k++) {
				if (i == 1 && j == 1 && k == 1) continue;
				yb::add_light(
					scn,
					ygl::vec3f{ -lqs + lqs*i, -lqs + lqs*j, -lqs + lqs*k },
					ygl::vec3f{ 1,1,1 } * 2e6,
					"light" + std::to_string(9 * i + 3 * j + k)
				);
			}
		}
	}
	//yb::add_light(scn, { -sz / 2,sz,-sz }, { 1e7f,1e7f,1e7f }, "sun");
	//yb::add_light(scn, { 1000,1000,0 }, { 1000000,1000000,1000000 }, "sun");

	// add camera
	ygl::vec3f cam_origin[] = { {0.f,150.f,500.f},{500.f,150.f,500.f} };
	for (int i = 0; i < 2; i++) {
		auto cam = new ygl::camera{ "cam"+std::to_string(i) };
		cam->frame = ygl::lookat_frame3f(cam_origin[i], { 0.f, 20.f, 0.f }, { 0.f, 1.f, 0.f });
		cam->yfov = 15.f * yb::pi / 180.f;
		cam->aspect = 16.0f / 9.0f;
		cam->aperture = 0.f;
		cam->focus = ygl::length(ygl::vec3f{ 0.f, 4.f, 10.f } - ygl::vec3f{ 0.f, 1.f, 0.f });
		cam->ortho = false;
		scn->cameras.push_back(cam);
	}
	ygl::save_scene("ciao.obj", scn, ygl::save_options());

	auto skyimage = ygl::make_sunsky_image(1024, yb::pi/4, 2.f, false, true);
	ygl::save_image4f("sky.hdr", skyimage);

	return 0;
}