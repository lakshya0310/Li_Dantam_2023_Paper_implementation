#include <amino.h>
#include <amino/rx.h>

struct aa_rx_sg * aa_rx_dl_sg__scenegraph(struct aa_rx_sg *sg, const char *root) 
{
	
	if (NULL == sg) 
	{
		sg = aa_rx_sg_create();
	}
	{
		static const double q[4] = {0.0, 0.0, 0.0, 1.0};
		static const double v[3] = {0.0, 0.0, 0.0};
		aa_rx_sg_add_frame_fixed(sg, root, "base_link-collision", q, v);
	}
	{
		static const double q[4] = {0.0, 0.0, 0.0, 1.0};
		static const double v[3] = {0.0, 0.0, 0.05000000074505806};
		aa_rx_sg_add_frame_fixed(sg, root, "base_link-inertial", q, v);
	}
	{
		static const double q[4] = {0.0, 0.0, 0.0, 1.0};
		static const double v[3] = {0.0, 0.0, 0.550000011920929};
		aa_rx_sg_add_frame_fixed(sg, root, "base_link-visual", q, v);
	}
	{
		static const double q[4] = {0.0, 0.0, 0.0, 1.0};
		static const double v[3] = {0.0, 0.25, 0.0};
		aa_rx_sg_add_frame_fixed(sg, "joint3", "dummy_link-collision", q, v);
	}
	{
		static const double q[4] = {0.0, 0.0, 0.0, 1.0};
		static const double v[3] = {0.0, 0.0, 0.05000000074505806};
		aa_rx_sg_add_frame_fixed(sg, "joint3", "dummy_link-inertial", q, v);
	}
	{
		static const double q[4] = {0.0, 0.0, 0.0, 1.0};
		static const double v[3] = {0.0, 0.25, 0.0};
		aa_rx_sg_add_frame_fixed(sg, "joint3", "dummy_link-visual", q, v);
	}
	{
		static const double q[4] = {0.0, 0.0, 0.0, 1.0};
		static const double v[3] = {0.0, 0.0, 0.15000000596046448};
		aa_rx_sg_add_frame_fixed(sg, "joint5", "ee-collision", q, v);
	}
	{
		static const double q[4] = {0.0, 0.0, 0.0, 1.0};
		static const double v[3] = {0.0, 0.0, 0.15000000596046448};
		aa_rx_sg_add_frame_fixed(sg, "joint5", "ee-inertial", q, v);
	}
	{
		static const double q[4] = {0.0, 0.0, 0.0, 1.0};
		static const double v[3] = {0.0, 0.0, 1.0499999523162842};
		static const double axis[3] = {0.0, 0.0, 1.0};
		aa_rx_sg_add_frame_revolute(sg, root, "joint1", q, v, "joint1", axis, 0.0);
		aa_rx_sg_set_limit_pos(sg, "joint1", -1.8325999975204468, 1.8325999975204468);
		aa_rx_sg_set_limit_vel(sg, "joint1", -1.0, 1.0);
		aa_rx_sg_set_limit_eff(sg, "joint1", -5.0, 5.0);
	}
	{
		static const double q[4] = {0.0, 0.0, 0.0, 1.0};
		static const double v[3] = {0.0, 0.4000000059604645, 0.10000000149011612};
		static const double axis[3] = {0.0, 0.0, 1.0};
		aa_rx_sg_add_frame_revolute(sg, "joint1", "joint2", q, v, "joint2", axis, 0.0);
		aa_rx_sg_set_limit_pos(sg, "joint2", -2.7488999366760254, 2.7488999366760254);
		aa_rx_sg_set_limit_vel(sg, "joint2", -1.0, 1.0);
		aa_rx_sg_set_limit_eff(sg, "joint2", -5.0, 5.0);
	}
	{
		static const double q[4] = {0.0, 0.0, 0.0, 1.0};
		static const double v[3] = {0.0, 0.44999998807907104, -0.44999998807907104};
		static const double axis[3] = {0.0, 0.0, 1.0};
		aa_rx_sg_add_frame_revolute(sg, "joint2", "joint3", q, v, "joint3", axis, 0.0);
		aa_rx_sg_set_limit_pos(sg, "jointscenes3", -3.0, 3.0);
		aa_rx_sg_set_limit_vel(sg, "joint3", -1.0, 1.0);
		aa_rx_sg_set_limit_eff(sg, "joint3", -5.0, 5.0);
	}
	{
		static const double q[4] = {0.0, 0.0, 0.0, 1.0};
		static const double v[3] = {0.0, 0.0, 0.05000000074505806};
		static const double axis[3] = {0.0, 0.0, 1.0};
		aa_rx_sg_add_frame_prismatic(sg, "joint3", "joint4", q, v, "joint4", axis, 0.0);
		aa_rx_sg_set_limit_pos(sg, "joint4", -0.05999999865889549, 0.10000000149011612);
		aa_rx_sg_set_limit_vel(sg, "joint4", -1.0, 1.0);
		aa_rx_sg_set_limit_eff(sg, "joint4", -5.0, 5.0);
	}
	{
		static const double q[4] = {0.0, 0.0, 0.0, 1.0};
		static const double v[3] = {0.0, 0.0, -0.44999998807907104};
		aa_rx_sg_add_frame_fixed(sg, "joint4", "joint5", q, v);
	}
	{
		static const double q[4] = {0.0, 0.0, 0.0, 1.0};
		static const double v[3] = {0.0, 0.25, 0.0};
		aa_rx_sg_add_frame_fixed(sg, "joint1", "link1-collision", q, v);
	}
	{
		static const double q[4] = {0.0, 0.0, 0.0, 1.0};
		static const double v[3] = {0.0, 0.0, 0.05000000074505806};
		aa_rx_sg_add_frame_fixed(sg, "joint1", "link1-inertial", q, v);
	}
	{
		static const double q[4] = {0.0, 0.0, 0.0, 1.0};
		static const double v[3] = {0.0, 0.25, 0.0};
		aa_rx_sg_add_frame_fixed(sg, "joint1", "link1-visual", q, v);
	}
	{
		static const double q[4] = {0.0, 0.0, 0.0, 1.0};
		static const double v[3] = {0.0, 0.0, 0.15000000596046448};
		aa_rx_sg_add_frame_fixed(sg, "joint2", "link2-collision", q, v);
	}
	{
		static const double q[4] = {0.0, 0.0, 0.0, 1.0};
		static const double v[3] = {0.0, 0.0, 0.15000000596046448};
		aa_rx_sg_add_frame_fixed(sg, "joint2", "link2-inertial", q, v);
	}
	{
		static const double q[4] = {0.0, 0.0, 0.0, 1.0};
		static const double v[3] = {0.0, 0.25, 0.0};
		aa_rx_sg_add_frame_fixed(sg, "joint2", "link2-visual", q, v);
	}
	{
		static const double q[4] = {0.0, 0.0, 0.0, 1.0};
		static const double v[3] = {0.0, 0.0, 0.15000000596046448};
		aa_rx_sg_add_frame_fixed(sg, "joint4", "link3-collision", q, v);
	}
	{
		static const double q[4] = {0.0, 0.0, 0.0, 1.0};
		static const double v[3] = {0.0, 0.0, 0.15000000596046448};
		aa_rx_sg_add_frame_fixed(sg, "joint4", "link3-inertial", q, v);
	}
	{
		static const double q[4] = {0.0, 0.0, 0.0, 1.0};
		static const double v[3] = {0.0, 0.0, 0.20000000298023224};
		aa_rx_sg_add_frame_fixed(sg, "joint4", "link3-visual", q, v);
	}
	{
		struct aa_rx_geom * geom;
		struct aa_rx_geom_opt * opt = aa_rx_geom_opt_create();
		aa_rx_geom_opt_set_color3(opt, 0.800000011920929, 0.800000011920929, 0.800000011920929);
		aa_rx_geom_opt_set_alpha(opt, 1.0);
		aa_rx_geom_opt_set_specular3(opt, 0, 0, 0);
		aa_rx_geom_opt_set_visual(opt, 0);
		aa_rx_geom_opt_set_collision(opt, 1);
		aa_rx_geom_opt_set_no_shadow(opt, 0);
		geom = aa_rx_geom_cylinder(opt, 0.10000000149011612, 0.05000000074505806);
		aa_rx_geom_attach(sg, "base_link-collision", geom);
		aa_rx_geom_opt_destroy(opt);
	}
	{
		struct aa_rx_geom * geom;
		struct aa_rx_geom_opt * opt = aa_rx_geom_opt_create();
		aa_rx_geom_opt_set_color3(opt, 0.800000011920929, 0.800000011920929, 0.800000011920929);
		aa_rx_geom_opt_set_alpha(opt, 1.0);
		aa_rx_geom_opt_set_specular3(opt, 0, 0, 0);
		aa_rx_geom_opt_set_visual(opt, 1);
		aa_rx_geom_opt_set_collision(opt, 0);
		aa_rx_geom_opt_set_no_shadow(opt, 0);
		geom = aa_rx_geom_cylinder(opt, 1.100000023841858, 0.10000000149011612);
		aa_rx_geom_attach(sg, "base_link-visual", geom);
		aa_rx_geom_opt_destroy(opt);
	}
	{
		struct aa_rx_geom * geom;
		struct aa_rx_geom_opt * opt = aa_rx_geom_opt_create();
		aa_rx_geom_opt_set_color3(opt, 0.800000011920929, 0.800000011920929, 0.800000011920929);
		aa_rx_geom_opt_set_alpha(opt, 1.0);
		aa_rx_geom_opt_set_specular3(opt, 0, 0, 0);
		aa_rx_geom_opt_set_visual(opt, 0);
		aa_rx_geom_opt_set_collision(opt, 1);
		aa_rx_geom_opt_set_no_shadow(opt, 0);
		static const double dimension[3] = {0.20000000298023224, 0.5, 0.10000000149011612};
		geom = aa_rx_geom_box(opt, dimension);
		aa_rx_geom_attach(sg, "dummy_link-collision", geom);
		aa_rx_geom_opt_destroy(opt);
	}
	{
		struct aa_rx_geom * geom;
		struct aa_rx_geom_opt * opt = aa_rx_geom_opt_create();
		aa_rx_geom_opt_set_color3(opt, 0.800000011920929, 0.800000011920929, 0.800000011920929);
		aa_rx_geom_opt_set_alpha(opt, 1.0);
		aa_rx_geom_opt_set_specular3(opt, 0, 0, 0);
		aa_rx_geom_opt_set_visual(opt, 1);
		aa_rx_geom_opt_set_collision(opt, 0);
		aa_rx_geom_opt_set_no_shadow(opt, 0);
		static const double dimension[3] = {0.0, 0.0, 0.0};
		geom = aa_rx_geom_box(opt, dimension);
		aa_rx_geom_attach(sg, "dummy_link-visual", geom);
		aa_rx_geom_opt_destroy(opt);
	}
	{
		struct aa_rx_geom * geom;
		struct aa_rx_geom_opt * opt = aa_rx_geom_opt_create();
		aa_rx_geom_opt_set_color3(opt, 0.800000011920929, 0.800000011920929, 0.800000011920929);
		aa_rx_geom_opt_set_alpha(opt, 1.0);
		aa_rx_geom_opt_set_specular3(opt, 0, 0, 0);
		aa_rx_geom_opt_set_visual(opt, 0);
		aa_rx_geom_opt_set_collision(opt, 1);
		aa_rx_geom_opt_set_no_shadow(opt, 0);
		static const double dimension[3] = {0.20000000298023224, 0.5, 0.10000000149011612};
		geom = aa_rx_geom_box(opt, dimension);
		aa_rx_geom_attach(sg, "ee-collision", geom);
		aa_rx_geom_opt_destroy(opt);
	}
	{
		struct aa_rx_geom * geom;
		struct aa_rx_geom_opt * opt = aa_rx_geom_opt_create();
		aa_rx_geom_opt_set_color3(opt, 0.800000011920929, 0.800000011920929, 0.800000011920929);
		aa_rx_geom_opt_set_alpha(opt, 1.0);
		aa_rx_geom_opt_set_specular3(opt, 0, 0, 0);
		aa_rx_geom_opt_set_visual(opt, 1);
		aa_rx_geom_opt_set_collision(opt, 0);
		aa_rx_geom_opt_set_no_shadow(opt, 0);
		static const double dimension[3] = {0.30000001192092896, 0.30000001192092896, 0.05000000074505806};
		geom = aa_rx_geom_box(opt, dimension);
		aa_rx_geom_attach(sg, "joint5", geom);
		aa_rx_geom_opt_destroy(opt);
	}
	{
		struct aa_rx_geom * geom;
		struct aa_rx_geom_opt * opt = aa_rx_geom_opt_create();
		aa_rx_geom_opt_set_color3(opt, 0.800000011920929, 0.800000011920929, 0.800000011920929);
		aa_rx_geom_opt_set_alpha(opt, 1.0);
		aa_rx_geom_opt_set_specular3(opt, 0, 0, 0);
		aa_rx_geom_opt_set_visual(opt, 0);
		aa_rx_geom_opt_set_collision(opt, 1);
		aa_rx_geom_opt_set_no_shadow(opt, 0);
		static const double dimension[3] = {0.20000000298023224, 0.5, 0.10000000149011612};
		geom = aa_rx_geom_box(opt, dimension);
		aa_rx_geom_attach(sg, "link1-collision", geom);
		aa_rx_geom_opt_destroy(opt);
	}
	{
		struct aa_rx_geom * geom;
		struct aa_rx_geom_opt * opt = aa_rx_geom_opt_create();
		aa_rx_geom_opt_set_color3(opt, 0.800000011920929, 0.800000011920929, 0.800000011920929);
		aa_rx_geom_opt_set_alpha(opt, 1.0);
		aa_rx_geom_opt_set_specular3(opt, 0, 0, 0);
		aa_rx_geom_opt_set_visual(opt, 1);
		aa_rx_geom_opt_set_collision(opt, 0);
		aa_rx_geom_opt_set_no_shadow(opt, 0);
		static const double dimension[3] = {0.20000000298023224, 0.5, 0.10000000149011612};
		geom = aa_rx_geom_box(opt, dimension);
		aa_rx_geom_attach(sg, "link1-visual", geom);
		aa_rx_geom_opt_destroy(opt);
	}
	{
		struct aa_rx_geom * geom;
		struct aa_rx_geom_opt * opt = aa_rx_geom_opt_create();
		aa_rx_geom_opt_set_color3(opt, 0.800000011920929, 0.800000011920929, 0.800000011920929);
		aa_rx_geom_opt_set_alpha(opt, 1.0);
		aa_rx_geom_opt_set_specular3(opt, 0, 0, 0);
		aa_rx_geom_opt_set_visual(opt, 0);
		aa_rx_geom_opt_set_collision(opt, 1);
		aa_rx_geom_opt_set_no_shadow(opt, 0);
		static const double dimension[3] = {0.20000000298023224, 0.5, 0.10000000149011612};
		geom = aa_rx_geom_box(opt, dimension);
		aa_rx_geom_attach(sg, "link2-collision", geom);
		aa_rx_geom_opt_destroy(opt);
	}
	{
		struct aa_rx_geom * geom;
		struct aa_rx_geom_opt * opt = aa_rx_geom_opt_create();
		aa_rx_geom_opt_set_color3(opt, 0.800000011920929, 0.800000011920929, 0.800000011920929);
		aa_rx_geom_opt_set_alpha(opt, 1.0);
		aa_rx_geom_opt_set_specular3(opt, 0, 0, 0);
		aa_rx_geom_opt_set_visual(opt, 1);
		aa_rx_geom_opt_set_collision(opt, 0);
		aa_rx_geom_opt_set_no_shadow(opt, 0);
		static const double dimension[3] = {0.20000000298023224, 0.5, 0.10000000149011612};
		geom = aa_rx_geom_box(opt, dimension);
		aa_rx_geom_attach(sg, "link2-visual", geom);
		aa_rx_geom_opt_destroy(opt);
	}
	{
		struct aa_rx_geom * geom;
		struct aa_rx_geom_opt * opt = aa_rx_geom_opt_create();
		aa_rx_geom_opt_set_color3(opt, 0.800000011920929, 0.800000011920929, 0.800000011920929);
		aa_rx_geom_opt_set_alpha(opt, 1.0);
		aa_rx_geom_opt_set_specular3(opt, 0, 0, 0);
		aa_rx_geom_opt_set_visual(opt, 0);
		aa_rx_geom_opt_set_collision(opt, 1);
		aa_rx_geom_opt_set_no_shadow(opt, 0);
		static const double dimension[3] = {0.20000000298023224, 0.5, 0.10000000149011612};
		geom = aa_rx_geom_box(opt, dimension);
		aa_rx_geom_attach(sg, "link3-collision", geom);
		aa_rx_geom_opt_destroy(opt);
	}
	{
		struct aa_rx_geom * geom;
		struct aa_rx_geom_opt * opt = aa_rx_geom_opt_create();
		aa_rx_geom_opt_set_color3(opt, 0.800000011920929, 0.800000011920929, 0.800000011920929);
		aa_rx_geom_opt_set_alpha(opt, 1.0);
		aa_rx_geom_opt_set_specular3(opt, 0, 0, 0);
		aa_rx_geom_opt_set_visual(opt, 1);
		aa_rx_geom_opt_set_collision(opt, 0);
		aa_rx_geom_opt_set_no_shadow(opt, 0);
		static const double dimension[3] = {0.10000000149011612, 0.10000000149011612, 0.4000000059604645};
		geom = aa_rx_geom_box(opt, dimension);
		aa_rx_geom_attach(sg, "link3-visual", geom);
		aa_rx_geom_opt_destroy(opt);
	}
	return sg;
}