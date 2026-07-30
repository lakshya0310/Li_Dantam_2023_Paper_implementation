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
		aa_rx_sg_add_frame_fixed(sg, root, "base_link-visual", q, v);
	}
	{
		static const double q[4] = {0.0, 0.0, 0.0, 1.0};
		static const double v[3] = {0.0, 0.0, 0.0};
		static const double axis[3] = {0.0, 0.0, 1.0};
		aa_rx_sg_add_frame_revolute(sg, root, "joint1", q, v, "joint1", axis, 0.0);
		aa_rx_sg_set_limit_pos(sg, "joint1", 0.0, 1.5700000524520874);
		aa_rx_sg_set_limit_vel(sg, "joint1", -1.0, 1.0);
		aa_rx_sg_set_limit_eff(sg, "joint1", -5.0, 5.0);
	}
	{
		static const double q[4] = {0.0, 0.0, 0.0, 1.0};
		static const double v[3] = {0.0, 0.0, 0.0};
		static const double axis[3] = {0.0, 1.0, 0.0};
		aa_rx_sg_add_frame_revolute(sg, "joint1", "joint2", q, v, "joint2", axis, 0.0);
		aa_rx_sg_set_limit_pos(sg, "joint2", 0.0, 1.5700000524520874);
		aa_rx_sg_set_limit_vel(sg, "joint2", -1.0, 1.0);
		aa_rx_sg_set_limit_eff(sg, "joint2", -5.0, 5.0);
	}
	{
		static const double q[4] = {0.0, 0.0, 0.0, 1.0};
		static const double v[3] = {0.0, 0.0, 0.0};
		static const double axis[3] = {1.0, 0.0, 0.0};
		aa_rx_sg_add_frame_revolute(sg, "joint2", "joint3", q, v, "joint3", axis, 0.0);
		aa_rx_sg_set_limit_pos(sg, "joint3", 0.0, 1.5700000524520874);
		aa_rx_sg_set_limit_vel(sg, "joint3", -1.0, 1.0);
		aa_rx_sg_set_limit_eff(sg, "joint3", -5.0, 5.0);
	}
	{
		static const double q[4] = {0.0, 0.0, 0.0, 1.0};
		static const double v[3] = {0.0, 0.0, 0.699999988079071};
		static const double axis[3] = {0.0, 1.0, 0.0};
		aa_rx_sg_add_frame_revolute(sg, "joint3", "joint4", q, v, "joint4", axis, 0.0);
		aa_rx_sg_set_limit_pos(sg, "joint4", -1.5700000524520874, 1.5700000524520874);
		aa_rx_sg_set_limit_vel(sg, "joint4", -1.0, 1.0);
		aa_rx_sg_set_limit_eff(sg, "joint4", -5.0, 5.0);
	}
	{
		static const double q[4] = {0.0, 0.0, 0.0, 1.0};
		static const double v[3] = {0.0, 0.0, 0.25};
		aa_rx_sg_add_frame_fixed(sg, "joint1", "link1-collision", q, v);
	}
	{
		static const double q[4] = {0.0, 0.0, 0.0, 1.0};
		static const double v[3] = {0.0, 0.0, 0.3499999940395355};
		aa_rx_sg_add_frame_fixed(sg, "joint3", "link3-collision", q, v);
	}
	{
		static const double q[4] = {0.0, 0.0, 0.0, 1.0};
		static const double v[3] = {0.0, 0.0, 0.3499999940395355};
		aa_rx_sg_add_frame_fixed(sg, "joint3", "link3-visual", q, v);
	}
	{
		static const double q[4] = {0.0, 0.0, 0.0, 1.0};
		static const double v[3] = {0.0, 0.0, 0.30000001192092896};
		aa_rx_sg_add_frame_fixed(sg, "joint4", "link4-collision", q, v);
	}
	{
		static const double q[4] = {0.0, 0.0, 0.0, 1.0};
		static const double v[3] = {0.0, 0.0, 0.30000001192092896};
		aa_rx_sg_add_frame_fixed(sg, "joint4", "link4-visual", q, v);
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
		geom = aa_rx_geom_sphere(opt, 0.0);
		aa_rx_geom_attach(sg, "base_link-visual", geom);
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
		aa_rx_geom_attach(sg, "joint1", geom);
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
		static const double dimension[3] = {0.0, 0.0, 0.0};
		geom = aa_rx_geom_box(opt, dimension);
		aa_rx_geom_attach(sg, "joint2", geom);
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
		aa_rx_geom_attach(sg, "joint2", geom);
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
		static const double dimension[3] = {0.10000000149011612, 0.10000000149011612, 0.5};
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
		aa_rx_geom_opt_set_visual(opt, 0);
		aa_rx_geom_opt_set_collision(opt, 1);
		aa_rx_geom_opt_set_no_shadow(opt, 0);
		static const double dimension[3] = {0.10000000149011612, 0.10000000149011612, 0.699999988079071};
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
		static const double dimension[3] = {0.10000000149011612, 0.10000000149011612, 0.699999988079071};
		geom = aa_rx_geom_box(opt, dimension);
		aa_rx_geom_attach(sg, "link3-visual", geom);
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
		static const double dimension[3] = {0.10000000149011612, 0.10000000149011612, 0.5};
		geom = aa_rx_geom_box(opt, dimension);
		aa_rx_geom_attach(sg, "link4-collision", geom);
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
		static const double dimension[3] = {0.10000000149011612, 0.10000000149011612, 0.5};
		geom = aa_rx_geom_box(opt, dimension);
		aa_rx_geom_attach(sg, "link4-visual", geom);
		aa_rx_geom_opt_destroy(opt);
	}
	return sg;
}