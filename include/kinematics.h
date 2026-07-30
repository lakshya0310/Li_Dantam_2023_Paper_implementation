#pragma once

#include <amino.h>
#include <amino/rx/scenegraph.h>
#include <amino/rx/scene_geom.h>
#include <amino/rx/scene_fk.h>
#include <amino/rx/scene_ik.h>
#include <amino/rx/scene_sub.h>
#include <amino/rx/scene_collision.h>

#include <string>
#include <vector>
#include <memory>
#include <stdexcept>
#include <functional>
#include <cmath>
#include <cassert>
#include <fstream>
#include <nlohmann/json.hpp>

struct aa_rx_sg;
struct aa_rx_cl;
struct aa_rx_sg_sub;

class Kinematics {

public:
    explicit Kinematics(struct aa_rx_sg* sg) :
        sg_(sg),
        nframes_(aa_rx_sg_frame_count(sg)),
        ndof_(aa_rx_sg_config_count(sg)),
        tf_rel_(nframes_ * 7, 0.0),
        tf_abs_(nframes_ * 7, 0.0) {
            cl_ = aa_rx_cl_create(sg_);
            if (!cl_) throw std::runtime_error("Failed to create cl context!");

            allow_adjacent_links();

            std::vector<double> q0(ndof_, 0.0);
            compute_fk(q0);

            q_lower_.resize(ndof_);
            q_upper_.resize(ndof_);

            for (size_t i = 0; i < ndof_; ++i) {
                aa_rx_config_id cid = (aa_rx_config_id)i;

                aa_rx_sg_get_limit_pos(sg_,
                                    cid,
                                    &q_lower_[i],
                                    &q_upper_[i]);
            }
    }

    Kinematics(const Kinematics&) = delete;
    Kinematics& operator=(const Kinematics&) = delete;
    Kinematics(Kinematics&& o) noexcept
        : sg_(o.sg_), cl_(o.cl_)
        , nframes_(o.nframes_), ndof_(o.ndof_)
        , tf_rel_(std::move(o.tf_rel_))
        , tf_abs_(std::move(o.tf_abs_))
    { o.sg_ = nullptr; o.cl_ = nullptr; }

    ~Kinematics() {
        if (cl_)  aa_rx_cl_destroy(cl_);
        if (sg_)  aa_rx_sg_destroy(sg_);
    }

    size_t ndof() const { return ndof_; }
    size_t nframes() const { return nframes_; }
    struct aa_rx_sg* sg() { return sg_; }

    aa_rx_frame_id frame_id(const std::string& name) const {
        return aa_rx_sg_config_id(sg_, name.c_str());
    }

    void compute_fk(const std::vector<double>& q) {
        assert(q.size() == ndof_);
        aa_rx_sg_tf(sg_, ndof_, q.data(), nframes_, tf_rel_.data(), 7, tf_abs_.data(), 7);
    }

    const double* frame_tf(const std::string& name) const {
        aa_rx_frame_id fid = aa_rx_sg_frame_id(sg_, name.c_str());
        if (fid == AA_RX_FRAME_NONE) throw std::invalid_argument("Unknown frame: " + name);
        return tf_abs_.data() + static_cast<ptrdiff_t>(fid) * 7;
    }

    const double* frame_tf(aa_rx_frame_id fid) const {
        return tf_abs_.data() + static_cast<ptrdiff_t>(fid) * 7;
    }

    const double* frame_pos(const std::string& name) const {
        return frame_tf(name) + 4;
    }

    bool in_collision(const std::vector<double>& q) {
        compute_fk(q);
        int hit = aa_rx_cl_check(cl_,
                                  nframes_, tf_abs_.data(), 7,
                                  nullptr);
        return hit != 0;
    }

    bool in_joint_limits(const std::vector<double>& q) const
    {
        assert(q.size() == ndof_);

        for (size_t i = 0; i < ndof_; ++i) {
            if (q[i] < q_lower_[i] || q[i] > q_upper_[i])
                return false;
        }

        return true;
    }

    std::vector<bool> in_collision_batch(
        const std::vector<std::vector<double>>& configs
    ) {
        std::vector<bool> results;
        results.reserve(configs.size());
        for (const auto& q : configs)
            results.push_back(in_collision(q));
        return results;
    }

    std::vector<std::pair<std::string,std::string>> collision_pairs(
        const std::vector<double>& q
    ) {
        compute_fk(q);

        struct aa_rx_cl_set* cl_set = aa_rx_cl_set_create(sg_);
        aa_rx_cl_check(cl_, nframes_, tf_abs_.data(), 7, cl_set);

        std::vector<std::pair<std::string,std::string>> result;
        for (aa_rx_frame_id i = 0; i < (aa_rx_frame_id)nframes_; ++i) {
            for (aa_rx_frame_id j = i+1; j < (aa_rx_frame_id)nframes_; ++j) {
                if (aa_rx_cl_set_get(cl_set, i, j)) {
                    const char* na = aa_rx_sg_frame_name(sg_, i);
                    const char* nb = aa_rx_sg_frame_name(sg_, j);
                    if (na && nb)
                        result.push_back({na, nb});
                }
            }
        }
        aa_rx_cl_set_destroy(cl_set);
        return result;
    }

    bool is_real_collision(const std::vector<double>& q) {
        auto pairs = collision_pairs(q);
        for (auto& p : pairs) {
            bool a_is_obs = p.first.find("obs_") != std::string::npos;
            bool b_is_obs = p.second.find("obs_") != std::string::npos;
            
            // Return true only if one is an obstacle and one is NOT
            // (This ignores obs vs obs)
            if (a_is_obs != b_is_obs) return true;
            
            // You might still want robot vs robot (a_is_obs == false && b_is_obs == false)
            // if (!a_is_obs && !b_is_obs) return true;
        }
        return false;
    }

    void allow_collision(const std::string& a, const std::string& b) {
        aa_rx_sg_allow_collision_name(sg_, a.c_str(), b.c_str(), 1);
    }


private:
    size_t nframes_, ndof_;
    struct aa_rx_sg* sg_ = nullptr;
    struct aa_rx_cl* cl_ = nullptr;

    std::vector<double> q_lower_;
    std::vector<double> q_upper_;

    std::vector<double> tf_rel_;
    std::vector<double> tf_abs_;

    void allow_adjacent_links() {
        for (aa_rx_frame_id fid = 0; fid < (aa_rx_frame_id)nframes_; fid++) {
            aa_rx_frame_id pid = aa_rx_sg_frame_parent(sg_, fid);
            if (pid == AA_RX_FRAME_ROOT || pid == AA_RX_FRAME_NONE) continue;
            const char* na = aa_rx_sg_frame_name(sg_, fid);
            const char* nb = aa_rx_sg_frame_name(sg_, pid);
            if (na && nb) aa_rx_sg_allow_collision_name(sg_, na, nb, 1);
        }
    }
};


void add_obstacles (
    struct aa_rx_sg* sg,
    const std::string& filepath
){
    std::ifstream file(filepath);
    if (!file.is_open()) throw std::runtime_error("Can't open the file bruh!: " + filepath);

    nlohmann::json j;
    file >> j;

    if (j.contains("boxes")) {
        int count = 0;
        for (auto& b : j["boxes"]) {
            std::string frame_name = "obs_box_" + std::to_string(count++);

            double size[3] = {b["size"][0], b["size"][1], b["size"][2]};
            double rpy[3] = {b["rpy"][0], b["rpy"][1], b["rpy"][2]};
            double center[3] = {b["center"][0], b["center"][1], b["center"][2]};
            double quat[4];
            aa_tf_eulerzyx2quat(rpy[0], rpy[1], rpy[2], quat);



            aa_rx_sg_add_frame_fixed(sg, "", frame_name.c_str(), quat, center);
            struct aa_rx_geom_opt* opt = aa_rx_geom_opt_create();
            aa_rx_geom_opt_set_collision(opt, 1);
            aa_rx_geom_opt_set_color3(opt, 0.5, 0.5, 0.5);
            struct aa_rx_geom* shape = aa_rx_geom_box(opt, size);
            aa_rx_geom_attach(sg, frame_name.c_str(), shape);
            aa_rx_geom_opt_destroy(opt);
        }
    }

    if (j.contains("spheres")) {
        int count = 0;
        for (auto& s : j["spheres"]) {
            std::string frame_name = "obs_sphere_" + std::to_string(count++);

            double center[3] = {s["center"][0], s["center"][1], s["center"][2]};
            double quat[4] = {0, 0, 0, 1};

            aa_rx_sg_add_frame_fixed(sg, "", frame_name.c_str(), quat, center);

            double radius = s["radius"];
            struct aa_rx_geom_opt* opt = aa_rx_geom_opt_create();
		    aa_rx_geom_opt_set_collision(opt, 1);
            aa_rx_geom_opt_set_color3(opt, 0.5, 0.5, 0.5);
            struct aa_rx_geom* shape = aa_rx_geom_sphere(opt, radius);
            aa_rx_geom_attach(sg, frame_name.c_str(), shape);
            aa_rx_geom_opt_destroy(opt);
        }
    }
}

