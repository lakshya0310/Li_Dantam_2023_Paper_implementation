/*
 * Infeasibility Proof Pipeline (Li-Dantam)
 *
 * Algorithm:
 *   1. Build a PRM over C-free. Label each milestone:
 *        +1  if connected to the goal component
 *        -1  if not connected to the goal component
 *   2. Train an RBF-SVM on those C-free samples with those labels.
 *      The SVM zero-level-set f=0 is a candidate separation manifold between
 *      the goal-connected region and the rest of C-free.
 *   3. Project every C-free milestone onto f=0 via gradient line-search
 *      + regula-falsi.  These become seeds for the Coxeter BFS.
 *   4. Trace and triangulate the SVM manifold using Gudhi Coxeter
 *      triangulation (manifold_tracing + manifold_meshing).
 *   5. Verify the triangulated manifold lies entirely in C-obs by collision-
 *      checking every simplex centroid.  If every simplex is in collision the
 *      infeasibility proof is complete.
 *
 * Dependencies: OMPL, Boost.Graph, Eigen3, ThunderSVM, GUDHI,
 *               libcuckoo, OpenMP, Amino
 *
 * Aayush Rath, Lakshya Jindal
 */

extern "C" {
    #include "out.c.h"
}

#include "prm_sampler.h"
#define SVM_H_INCLUDED
#include "svm.h"
#include "man_trace.h"
#include "kinematics.h"

#include <nlopt.hpp>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/base/samplers/GaussianValidStateSampler.h>

#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <vector>
#include <random>
#include <chrono>
#include <cassert>
#include <cmath>
#include <chrono>

namespace ob = ompl::base;
namespace og = ompl::geometric;

static Kinematics* g_kin = nullptr;

inline double wrapToPi(double x) {
    x = std::fmod(x + M_PI, 2.0 * M_PI);
    if (x < 0) x += 2.0 * M_PI;
    return x - M_PI;
}


bool StateValidator::isValid(const ob::State* state) const {
    assert(g_kin && "g_kin must be set before building the PRM");
    const auto* rs = state->as<ob::RealVectorStateSpace::StateType>();

    int ndof = (int)g_kin->ndof();
    std::vector<double> q(ndof);

    for (int i = 0; i < ndof; ++i) {
        q[i] = rs->values[i];
    }

    return !g_kin->is_real_collision(q) && g_kin->in_joint_limits(q);
}
struct PRMSamples {
    std::vector<Eigen::VectorXd> samples;     // C-free
    std::vector<float>           labels;

    std::vector<Eigen::VectorXd> obstacle_samples; // NEW
};

PRMSamples build_prm_samples(
    Kinematics&                kin,
    const std::vector<double>& bounds_lo,
    const std::vector<double>& bounds_hi,
    const Eigen::VectorXd&     q_start,
    const Eigen::VectorXd&     q_goal,
    int                        n_milestones = 4000,
    double                     max_time_s   = 5.0
) {
    int ndof = (int)kin.ndof();
    assert((int)bounds_lo.size() == ndof);

    g_kin = &kin;

    auto space = std::make_shared<ob::RealVectorStateSpace>(ndof);
    ob::RealVectorBounds ompl_bounds(ndof);
    for (int i = 0; i < ndof; ++i) {
        ompl_bounds.setLow(i,  bounds_lo[i]);
        ompl_bounds.setHigh(i, bounds_hi[i]);
    }
    space->setBounds(ompl_bounds);

    auto si = std::make_shared<ob::SpaceInformation>(space);
    si->setStateValidityChecker(std::make_shared<StateValidator>(si));
    si->setValidStateSamplerAllocator(
        [](const ob::SpaceInformation* si) { 
            auto sampler = std::make_shared<ob::GaussianValidStateSampler>(si);

            sampler->setStdDev(0.1);
            return sampler;
        }
    );
    si->setup();

    // Allocate start/goal states so PRM knows the goal component
    auto start_state = si->allocState();
    auto goal_state  = si->allocState();
    {
        auto* sr = start_state->as<ob::RealVectorStateSpace::StateType>();
        auto* gr =  goal_state->as<ob::RealVectorStateSpace::StateType>();
        for (int i = 0; i < ndof; ++i) { sr->values[i] = q_start[i];
                                          gr->values[i] = q_goal[i]; }
    }
    std::cout << "Checking start state...\n";
    std::cout << "OMPL valid = "
            << si->isValid(start_state)
            << std::endl;

    PRMGraph prm(si, q_start, q_goal);
    auto pdef = std::make_shared<ob::ProblemDefinition>(si);
    pdef->setStartAndGoalStates(start_state, goal_state);
    prm.setProblemDefinition(pdef);
    prm.setup();

    // ── Grow roadmap ──────────────────────────────────────────────────────────
    auto t0 = std::chrono::steady_clock::now();
    ob::PlannerTerminationCondition ptc([&]() {
        double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
        return elapsed > max_time_s ||
               (int)boost::num_vertices(prm.getGraph()) >= n_milestones;
    });

    std::cout << "[PRM] Growing (max " << n_milestones
              << " nodes / " << max_time_s << " s)...\n";
    prm.growRoadmap(ptc);
    ob::PlannerStatus solved = prm.solve(ob::timedPlannerTerminationCondition(0.0));
    if (solved) {
        std::cout << "[PRM] Found path!\n";
    } else {
        std::cout << "[PRM] No path found.\n";
    }
    // ── Sample obstacle points explicitly ─────────────────────────────
    int n_obstacle_samples = n_milestones;  // or tune separately

    auto sampler = si->allocStateSampler();
    auto tmp_state = si->allocState();

    int collected = 0;
    int attempts  = 0;
    int max_attempts = 10 * n_obstacle_samples;  // avoid infinite loops

    std::vector<Eigen::VectorXd> obstacle_pts;
    obstacle_pts.reserve(n_obstacle_samples);

    while (collected < n_obstacle_samples && attempts < max_attempts) {
        sampler->sampleUniform(tmp_state);

        if (!si->isValid(tmp_state)) {  // <-- key condition
            auto* rs = tmp_state->as<ob::RealVectorStateSpace::StateType>();
            Eigen::VectorXd q(ndof);
            for (int i = 0; i < ndof; ++i) q[i] = rs->values[i];

            obstacle_pts.push_back(q);
            collected++;
        }
        attempts++;
    }

    si->freeState(tmp_state);

    std::cout << "[PRM] Collected " << obstacle_pts.size()
            << " obstacle samples (" << attempts << " attempts)\n";
    std::cout << "[PRM] Done. Milestones: "
              << boost::num_vertices(prm.getGraph()) << "\n";

    const auto& G        = prm.getGraph();
    auto        stateMap = boost::get(og::PRM::vertex_state_t(), G);
    std::vector<int> conn = prm.getMilestoneLabels();  // 1=goal-conn, 0=not

    PRMSamples result;
    for (auto v : boost::make_iterator_range(boost::vertices(G))) {
        const auto* rs = stateMap[v]->as<ob::RealVectorStateSpace::StateType>();
        Eigen::VectorXd q(ndof);
        for (int i = 0; i < ndof; ++i) q[i] = rs->values[i];
        result.samples.push_back(q);
        result.labels.push_back(conn[v] == 1 ? +1.0f : -1.0f);
    }

    si->freeState(start_state);
    si->freeState(goal_state);

    return result;
}
void train_svm(Manifold& manifold, const PRMSamples& prm) {
    int n_pos = 0, n_neg = 0;
    for (float l : prm.labels) (l > 0 ? n_pos : n_neg)++;

    std::cout << "[SVM] Training on " << prm.samples.size()
              << " C-free samples  (+1: " << n_pos
              << "  -1: " << n_neg << ")...\n";

    std::vector<float> labels_copy = prm.labels;
    manifold.train_samples(prm.samples, labels_copy);
    std::cout << "[SVM] Done.\n";
}
struct ProjData {
    const Eigen::VectorXd& q0;
    const Manifold&        manifold;
    int                    ndof;
};

// Objective: |F(q)|
static double proj_obj(const std::vector<double>& x,
                       std::vector<double>&       grad,
                       void*                      raw)
{
    auto* pd = static_cast<ProjData*>(raw);
    int   n  = pd->ndof;
    Eigen::VectorXd q = Eigen::Map<const Eigen::VectorXd>(x.data(), n);

    if (grad.empty()) {
        return std::abs(pd->manifold(q));
    }

    const double h = 1e-5;
    int batch_n = 2 * n + 1;
    std::vector<float> rows(batch_n * n);
    for (int j = 0; j < n; ++j) rows[j] = (float)x[j];           // row 0
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            rows[(2*i+1)*n + j] = (float)x[j];
            rows[(2*i+2)*n + j] = (float)x[j];
        }
        rows[(2*i+1)*n + i] += (float)h;
        rows[(2*i+2)*n + i] -= (float)h;
    }

    // One batch_predict call instead of 2n single calls
    std::vector<double> vals = pd->manifold.batch_predict(rows, batch_n);
    double fq = vals[0];
    for (int i = 0; i < n; ++i) {
        double df  = (vals[2*i+1] - vals[2*i+2]) / (2.0 * h);
        grad[i]    = (fq >= 0.0 ? 1.0 : -1.0) * df;   // gradient of |F|
    }
    return std::abs(fq);
}

Eigen::VectorXd project_to_manifold(
    const Eigen::VectorXd&     q0,
    const Manifold&            manifold,
    const std::vector<double>& bounds_lo,
    const std::vector<double>& bounds_hi,
    double tol      = 1e-13,
    int    max_eval = 5000
) {
    int ndof = (int)q0.size();
    if (std::abs(manifold(q0)) < tol) return q0;

    ProjData pd{q0, manifold, ndof};

    nlopt::opt opt(nlopt::LD_SLSQP, ndof);
    opt.set_lower_bounds(bounds_lo);
    opt.set_upper_bounds(bounds_hi);
    opt.set_min_objective(proj_obj, &pd);
    opt.set_xtol_rel(tol);
    opt.set_ftol_rel(tol);
    opt.set_maxeval(max_eval);

    std::vector<double> x(q0.data(), q0.data() + ndof);
    double minf = 0.0;
    try {
        opt.optimize(x, minf);
    } catch (const std::exception& e) {
        std::cerr << "[Project] NLopt: " << e.what() << " — keeping seed\n";
    }
    return Eigen::Map<Eigen::VectorXd>(x.data(), ndof);
}

std::vector<Eigen::VectorXd> project_all_samples(
    const PRMSamples& prm,
    const Manifold&   manifold,
    const std::vector<double>& bounds_lo,
    const std::vector<double>& bounds_hi,
    int n_samples = -1
) {
    std::vector<Eigen::VectorXd> all_seeds;

    // Combine both sets
    all_seeds.reserve(prm.samples.size() + prm.obstacle_samples.size());
    all_seeds.insert(all_seeds.end(), prm.samples.begin(), prm.samples.end());
    all_seeds.insert(all_seeds.end(), prm.obstacle_samples.begin(), prm.obstacle_samples.end());

    // Subsample if needed
    std::vector<int> indices(all_seeds.size());
    std::iota(indices.begin(), indices.end(), 0);

    if (n_samples > 0 && n_samples < (int)indices.size()) {
        std::vector<int> subsampled;
        subsampled.reserve(n_samples);

        double stride = (double)(indices.size() - 1) / (n_samples - 1);
        for (int k = 0; k < n_samples; ++k)
            subsampled.push_back((int)std::round(k * stride));

        indices = std::move(subsampled);
    }

    std::cout << "[Project] Projecting " << indices.size()
              << " seeds\n";

    std::vector<Eigen::VectorXd> projected(indices.size());

    #pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < (int)indices.size(); ++i)
        projected[i] = project_to_manifold(
            all_seeds[indices[i]],
            manifold,
            bounds_lo,
            bounds_hi
        );

    return projected;
}


using MeshMap = libcuckoo::cuckoohash_map<
    PermRep,
    std::vector<Eigen::VectorXd>,
    PermRepHash>;

MeshMap trace_svm_manifold(
    const std::function<double(const Eigen::VectorXd&)>& f,
    const Manifold&                                      manifold_ref,
    const std::vector<Eigen::VectorXd>&                  seeds,
    int                                                  ndof,
    double                                               cell_size = -1.0 
) {

    if (cell_size <= 0.0) cell_size = 0.1;
    std::cout << "[Trace] lambda_T = " << cell_size << "\n";

    CoxTri ct(ndof);
    ct.change_matrix(cell_size * ct.matrix());

    libcuckoo::cuckoohash_map<PermRep, Eigen::VectorXd, PermRepHash> visited;
    MeshMap mesh;

    std::cout << "[Trace] BFS with " << seeds.size() << " seeds...\n";
    manifold_tracing(ndof, f, seeds, visited, ct);

    { auto lt = visited.lock_table();
      std::cout << "[Trace] " << lt.size() << " intersecting edges.\n"; }

    manifold_meshing(ndof, ct, visited, mesh);

    { auto lt = mesh.lock_table();
      std::cout << "[Trace] " << lt.size() << " mesh simplices.\n"; }

    return mesh;
}


bool verify_manifold_in_cobs(MeshMap& mesh, Kinematics& kin) {
    auto lt = mesh.lock_table();

    int total = 0, in_cobs = 0;
    for (const auto& kv : lt) {
        const auto& pts = kv.second;
        if (pts.empty()) continue;

        // Compute simplex centroid
        Eigen::VectorXd centroid = Eigen::VectorXd::Zero(pts[0].size());
        for (const auto& p : pts) centroid += p;
        centroid /= (double)pts.size();

        std::vector<double> q(centroid.data(), centroid.data() + centroid.size());
        ++total;
        if (kin.is_real_collision(q)) ++in_cobs;
    }

    double ratio = total > 0 ? (double)in_cobs / total : 0.0;
    std::cout << "[Verify] " << in_cobs << " / " << total
              << " simplex centroids in C-obs  ("
              << int(ratio * 100) << "%).\n";

    return (in_cobs == total);
}

int main(int argc, char* argv[]) {

    struct aa_rx_sg* sg = aa_rx_sg_create();
    sg = aa_rx_dl_sg__scenegraph(sg, "");

    const std::string scene_file = (argc > 1) ? argv[1] : "../../../FK-Tracing-for-Infeasibility/scenes/scene5updated.json";
    try {
        add_obstacles(sg, scene_file);
    } catch (const std::exception& e) {
        std::cerr << "[Warn] Could not load obstacles: " << e.what() << "\n";
    }

    aa_rx_sg_init(sg);
    aa_rx_sg_cl_init(sg);

    Kinematics kin(sg);
    int ndof = (int)kin.ndof();
    std::cout << "[Main] Robot has " << ndof << " DOF.\n";

    // std::vector<double> lo(ndof, -M_PI);
    // std::vector<double> hi(ndof,  M_PI);

    static const char* config_names[] = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

    std::vector<double> lo(ndof), hi(ndof);
    for (int i = 0; i < ndof; ++i) {
        aa_rx_config_id cid = aa_rx_sg_config_id(sg, config_names[i]);
        double q_min, q_max;
        aa_rx_sg_get_limit_pos(sg, cid, &q_min, &q_max);
        lo[i] = q_min;
        hi[i] = q_max;
        std::cout << "[Main] " << config_names[i] << " limits: ["
                << lo[i] << ", " << hi[i] << "]\n";
    }

    Eigen::VectorXd q_start = Eigen::VectorXd::Zero(ndof);
    Eigen::VectorXd q_goal = Eigen::VectorXd::Zero(ndof);
    // q_goal << 1.7, -0.85, 0, -0.055;
    q_start << 0.01, 0.01, 0.01, 0.01;
    q_goal << 0.3, 0.01, 0.1, 1.52;

    std::cout << "\n[Stage 1] PRM sampling...\n";
    PRMSamples prm_data = build_prm_samples(kin, lo, hi, q_start, q_goal, 64000, 60.0);

    int n_pos = 0, n_neg = 0;
    for (float l : prm_data.labels) (l > 0 ? n_pos : n_neg)++;
    std::cout << "[Stage 1] Milestones: " << prm_data.samples.size()
              << "  (+1 goal-conn: " << n_pos
              << "  -1 not-conn: "   << n_neg << ")\n";

    if (n_pos == 0 || n_neg == 0) {
        std::cerr << "[Main] Both classes required for SVM — "
                     "check start/goal or increase n_milestones.\n";
        return 1;
    }

    std::cout << "\n[Stage 2] Training SVM...\n";
    Manifold manifold(ndof);
    train_svm(manifold, prm_data);

    std::function<double(const Eigen::VectorXd&)> f =
        [&manifold](const Eigen::VectorXd& q) { return manifold(q); };

    std::cout << "\n[Stage 3] Projecting milestones onto SVM manifold...\n";
    const int n_projection_samples = 32000;
    std::vector<Eigen::VectorXd> projected =
        project_all_samples(prm_data, manifold, lo, hi, n_projection_samples);

    std::vector<Eigen::VectorXd> seeds;
    seeds.reserve(projected.size());
    const double seed_tol = 1e-6;
    for (const auto& p : projected)
        if (std::abs(f(p)) < seed_tol) seeds.push_back(p);

    std::cout << "[Stage 3] Valid seeds: " << seeds.size()
              << " / " << projected.size() << "\n";

    if (seeds.empty()) {
        std::cerr << "[Main] No valid seeds — SVM may not have a clean "
                     "zero-crossing in C-free. Try more milestones.\n";
        return 1;
    }

    std::cout << "\n[Stage 4] Tracing SVM manifold...\n";
    auto start = std::chrono::high_resolution_clock::now();
    MeshMap mesh = trace_svm_manifold(f, manifold, seeds, ndof, 0.1);
    auto end = std::chrono::high_resolution_clock::now();
    auto dur = std::chrono::duration_cast<std::chrono::seconds>(end - start);
    std::cout << "Time taken for tracing: " << dur.count() << " seconds\n";

    std::cout << "\n[Stage 5] Verifying manifold is in C-obs...\n";
    start = std::chrono::high_resolution_clock::now();
    bool proven = verify_manifold_in_cobs(mesh, kin);
    end = std::chrono::high_resolution_clock::now();
    dur = std::chrono::duration_cast<std::chrono::seconds>(end - start);
    std::cout << "Time taken for verifying: " << dur.count() << " seconds\n";
    if (proven)
        std::cout << "[Main] Infeasibility PROVEN: manifold lies entirely in C-obs.\n";
    else
        std::cout << "[Main] Proof INCOMPLETE: some simplex centroids are not in "
                     "collision. Consider a finer cell_size or more milestones.\n";

    {
        std::ofstream out("manifold_mesh.csv");
        if (out) {
            auto lt = mesh.lock_table();
            for (const auto& kv : lt)
                for (const auto& pt : kv.second) {
                    for (int i = 0; i < (int)pt.size(); ++i)
                        out << pt[i] << (i + 1 < (int)pt.size() ? ',' : '\n');
                }
            std::cout << "[Main] Mesh written to manifold_mesh.csv\n";
        }
    }

    std::cout << "\n[Main] Pipeline complete.\n";
    return 0;
}