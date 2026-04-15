#include "prm_sampler.h"

/*

Generate samples with labels for goal connected
and not connected using a PRM Graph
with the help of OMPL

Aayush Rath, Lakshya Jindal

*/

std::vector<int> PRMGraph::getMilestoneLabels() {
    // Number of vertices
    size_t n = boost::num_vertices(this->g_);

    // Component id for each vertex
    std::vector<int> component(n);
    int num_components = boost::connected_components(this->g_, &component[0]);

    // Find goal vertex
    Vertex goalVertex = findClosestVertex(goal_);
    int goalComponent = component[goalVertex];

    // Assign labels
    std::vector<int> labels(n, 0);
    for (size_t i = 0; i < n; ++i) {
        if (component[i] == goalComponent)
            labels[i] = 1;
    }

    return labels;
}

Vertex PRMGraph::findClosestVertex(const Eigen::VectorXd& q) {
    auto stateMap = boost::get(vertex_state_t(), this->g_);

    Vertex closest = *vertices(this->g_).first;
    double minDist = std::numeric_limits<double>::infinity();
    for (auto v : boost::make_iterator_range(vertices(this->g_))) {

        const ob::State* s = stateMap[v];
        const auto* rs = s->as<ob::RealVectorStateSpace::StateType>();

        double dist = 0.0;
        for (unsigned int i = 0; i < q.size(); ++i) {
            double d = rs->values[i] - q[i];
            dist += d * d;
        }

        if (dist < minDist) {
            minDist = dist;
            closest = v;
        }
    }

    return closest;
}

void PRMGraph::printLabeledSamples(const std::vector<int>& labels) {
    auto stateMap = boost::get(vertex_state_t(), this->g_);

    std::cout << "\n=== Connected to Goal ===\n";
    for (auto v : boost::make_iterator_range(vertices(this->g_))) {
        if (labels[v] == 1) {
            const auto* rs = stateMap[v]->as<ob::RealVectorStateSpace::StateType>();
            std::cout << rs->values[0] << ", " << rs->values[1] << "\n";
        }
    }

    std::cout << "\n=== NOT Connected to Goal ===\n";
    for (auto v : boost::make_iterator_range(vertices(this->g_))) {
        if (labels[v] == 0) {
            const auto* rs = stateMap[v]->as<ob::RealVectorStateSpace::StateType>();
            std::cout << rs->values[0] << ", " << rs->values[1] << "\n";
        }
    }
}