#ifndef PRM_H
#define PRM_H

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <boost/graph/connected_components.hpp>
#include <Eigen/Dense>

namespace ob = ompl::base;
namespace og = ompl::geometric;
using Vertex = boost::graph_traits<og::PRM::Graph>::vertex_descriptor;

class StateValidator : public ob::StateValidityChecker {
public:
    StateValidator(const ob::SpaceInformationPtr& si)
    : ob::StateValidityChecker(si) {};

    bool isValid(const ob::State* state) const override;
};

class PRMGraph : public og::PRM {
public:
    PRMGraph(const ob::SpaceInformationPtr& si,
             const Eigen::VectorXd& start,
             const Eigen::VectorXd& goal)
        : og::PRM(si), start_(start), goal_(goal) {}

    
    const Graph& getGraph() const { return this->g_; }
    Graph& getGraph() { return this->g_; }
    std::vector<int> getMilestoneLabels();
    void printLabeledSamples(const std::vector<int>& labels);

private:
    Eigen::VectorXd start_, goal_;
    Vertex findClosestVertex(const Eigen::VectorXd& q);
};


#endif