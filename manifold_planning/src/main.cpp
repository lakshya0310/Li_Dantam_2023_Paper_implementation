#include "PyBulletSDF.hpp"
#include "CoxeterTriangulation.hpp"
#include "Manifold.hpp"
#include "ElasticUpdate.hpp"
#include "Graph.hpp"
#include "AStar.hpp"
#include "CSVWriter.hpp"
#include <cmath>

int main(){
    PyBulletSDF sdf("python3 pybullet/sdf_server.py");

    CoxeterTriangulation ctr(-M_PI,M_PI,0.25);

    Manifold m;
    m.extract(ctr,sdf);

    std::vector<std::vector<double>> pts;
    for(auto&e:m.edges){
        auto p=ctr.vertices[e.a];
        for(int i=0;i<10;i++) p=elasticUpdate(p,sdf);
        pts.push_back(p);
    }

    auto graph=buildGraph(pts);
    int s=0, g=pts.size()-1;
    auto idx=astar(graph,pts,s,g);

    std::vector<std::vector<double>> path;
    for(int i:idx) path.push_back(pts[i]);

    writeCSV("data/path.csv",path);
}
