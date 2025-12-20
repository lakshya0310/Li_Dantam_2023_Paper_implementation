#include "Graph.hpp"
#include <cmath>

std::vector<std::vector<int>>
buildGraph(const std::vector<std::vector<double>>& pts,int k){
    int n=pts.size();
    std::vector<std::vector<int>> g(n);
    for(int i=0;i<n;i++)
        for(int j=0;j<n;j++){
            if(i==j) continue;
            double d=std::hypot(pts[i][0]-pts[j][0],
                                pts[i][1]-pts[j][1]);
            if(d<0.3) g[i].push_back(j);
        }
    return g;
}
