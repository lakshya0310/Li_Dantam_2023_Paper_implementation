#include "AStar.hpp"
#include <queue>
#include <cmath>
#include <unordered_map>

std::vector<int> astar(
    const std::vector<std::vector<int>>& g,
    const std::vector<std::vector<double>>& p,
    int s,int t)
{
    using P=std::pair<double,int>;
    std::priority_queue<P,std::vector<P>,std::greater<P>> pq;
    pq.push({0,s});
    std::unordered_map<int,int> parent;
    std::unordered_map<int,double> dist;
    dist[s]=0;

    auto h=[&](int i){
        return std::hypot(p[i][0]-p[t][0],p[i][1]-p[t][1]);
    };

    while(!pq.empty()){
        auto [_,u]=pq.top(); pq.pop();
        if(u==t) break;
        for(int v:g[u]){
            double nd=dist[u]+h(v);
            if(!dist.count(v)||nd<dist[v]){
                dist[v]=nd;
                parent[v]=u;
                pq.push({nd+h(v),v});
            }
        }
    }

    std::vector<int> path;
    for(int v=t;parent.count(v);v=parent[v]) path.push_back(v);
    path.push_back(s);
    return path;
}
