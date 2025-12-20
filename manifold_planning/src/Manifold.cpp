#include "Manifold.hpp"

void Manifold::extract(const CoxeterTriangulation& ctr, SDFOracle& sdf){
    for(auto& s:ctr.simplices){
        int v[3]={s.v[0],s.v[1],s.v[2]};
        for(int i=0;i<3;i++){
            double f1=sdf(ctr.vertices[v[i]]);
            double f2=sdf(ctr.vertices[v[(i+1)%3]]);
            if(f1*f2<0)
                edges.push_back({v[i],v[(i+1)%3]});
        }
    }
}
