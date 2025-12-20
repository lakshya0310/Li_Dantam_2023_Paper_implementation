#include "ElasticUpdate.hpp"
#include <cmath>

std::vector<double> elasticUpdate(
    const std::vector<double>& p,
    SDFOracle& sdf,
    double a)
{
    double eps=1e-3;
    std::vector<double> g(2);
    for(int i=0;i<2;i++){
        auto p1=p,p2=p;
        p1[i]+=eps; p2[i]-=eps;
        g[i]=(sdf(p1)-sdf(p2))/(2*eps);
    }
    double f=sdf(p);
    double n=std::sqrt(g[0]*g[0]+g[1]*g[1])+1e-8;
    return {p[0]-a*f*g[0]/n, p[1]-a*f*g[1]/n};
}
