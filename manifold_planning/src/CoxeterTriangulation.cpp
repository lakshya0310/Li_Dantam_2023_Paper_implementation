#include "CoxeterTriangulation.hpp"
#include <cmath>

CoxeterTriangulation::CoxeterTriangulation(double qmin, double qmax, double h) {
    int n = std::floor((qmax-qmin)/h)+1;

    for(int i=0;i<n;i++)
        for(int j=0;j<n;j++)
            vertices.push_back({qmin+i*h, qmin+j*h});

    auto idx=[&](int i,int j){ return i*n+j; };

    for(int i=0;i<n-1;i++)
        for(int j=0;j<n-1;j++){
            int v00=idx(i,j), v10=idx(i+1,j);
            int v01=idx(i,j+1), v11=idx(i+1,j+1);
            simplices.push_back({{v00,v10,v11}});
            simplices.push_back({{v00,v11,v01}});
        }
}
