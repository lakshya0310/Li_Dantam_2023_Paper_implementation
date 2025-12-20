#pragma once
#include <fstream>
#include <vector>

inline void writeCSV(
    const char* f,
    const std::vector<std::vector<double>>& p)
{
    std::ofstream o(f);
    o<<"q1,q2\n";
    for(auto& x:p) o<<x[0]<<","<<x[1]<<"\n";
}
