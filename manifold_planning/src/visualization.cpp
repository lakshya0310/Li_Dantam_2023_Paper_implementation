#include "visualization.hpp"
#include <fstream>

void Visualization::saveOBJ(
    const std::vector<std::vector<double>>& vertices,
    const std::vector<Face>& faces,
    const std::string& filename) {

    std::ofstream out(filename);
    for (const auto& v : vertices)
        out << "v " << v[0] << " " << v[1] << " " << v[2] << "\n";

    for (const auto& f : faces)
        out << "f " << f.a+1 << " " << f.b+1 << " " << f.c+1 << "\n";
}

void Visualization::savePointCloud(
    const std::vector<std::vector<double>>& points,
    const std::string& filename) {

    std::ofstream out(filename);
    for (const auto& p : points)
        out << p[0] << " " << p[1] << " " << p[2] << "\n";
}
