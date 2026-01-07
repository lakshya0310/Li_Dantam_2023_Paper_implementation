#pragma once
#include <vector>
#include <string>

struct Face {
    int a, b, c;
};

class Visualization {
public:
    static void saveOBJ(
        const std::vector<std::vector<double>>& vertices,
        const std::vector<Face>& faces,
        const std::string& filename);

    static void savePointCloud(
        const std::vector<std::vector<double>>& points,
        const std::string& filename);
};
