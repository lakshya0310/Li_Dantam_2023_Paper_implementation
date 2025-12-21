#include <gudhi/Coxeter_triangulation.h>
#include <gudhi/Functions/Function_Sm_in_Rd.h>
#include <gudhi/Implicit_manifold_intersection_oracle.h>
#include <gudhi/Manifold_tracing.h>
#include <gudhi/Coxeter_triangulation/Cell_complex/Cell_complex.h>
#include <fstream>
#include <iostream>
#include <map>
#include <vector>
#include <Eigen/Dense>

using namespace Gudhi::coxeter_triangulation;

int main() {
    double radius = 3.0;
    Function_Sm_in_Rd circle_function(radius, 1);
    auto oracle = make_oracle(circle_function);
    double lambda = 0.3;  // Mesh resolution (smaller = finer)
    Coxeter_triangulation<> cox_tr(oracle.amb_d());
    
    cox_tr.change_offset(Eigen::VectorXd::Random(oracle.amb_d()) * 0.1);
    
    // Scale the triangulation for desired resolution
    cox_tr.change_matrix(lambda * cox_tr.matrix());
    
    // Run manifold tracing algorithm
    using MT = Manifold_tracing<Coxeter_triangulation<>>;
    using Out_simplex_map = typename MT::Out_simplex_map;
    std::vector<Eigen::VectorXd> seed_points(1, oracle.seed());
    Out_simplex_map interior_simplex_map;
    
    std::cout << "Running manifold tracing algorithm..." << std::endl;
    manifold_tracing_algorithm(seed_points, cox_tr, oracle, interior_simplex_map);
    
    std::cout << "Found " << interior_simplex_map.size() << " intersecting simplices" << std::endl;
    
    // Build cell complex to get edges
    std::size_t intr_d = oracle.amb_d() - oracle.cod_d();
    Cell_complex<Out_simplex_map> cell_complex(intr_d);
    cell_complex.construct_complex(interior_simplex_map);
    
    // Extract vertices and their coordinates
    std::map<Cell_complex<Out_simplex_map>::Hasse_cell*, std::size_t> vertex_id_map;
    std::vector<Eigen::VectorXd> vertex_positions;
    std::size_t vertex_count = 0;
    
    std::cout << "Extracting vertices..." << std::endl;
    for (const auto& cell_point_pair : cell_complex.cell_point_map()) {
        vertex_id_map[cell_point_pair.first] = vertex_count++;
        vertex_positions.push_back(cell_point_pair.second);
    }
    
    std::cout << "Number of vertices: " << vertex_count << std::endl;
    
    // Write vertices to CSV
    std::ofstream vout("circle_vertices.csv");
    vout << "id,x,y\n";
    for (size_t i = 0; i < vertex_positions.size(); i++) {
        vout << i << "," 
             << vertex_positions[i](0) << "," 
             << vertex_positions[i](1) << "\n";
    }
    vout.close();
    std::cout << "Written circle_vertices.csv" << std::endl;
    
    // Extract edges on the circle (between intersection points)
    std::vector<std::pair<std::size_t, std::size_t>> circle_edges;
    
    std::cout << "Extracting circle edges..." << std::endl;
    for (const auto& simplex_cell_pair : cell_complex.interior_simplex_cell_map(1)) {
        Cell_complex<Out_simplex_map>::Hasse_cell* edge_cell = simplex_cell_pair.second;
        
        // Get the two vertices of this edge
        std::vector<std::size_t> edge_vertices;
        for (const auto& boundary_cell : edge_cell->get_boundary()) {
            auto it = vertex_id_map.find(boundary_cell.first);
            if (it != vertex_id_map.end()) {
                edge_vertices.push_back(it->second);
            }
        }
        
        if (edge_vertices.size() == 2) {
            circle_edges.push_back({edge_vertices[0], edge_vertices[1]});
        }
    }
    
    std::cout << "Number of circle edges: " << circle_edges.size() << std::endl;
    
    // Now extract the actual Coxeter triangulation edges that intersect the circle
    std::cout << "Extracting Coxeter triangulation edges..." << std::endl;
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> coxeter_edges;
    
    for (const auto& simplex_point_pair : interior_simplex_map) {
        const auto& simplex = simplex_point_pair.first;
        
        // Use Vertex_iterator to get the vertices of this edge
        using Simplex_handle = typename Out_simplex_map::key_type;
        Gudhi::coxeter_triangulation::Vertex_iterator<Simplex_handle> vi(simplex);
        
        std::vector<Eigen::VectorXd> vertices;
        
        // Iterate through vertices
        while (vi != Gudhi::coxeter_triangulation::Vertex_iterator<Simplex_handle>()) {
            Eigen::VectorXd cart_coord = cox_tr.cartesian_coordinates(*vi);
            vertices.push_back(cart_coord);
            ++vi;
        }
        
        // For an edge (1-simplex), we should have 2 vertices
        if (vertices.size() == 2) {
            coxeter_edges.push_back({vertices[0], vertices[1]});
        }
    }
    
    std::cout << "Number of Coxeter edges: " << coxeter_edges.size() << std::endl;
    
    // Write circle edges to CSV (edges along the circle)
    std::ofstream eout("circle_edges.csv");
    eout << "v0,v1\n";
    for (const auto& edge : circle_edges) {
        eout << edge.first << "," << edge.second << "\n";
    }
    eout.close();
    std::cout << "Written circle_edges.csv" << std::endl;
    
    // Write line segments on circle for easy plotting
    std::ofstream lout("circle_segments.csv");
    lout << "x0,y0,x1,y1\n";
    for (const auto& edge : circle_edges) {
        const auto& v0 = vertex_positions[edge.first];
        const auto& v1 = vertex_positions[edge.second];
        lout << v0(0) << "," << v0(1) << "," 
             << v1(0) << "," << v1(1) << "\n";
    }
    lout.close();
    std::cout << "Written circle_segments.csv" << std::endl;
    
    // Write Coxeter triangulation edges that intersect the circle
    std::ofstream cout_file("coxeter_edges.csv");
    cout_file << "x0,y0,x1,y1\n";
    for (const auto& edge : coxeter_edges) {
        cout_file << edge.first(0) << "," << edge.first(1) << "," 
                  << edge.second(0) << "," << edge.second(1) << "\n";
    }
    cout_file.close();
    std::cout << "Written coxeter_edges.csv" << std::endl;
    
    std::cout << "\nSummary:" << std::endl;
    std::cout << "  Circle radius: " << radius << std::endl;
    std::cout << "  Mesh resolution (lambda): " << lambda << std::endl;
    std::cout << "  Vertices on circle: " << vertex_count << std::endl;
    std::cout << "  Circle edge segments: " << circle_edges.size() << std::endl;
    std::cout << "  Coxeter edges intersecting circle: " << coxeter_edges.size() << std::endl;
    std::cout << "\nOutput files created:" << std::endl;
    std::cout << "  - circle_vertices.csv: intersection points on circle" << std::endl;
    std::cout << "  - circle_edges.csv: edge connectivity on circle" << std::endl;
    std::cout << "  - circle_segments.csv: line segments on circle (x0,y0,x1,y1)" << std::endl;
    std::cout << "  - coxeter_edges.csv: Coxeter triangulation edges intersecting circle (x0,y0,x1,y1)" << std::endl;
    
    return 0;
}