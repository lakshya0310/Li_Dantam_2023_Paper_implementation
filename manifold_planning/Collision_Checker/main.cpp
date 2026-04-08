#include <fcl/fcl.h>
#include <fcl/geometry/shape/box.h>
#include <fcl/geometry/shape/sphere.h>
#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_object.h>
#include <iostream>
#include <memory>

using namespace fcl;

class CollisionChecker {
public:
    std::shared_ptr<CollisionObjectd> createSphere(double radius, double tx, double ty, double tz) {
        auto sphere_geom = std::make_shared<Sphered>(radius);
        
        Transform3d tf = Transform3d::Identity();
        tf.translation() = Eigen::Vector3d(tx, ty, tz);
        
        return std::make_shared<CollisionObjectd>(sphere_geom, tf);
    }

    std::shared_ptr<CollisionObjectd> createBox(double dx, double dy, double dz, double tx, double ty, double tz) {
        auto box_geom = std::make_shared<Boxd>(dx, dy, dz);
        
        Transform3d tf = Transform3d::Identity();
        tf.translation() = Eigen::Vector3d(tx, ty, tz);
        
        return std::make_shared<CollisionObjectd>(box_geom, tf);
    }

    // Check collision between two generic collision objects
    bool checkCollision(const std::shared_ptr<CollisionObjectd>& obj1, 
                        const std::shared_ptr<CollisionObjectd>& obj2) {
        CollisionRequestd request;
        CollisionResultd result;
        
        // Perform the narrow-phase collision check
        collide(obj1.get(), obj2.get(), request, result);
        
        return result.isCollision();
    }
};

int main() {
    CollisionChecker checker;
    auto box = checker.createBox(1.0, 1.0, 1.0, 0.0, 0.0, 0.0);
    auto close_sphere = checker.createSphere(0.5, 0.8, 0.0, 0.0);
    auto far_sphere = checker.createSphere(0.5, 2.0, 0.0, 0.0);

    std::cout << "--- FCL Collision Test ---\n";
    
    bool collision1 = checker.checkCollision(box, close_sphere);
    std::cout << "Test 1 (Box and close Sphere): " 
              << (collision1 ? "COLLISION DETECTED" : "NO COLLISION") << "\n";

    bool collision2 = checker.checkCollision(box, far_sphere);
    std::cout << "Test 2 (Box and far Sphere):   " 
              << (collision2 ? "COLLISION DETECTED" : "NO COLLISION") << "\n";

    return 0;
}