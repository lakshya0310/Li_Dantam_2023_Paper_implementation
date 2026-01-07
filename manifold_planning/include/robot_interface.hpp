#pragma once
#include <vector>
#include <string>

/**
 * Abstract interface so we can:
 *  - swap PyBullet
 *  - swap FCL
 *  - test analytically
 */
class RobotInterface {
public:
    virtual ~RobotInterface() = default;

    /// Signed distance function (positive = free, negative = collision)
    virtual double sdf(const std::vector<double>& q) = 0;

    /// Binary collision check
    virtual bool inCollision(const std::vector<double>& q) = 0;

    /// Configuration dimension
    virtual size_t dof() const = 0;
};
