#ifndef AERO_BODY_MANAGER_H
#define AERO_BODY_MANAGER_H
#include "AeroBody2D.h"

namespace Aerolite
{
    class BodyManager {
    public:
        BodyManager() = default;
        ~BodyManager() = default;

        std::shared_ptr<AeroBody2D> CreateBody(const std::shared_ptr<Shape>& shape, float x, float y, float mass) {
            auto body = std::make_shared<AeroBody2D>(shape, x, y, mass);
            bodies.push_back(body);
            return body;
        }

        // Get a const reference to the vector of bodies
        const std::vector<std::shared_ptr<AeroBody2D>>& GetBodies() const {
            return bodies;
        }

        // Get a non-const reference to the vector of bodies
        std::vector<std::shared_ptr<AeroBody2D>>& GetBodies() {
            return bodies;
        }

    private:
        std::vector<std::shared_ptr<AeroBody2D>> bodies;
    };
}


#endif
