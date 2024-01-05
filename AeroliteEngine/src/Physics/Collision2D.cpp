#include <iostream>
#include "Collision2D.h"
#include "Shape.h"

namespace Aerolite
{
    bool CollisionDetection2D::IsColliding(Aerolite::Body2D* a, Aerolite::Body2D* b, Aerolite::Contact2D& contact)
    {   
        bool aIsCircle = a->shape->GetType() == Aerolite::ShapeType::Circle;
        bool bIsCircle = b->shape->GetType() == Aerolite::ShapeType::Circle;

        if(aIsCircle && bIsCircle)
        {
            bool result = IsCollidingCircleCircle(a, b, contact);
            return result;
        }

        return false;
    }

    bool CollisionDetection2D::IsCollidingCircleCircle(Aerolite::Body2D* a, Aerolite::Body2D* b, Aerolite::Contact2D& contact)
    {
        auto aCircleShape = std::dynamic_pointer_cast<Aerolite::CircleShape>(a->shape);
        auto bCircleShape = std::dynamic_pointer_cast<Aerolite::CircleShape>(b->shape);

        const Vec2 distanceBetweenCenters = b->position - a->position;
        const Aerolite::real sumRadius = aCircleShape->radius + bCircleShape->radius;
        auto magSquared = distanceBetweenCenters.MagnitudeSquared();
        auto radSquared = sumRadius * sumRadius;
        bool isColliding = magSquared <= radSquared;
        
        if(!isColliding) {
            return false;
        }

        contact.a = a;
        contact.b = b;
        contact.normal = distanceBetweenCenters.UnitVector();   
        contact.start = b->position - (contact.normal * bCircleShape->radius);
        contact.end = a->position + (contact.normal * aCircleShape->radius);
        contact.depth = (contact.end - contact.start).Magnitude();

        return true;
    }
}

