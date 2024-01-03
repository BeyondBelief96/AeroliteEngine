#include <iostream>
#include "Shape.h"

Aerolite::CircleShape::CircleShape(const real radius)
{
    this->radius = radius;
    std::cout << "CircleShape constructor called!" << std::endl;
}

Aerolite::CircleShape::~CircleShape()
{
    std::cout << "CircleShape destructor called!" << std::endl;
}

Aerolite::ShapeType Aerolite::CircleShape::GetType() const
{
    return ShapeType::Circle;
}

// For solid circles, the moment of inertia is 1/2 * r^2 * mass
// This still needs to be multiplied by the mass of the body containing
// this shape.
Aerolite::real Aerolite::CircleShape::GetMomentOfInertia() const
{
    // This still needs to be multiplied by the rigid bodies mass.
    // Only returns partial equation based on the geometric quantities.
    return (0.5) * (radius * radius);
}

Aerolite::PolygonShape::PolygonShape(const std::vector<Vec2> &vertices)
{
    this->localVertices = vertices;
    this->worldVertices = std::vector<Vec2>();
    std::cout << "PolygonShape constructor called!" << std::endl;
}

Aerolite::PolygonShape::~PolygonShape()
{
    std::cout << "PolygonShape destructor called!" << std::endl;
}

Aerolite::ShapeType Aerolite::PolygonShape::GetType() const
{
    return ShapeType::Polygon;
}

Aerolite::real Aerolite::PolygonShape::GetMomentOfInertia() const
{
    return 0.0f;
}

void Aerolite::PolygonShape::UpdateVertices(const real angle, const Vec2 &position)
{   
    // Rotate first, then translate.
    for(int i = 0; i < localVertices.size(); i++)
    {
        worldVertices[i] = localVertices[i].Rotate(angle);
        worldVertices[i] += position;
    }
}

Aerolite::BoxShape::BoxShape(const real width, const real height)
{
    this->width = width;
    this->height = height;

    // Initializing the local and world vertices too the same set. 
    this->localVertices.push_back(Vec2(-width / 2.0, -height / 2.0));
    this->localVertices.push_back(Vec2(width / 2.0, -height / 2.0));
    this->localVertices.push_back(Vec2(width / 2.0, height / 2.0));
    this->localVertices.push_back(Vec2(-width / 2.0, height / 2.0));

    this->worldVertices.push_back(Vec2(-width / 2.0, -height / 2.0));
    this->worldVertices.push_back(Vec2(width / 2.0, -height / 2.0));
    this->worldVertices.push_back(Vec2(width / 2.0, height / 2.0));
    this->worldVertices.push_back(Vec2(-width / 2.0, height / 2.0));

    std::cout << "BoxShape constructor called" << std::endl;
}

Aerolite::BoxShape::~BoxShape()
{
    std::cout << "BoxShape destructor called" << std::endl;
}

Aerolite::ShapeType Aerolite::BoxShape::GetType() const
{
    return Aerolite::ShapeType::Box;
}

// The moment of inertia of a rectangle is 1/12 * width^2 + height^2
// This still needs to be multiplied by the mass of body containing this shape.
Aerolite::real Aerolite::BoxShape::GetMomentOfInertia() const
{
    // Still needs to be multiplied by the mass. Shape's GetMomentOfInertia() function
    // returns only geometric dependent calculations.
    return (1.0f / 12.0f) * (width * width + height * height);
}
