#include <iostream>
#include <cmath>
#include <limits>
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

void Aerolite::CircleShape::UpdateVertices(Aerolite::real angle, const Aerolite::Vec2& position)
{   
    return; // Circles don't have vertices, so nothing to do here.
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

Aerolite::PolygonShape::PolygonShape(const std::vector<Vec2>& vertices)
{
    for (auto& vertex : vertices) {
        localVertices.emplace_back(vertex);
        worldVertices.emplace_back(vertex);
    }
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
    return 5000;
}

Aerolite::PolygonShape* Aerolite::PolygonShape::CreateRegularPolygon(int sides, real sideLength) {
    if (sides < 3) {
        throw std::invalid_argument("Polygon must have at least 3 sides.");
    }

    std::vector<Vec2> vertices;
    const real centralAngle = 2 * M_PI / sides;
    const real radius = sideLength / (2 * std::sin(centralAngle / 2));

    for (int i = 0; i < sides; ++i) {
        real angle = i * centralAngle;
        real x = std::cos(angle) * radius;
        real y = std::sin(angle) * radius;
        vertices.emplace_back(x, y);
    }

    return new PolygonShape(vertices);
}

Aerolite::Vec2 Aerolite::PolygonShape::EdgeAt(int index) const
{
    int curVertex = index;
    int nextVertex = (index + 1) % worldVertices.size();
    return (worldVertices[nextVertex] - worldVertices[curVertex]);
}

Aerolite::Vec2 Aerolite::PolygonShape::GeometricCenter(void) const
{
    Aerolite::real sumX = 0.0f;
    Aerolite::real sumY = 0.0f;

    for (int i = 0; i < worldVertices.size(); i++)
    {
        Aerolite::Vec2 v = worldVertices[i];
        sumX += v.x;
        sumY += v.y;
    }

    return Aerolite::Vec2(sumX / (Aerolite::real)worldVertices.size(),
        sumY / (Aerolite::real)worldVertices.size());
}

Aerolite::real Aerolite::PolygonShape::FindMinimumSeparation(const Aerolite::PolygonShape& other, Vec2& edgeOfSeparation, Vec2& point) const
{
    Aerolite::real separation = std::numeric_limits<Aerolite::real>::lowest();

    for (int i = 0; i < worldVertices.size(); i++)
    {
        Aerolite::Vec2 va = worldVertices[i];
        Aerolite::Vec2 normal = EdgeAt(i).Normal().UnitVector();

        Aerolite::real minSep = std::numeric_limits < Aerolite::real>::max();
        Vec2 minVertex;
        for (int j = 0; j < other.worldVertices.size(); j++)
        {
            Aerolite::Vec2 vb = other.worldVertices[j];
            Aerolite::real projection = (vb - va).Dot(normal);
            if (projection < minSep) {
                minSep = projection;
                minVertex = vb;
            }
        }

        if (minSep > separation)
        {
            separation = minSep;
            edgeOfSeparation = EdgeAt(i);
            point = minVertex;
        }

        if (separation > 0) return separation;
    }

    return separation;
}

void Aerolite::PolygonShape::UpdateVertices(const real angle, const Vec2& position)
{
    // Rotate first, then translate.
    for (int i = 0; i < localVertices.size(); i++)
    {
        worldVertices[i] = localVertices[i].Rotate(angle);
        worldVertices[i] += position;
    }
}
