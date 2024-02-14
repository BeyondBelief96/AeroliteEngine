#include <iostream>
#include <cmath>
#include <limits>
#include "Shape.h"

Aerolite::CircleShape::CircleShape(const real radius)
{
    this->radius = radius;
}

Aerolite::ShapeType Aerolite::CircleShape::GetType() const
{
    return Circle;
}

// For solid circles, the moment of inertia is 1/2 * r^2 * mass
// This still needs to be multiplied by the mass of the body containing
// this shape.
Aerolite::real Aerolite::CircleShape::GetMomentOfInertia() const
{
    // This still needs to be multiplied by the rigid bodies mass.
    // Only returns partial equation based on the geometric quantities.
    return 0.5f * (radius * radius);
}

void Aerolite::CircleShape::UpdateVertices(real angle, const AeroVec2& position)
{   
    return; // Circles don't have vertices, so nothing to do here.
}

Aerolite::BoxShape::BoxShape(const real width, const real height)
{
    this->width = width;
    this->height = height;

    // Initializing the local and world vertices too the same set. 
    this->localVertices.emplace_back(-width / 2.0, -height / 2.0);
    this->localVertices.emplace_back(width / 2.0, -height / 2.0);
    this->localVertices.emplace_back(width / 2.0, height / 2.0);
    this->localVertices.emplace_back(-width / 2.0, height / 2.0);

    this->worldVertices.emplace_back(-width / 2.0, -height / 2.0);
    this->worldVertices.emplace_back(width / 2.0, -height / 2.0);
    this->worldVertices.emplace_back(width / 2.0, height / 2.0);
    this->worldVertices.emplace_back(-width / 2.0, height / 2.0);
}

Aerolite::ShapeType Aerolite::BoxShape::GetType() const
{
    return Box;
}

// The moment of inertia of a rectangle is 1/12 * width^2 + height^2
// This still needs to be multiplied by the mass of body containing this shape.
Aerolite::real Aerolite::BoxShape::GetMomentOfInertia() const
{
    // Still needs to be multiplied by the mass. Shape's GetMomentOfInertia() function
    // returns only geometric dependent calculations.
    return (1.0f / 12.0f) * (width * width + height * height);
}

Aerolite::PolygonShape::PolygonShape(const std::vector<AeroVec2>& vertices)
{
    for (auto& vertex : vertices) {
        localVertices.emplace_back(vertex);
        worldVertices.emplace_back(vertex);
    }
}

Aerolite::ShapeType Aerolite::PolygonShape::GetType() const
{
    return Polygon;
}

Aerolite::real Aerolite::PolygonShape::GetMomentOfInertia() const
{
    real acc0 = 0;
    real acc1 = 0;
    for (int i = 0; i < localVertices.size(); i++) {
        auto a = localVertices[i];
        auto b = localVertices[(i + 1) % localVertices.size()];
        const auto cross = abs(a.Cross(b));
        acc0 += cross * (a.Dot(a) + b.Dot(b) + a.Dot(b));
        acc1 += cross;
    }
    return acc0 / 6 / acc1;
}

std::shared_ptr<Aerolite::PolygonShape> Aerolite::PolygonShape::CreateRegularPolygon(int sides, real sideLength) {
    if (sides < 3) {
        throw std::invalid_argument("Polygon must have at least 3 sides.");
    }

    std::vector<AeroVec2> vertices;
    const real centralAngle = 2 * M_PI / sides;
    const real radius = sideLength / (2 * std::sin(centralAngle / 2));

    for (int i = 0; i < sides; ++i) {
	    const real angle = i * centralAngle;
        real x = std::cos(angle) * radius;
        real y = std::sin(angle) * radius;
        vertices.emplace_back(x, y);
    }

    return std::make_shared<PolygonShape>(vertices);
}

Aerolite::AeroVec2 Aerolite::PolygonShape::EdgeAt(int index) const
{
	const int curVertex = index;
	const int nextVertex = (index + 1) % worldVertices.size();
    return (worldVertices[nextVertex] - worldVertices[curVertex]);
}

Aerolite::AeroVec2 Aerolite::PolygonShape::GeometricCenter(void) const
{
	real sumX = 0.0f;
	real sumY = 0.0f;

    for (const auto v : worldVertices)
    {
	    sumX += v.x;
        sumY += v.y;
    }

    return AeroVec2(sumX / static_cast<real>(worldVertices.size()),
                    sumY / static_cast<real>(worldVertices.size()));
}

int Aerolite::PolygonShape::FindIncidentEdgeIndex(const AeroVec2& referenceEdgeNormal) const
{
    int indexIncidentEdge = 0;
    real minProj = std::numeric_limits<real>::max();
    for (int i = 0; i < this->worldVertices.size(); i++)
    {
        auto edgeNormal = EdgeAt(i).Normal();
        const auto proj = edgeNormal.Dot(referenceEdgeNormal);
        if (proj < minProj) {
            minProj = proj;
            indexIncidentEdge = i;
        }
    }

    return indexIncidentEdge;
}

int Aerolite::PolygonShape::ClipLineSegmentToLine(const std::vector<AeroVec2>& contactsIn, std::vector<AeroVec2>& contactsOut, const AeroVec2& c0, const AeroVec2& c1)
{
    // Start with no output points.
    int numOut = 0;

    // Calculate the distance of end points to the line.
    const AeroVec2 normal = (c1 - c0).UnitVector();
    const real dist0 = (contactsIn[0] - c0).Cross(normal);
    const real dist1 = (contactsIn[1] - c0).Cross(normal);

    // If the points are behind the plane
    if (dist0 <= 0)
        contactsOut[numOut++] = contactsIn[0];
    if (dist1 <= 0)
        contactsOut[numOut++] = contactsIn[1];

    // If the points are on different sides of the plane (one distance is negative and other is positive)
    if (dist0 * dist1 < 0) {
	    const real totalDist = dist0 - dist1;

        // Find the intersection using linear interpolation
        real t = dist0 / (totalDist);
	    const AeroVec2 contact = contactsIn[0] + (contactsIn[1] - contactsIn[0]) * t;
        contactsOut[numOut] = contact;
        numOut++;
    }

    return numOut;
}

Aerolite::real Aerolite::PolygonShape::FindMinimumSeparation(const PolygonShape& other, int& indexReferenceEdge, AeroVec2& supportPoint) const
{
	real separation = std::numeric_limits<real>::lowest();

    for (int i = 0; i < worldVertices.size(); i++)
    {
	    AeroVec2 va = worldVertices[i];
	    AeroVec2 normal = EdgeAt(i).Normal().UnitVector();

	    real minSep = std::numeric_limits <real>::max();
        AeroVec2 minVertex;
        for (auto vb : other.worldVertices)
        {
	        const real projection = (vb - va).Dot(normal);
            if (projection < minSep) {
                minSep = projection;
                minVertex = vb;
            }
        }

        if (minSep > separation)
        {
            separation = minSep;
            indexReferenceEdge = i;
            supportPoint = minVertex;
        }

        if (separation > 0) return separation;
    }

    return separation;
}

void Aerolite::PolygonShape::UpdateVertices(const real angle, const AeroVec2& position)
{
    // Rotate first, then translate.
    for (int i = 0; i < localVertices.size(); i++)
    {
        worldVertices[i] = localVertices[i].Rotate(angle);
        worldVertices[i] += position;
    }
}
