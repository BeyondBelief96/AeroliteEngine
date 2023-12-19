#include <math.h>
#include <stdexcept>
#include "Vec2.h"

Vec2::Vec2() : x(0.0), y(0.0) {}

Vec2::Vec2(float x, float y): x(x), y(y) {}

void Vec2::Add(const Vec2& v)
{
	x += v.x;
	y += v.y;
}

void Vec2::Sub(const Vec2& v)
{
	x -= v.x;
	y -= v.y;
}

void Vec2::Scale(const float n)
{
	x *= n;
	y *= n;
}

Vec2 Vec2::Rotate(const float angle) const
{
	Vec2 result;
	result.x = x * cos(angle) - y * sin(angle);
	result.y = x * sin(angle) + y * cos(angle);
	return result;
}

float Vec2::Magnitude() const
{
	return sqrt(x * x + y * y);
}

float Vec2::MagnitudeSquared() const
{
	return x * x + y * y;
}

Vec2& Vec2::Normalize()
{
	float length = Magnitude();
	if (length != 0.0f)
	{
		x /= length;
		y /= length;
	}

	return *this;
}

Vec2 Vec2::UnitVector() const
{
	Vec2 result = Vec2{ 0, 0 };
	float length = Magnitude();
	if (length != 0.0f)
	{
		result.x /= length;
		result.y /= length;
	}
	return result;
}

Vec2 Vec2::Normal() const
{
	return Vec2(y, -x).Normalize();
}

float Vec2::Dot(const Vec2& v) const
{
	return x * v.x + y * v.y;
}

float Vec2::Cross(const Vec2& v) const
{
	return (x * v.y) - (y * v.x);
}

Vec2& Vec2::operator=(const Vec2& v)
{
	x = v.x;
	y = v.y;
	return *this;
}

bool Vec2::operator==(const Vec2& v) const
{
	return v.x == x && v.y == y;
}

bool Vec2::operator!=(const Vec2& v) const
{
	return v.x != x || v.y != y;
}

Vec2 Vec2::operator+(const Vec2& v) const
{
	return Vec2(x + v.x, y + v.y);
}

Vec2 Vec2::operator-(const Vec2& v) const
{
	return Vec2(x - v.x, y - v.y);
}

Vec2 Vec2::operator*(const float n) const
{
	return Vec2(x * n, y * n);
}

Vec2 Vec2::operator/(const float n) const
{
	if (n == 0) throw std::runtime_error("Division by zero error in Vec2 operator /");

	return Vec2(x / n, y / n);
}

Vec2 Vec2::operator-()
{
	return Vec2(-x, -y);
}

Vec2& Vec2::operator+=(const Vec2& v)
{
	x += v.x;
	y += v.y;
	return *this;
}

Vec2& Vec2::operator-=(const Vec2& v)
{
	x -= v.x;
	y -= v.y;
	return *this;
}

Vec2& Vec2::operator*=(const float n)
{
	x *= n;
	y *= n;
	return *this;
}

Vec2& Vec2::operator/=(const float n)
{
	x /= n;
	y /= n;
	return *this;
}
