#include "Vec2.h"
#include <cmath>
#include <stdexcept>

namespace Aerolite {
     Vec2::Vec2(float x, float y) noexcept : x(x), y(y) {}

     constexpr void Vec2::Add(const Vec2& v) noexcept
    {
        x += v.x;
        y += v.y;
    }

     constexpr void Vec2::Sub(const Vec2& v) noexcept
    {
        x -= v.x;
        y -= v.y;
    }

     constexpr void Vec2::Scale(const float n) noexcept
    {
        x *= n;
        y *= n;
    }

     Vec2 Vec2::Rotate(const float angle) const noexcept
    {
        return Vec2(x * cosf(angle) - y * sinf(angle), x * sinf(angle) + y * cosf(angle));
    }

     float Vec2::Magnitude() const noexcept
    {
        return std::hypot(x, y);
    }

     constexpr float Vec2::MagnitudeSquared() const noexcept
    {
        return x * x + y * y;
    }

     Vec2& Vec2::Normalize() noexcept
    {
        float length = Magnitude();
        if (length != 0.0f)
        {
            x /= length;
            y /= length;
        }

        return *this;
    }

     Vec2 Vec2::UnitVector() const noexcept
    {
        Vec2 result = *this;
        return result.Normalize();
    }

     Vec2 Vec2::Normal() const noexcept
    {
        return Vec2(y, -x).Normalize();
    }

     constexpr float Vec2::Dot(const Vec2& v) const noexcept
    {
        return x * v.x + y * v.y;
    }

     constexpr float Vec2::Cross(const Vec2& v) const noexcept
    {
        return (x * v.y) - (y * v.x);
    }

     Vec2& Vec2::operator=(const Vec2& v) noexcept
    {
        x = v.x;
        y = v.y;
        return *this;
    }

     bool Vec2::operator==(const Vec2& v) const noexcept
    {
        return x == v.x && y == v.y;
    }

     bool Vec2::operator!=(const Vec2& v) const noexcept
    {
        return x != v.x || y != v.y;
    }

     Vec2 Vec2::operator+(const Vec2& v) const noexcept
    {
        return Vec2(x + v.x, y + v.y);
    }

     Vec2 Vec2::operator-(const Vec2& v) const noexcept
    {
        return Vec2(x - v.x, y - v.y);
    }

     Vec2 Vec2::operator*(const float n) const noexcept
    {
        return Vec2(x * n, y * n);
    }

     Vec2 Vec2::operator/(const float n) const
    {
        if (n == 0) throw std::runtime_error("Division by zero error in Vec2 operator /");

        return Vec2(x / n, y / n);
    }

     Vec2 Vec2::operator-() const noexcept
    {
        return Vec2(-x, -y);
    }

     Vec2& Vec2::operator+=(const Vec2& v) noexcept
    {
        Add(v);
        return *this;
    }

     Vec2& Vec2::operator-=(const Vec2& v) noexcept
    {
        Sub(v);
        return *this;
    }

     Vec2& Vec2::operator*=(const float n) noexcept
    {
        Scale(n);
        return *this;
    }

     Vec2& Vec2::operator/=(const float n)
    {
        if (n == 0) throw std::runtime_error("Division by zero error in Vec2 operator /=");

        x /= n;
        y /= n;
        return *this;
    }
}
