#include "Vec2.h"
#include <cmath>
#include <stdexcept>

namespace Aerolite {
     Vec2::Vec2(float x, float y) noexcept : x(x), y(y) {}

    Vec2 Vec2::Rotate(const float angle) const noexcept
    {
        return Vec2(x * cosf(angle) - y * sinf(angle), x * sinf(angle) + y * cosf(angle));
    }

    float Vec2::Magnitude() const noexcept
    {
        return std::hypot(x, y);
    }

    void Vec2::Normalize() noexcept
    {
        float length = Magnitude();
        if (length != 0.0f)
        {
            this->x /= length;
            this->y /= length;
        }
    }

    Vec2 Vec2::UnitVector() const noexcept
    {
        Vec2 result = Vec2(*this);
        result.Normalize();
        return result;
    }

    Vec2 Vec2::Normal() const noexcept
    {
        Vec2 result = Vec2(-y, x);
        result.Normalize();
        return result;
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
