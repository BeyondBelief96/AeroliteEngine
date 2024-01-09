#include <cmath>
#include <stdexcept>
#include "Vec2.h"
#include "Precision.h"


namespace Aerolite {
     Vec2::Vec2(Aerolite::real x, Aerolite::real y) noexcept : x(x), y(y) {}

    Vec2 Vec2::Rotate(const Aerolite::real angle) const noexcept
    {
        return Vec2(x * cosf(angle) - y * sinf(angle), x * sinf(angle) + y * cosf(angle));
    }

    Aerolite::real Vec2::Magnitude() const noexcept
    {
        return std::hypot(x, y);
    }

    void Vec2::Normalize() noexcept
    {
        Aerolite::real length = Magnitude();
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

    Vec2 Vec2::operator*(const Aerolite::real n) const noexcept
    {
        return Vec2(x * n, y * n);
    }

    Vec2 Vec2::operator/(const Aerolite::real n) const
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

    Vec2& Vec2::operator*=(const Aerolite::real n) noexcept
    {
        Scale(n);
        return *this;
    }

    Vec2& Vec2::operator/=(const Aerolite::real n)
    {
        if (n == 0) throw std::runtime_error("Division by zero error in Vec2 operator /=");

        x /= n;
        y /= n;
        return *this;
    }
}
