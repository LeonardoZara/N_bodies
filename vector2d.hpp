#ifndef Vector2d_HPP
#define Vector2d_HPP

namespace nb{
struct Vector2d
{
    double x{};
    double y{};

    Vector2d operator+(const Vector2d &q) const; 
    Vector2d operator-(const Vector2d &q) const; 
    Vector2d operator*(double n) const;
    Vector2d &operator+=(const Vector2d &q);
    double module() const;
};
}
#endif