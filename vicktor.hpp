#ifndef VICKTOR_HPP
#define VICKTOR_HPP

namespace nb{
struct Vicktor
{
    double x{};
    double y{};

    Vicktor operator+(const Vicktor &q) const; 
    Vicktor operator-(const Vicktor &q) const; 
    Vicktor operator*(double n) const;
    Vicktor &operator+=(const Vicktor &q);
    double module() const;
};
}
#endif