#ifndef VICKTOR_HPP
#define VICKTOR_HPP
/*struct Vicktor
{
    double x{};
    double y{};

    Vicktor sum(Vicktor p, Vicktor q);

    Vicktor subtract(Vicktor p, Vicktor q);

    Vicktor scalar_multi(Vicktor p, double n);

    double module(Vicktor p);
};*/
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

//inline Vicktor operator*(double n, const Vicktor &v);


#endif