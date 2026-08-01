#ifndef VICKTOR_HPP
#define VICKTOR_HPP
struct Vicktor
{
    double x{};
    double y{};

    Vicktor sum(Vicktor p, Vicktor q);

    Vicktor subtract(Vicktor p, Vicktor q);

    Vicktor scalar_multi(Vicktor p, double n);

    double module(Vicktor p);

};
#endif