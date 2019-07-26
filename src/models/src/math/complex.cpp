#include <cmath>
#include <complex.hpp>

namespace lambda
{

complex::complex() : _real(0), _imag(0) { }

complex::complex(double real, double imag) :
    _real(real), _imag(imag) { }

complex::complex(const complex &c) :
    _real(c.real()), _imag(c.imag()) { }

complex& complex::operator = (complex c)
{
    _real = c.real();
    _imag = c.imag();
    return *this;
}

complex& complex::operator += (complex c)
{
    return *this = *this + c;
}

complex& complex::operator -= (complex c)
{
    return *this = *this - c;
}

complex& complex::operator *= (double scalar)
{
    return *this = *this * scalar;
}

complex& complex::operator *= (complex c)
{
    return *this = *this * c;
}

complex& complex::operator /= (double scalar)
{
    return *this = *this / scalar;
}

complex& complex::operator /= (complex c)
{
    return *this = *this / c;
}

double complex::real() const
{
    return _real;
}

double& complex::real()
{
    return _real;
}

double complex::imag() const
{
    return _imag;
}

double& complex::imag()
{
    return _imag;
}

complex operator + (complex c1, complex c2)
{
    return complex(c1.real() + c2.real(), c1.imag() + c2.imag());
}

complex operator - (complex c1, complex c2)
{
    return c1 + -c2;
}

complex operator * (complex c1, complex c2)
{
    return complex(c1.real()*c2.real() - c1.imag()*c2.imag(),
                   c1.real()*c2.imag() + c2.real()*c1.imag());
}

complex operator * (complex c, double scalar)
{
    return complex(c.real()*scalar, c.imag()*scalar);
}

complex operator / (complex c1, complex c2)
{
    return c1*conj(c2)/(c2*conj(c2)).real();
}

complex operator / (complex c, double scalar)
{
    return c * (1/scalar);
}

complex operator - (complex c)
{
    return complex(-c.real(), -c.imag());
}

double arg(complex c)
{
    return std::atan2(c.imag(), c.real());
}

double norm(complex c)
{
    return std::sqrt(std::pow(c.real(), 2) + std::pow(c.imag(), 2));
}

complex conj(complex c)
{
    return complex(c.real(), -c.imag());
}

} // namespace lambda
