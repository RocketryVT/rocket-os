#ifndef LAMBDA_COMPLEX_HPP
#define LAMBDA_COMPLEX_HPP

/*! 
    \file
    \brief Defines lambda::complex and supporting operations
           for complex arithmetic.
*/

namespace lambda
{

/// \brief A complex number class. Supports complex arithmetic
/// like multiplication, division, conjugation.
class complex
{
    public:

    /// \brief Default constructor.
    complex();

    /// \brief Construct with a real and imaginary component.
    complex(double real, double imag);

    /// \brief Copy-constructor.
    complex(const complex &c);

    /// \brief Assignment operator.
    complex& operator = (complex c);

    /// \brief Add another complex number to this one.
    complex& operator += (complex c);

    /// \brief Subtract another complex number from this one.
    complex& operator -= (complex c);

    /// \brief Multiply this complex number by a scalar.
    complex& operator *= (double scalar);

    /// \brief Multiply this complex number by another one.
    complex& operator *= (complex c);

    /// \brief Divide this complex number by a scalar.
    complex& operator /= (double scalar);

    /// \brief Divide this complex number by another one.
    complex& operator /= (complex divisor);

    /// \brief Get the real component of this complex number.
    double real() const;

    /// \brief Get a reference to the real component.
    double& real();

    /// \brief Get the imaginary component of this complex number.
    double imag() const;

    /// \brief Get a reference to the imaginary number.
    double& imag();

    private:

    double _real;
    double _imag;
};

/// \brief Add two complex numbers.
complex operator + (complex c1, complex c2);

/// \brief Subtract two complex numbers.
complex operator - (complex c1, complex c2);

/// \brief multiply two complex numbers.
complex operator * (complex c1, complex c2);

/// \brief Multiply a complex number by a scalar.
complex operator * (complex c, double scalar);

/// \brief Divide a complex number by another one.
complex operator / (complex c1, complex c2);

/// \brief Divide a complex number by a scalar.
complex operator / (complex c, double scalar);

/// \brief Negate a complex number.
complex operator - (complex c);

/// \brief Get the argument of a complex number.
double arg(complex c);

/// \brief Get the norm of a complex number.
double norm(complex c);

/// \brief Get the conjugate of a complex number.
complex conj(complex c);

} // namespace lambda

#endif // LAMBDA_COMPLEX_HPP
