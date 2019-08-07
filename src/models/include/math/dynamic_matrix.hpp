#ifndef LAMBDA_DYNAMIC_MATRIX_HPP
#define LAMBDA_DYNAMIC_MATRIX_HPP

#include <array>
#include <vector>
#include <cstdint>
#include <iostream>
#include <iomanip>
#include <sstream>

/*!
    \file
    \brief Defines lambda::dynamic_matrix, as well as supporting operations for
    basic matrix arithmetic.
*/

/// \brief The namespace used by all lambda classes and functions.
namespace lambda
{

  /// \brief A dynamically sized matrix class. Supports basic operations like
  /// addition, multiplication. Is default-constructable and copy-constructable.
class dynamic_matrix
{
    public:

    /// \brief Copy constructor.
    dynamic_matrix(const dynamic_matrix &other);

    /// \brief Construct with dimensions only.
    dynamic_matrix(size_t rows, size_t columns);

    /// \brief Construct with dimensions and contents.
    dynamic_matrix(size_t rows, size_t columns,
                   const std::vector<double> &data);

    /// \brief Assignment operator.
    dynamic_matrix& operator = (const dynamic_matrix &m);

    /// \brief Get the number of rows in this matrix.
    size_t rows() const;

    /// \brief Get the number of columns in this matrix.
    size_t columns() const;

    /// \brief Access an element in row i, column j.
    double operator () (size_t i, size_t j) const;

    /// \brief Get a reference to the element (i, j).
    double& operator () (size_t i, size_t j);

    /// \brief Get the element at index i, interpreting the matrix
    /// as a 1-dimensional array in a row-major fashion.
    double operator [] (size_t i) const;

    /// \brief Get a reference to the element at i, in the
    /// equivalent 1-dimensional row-major array.
    double& operator [] (size_t i);

    /// \brief Same as operator [], but throws an exception if
    /// the index provided is out of bounds.
    double at(size_t i) const;

    /// \brief Same as operator [], but throws an exception if
    /// the index provided is out of bounds.
    double& at(size_t i);

    /// \brief Same as operator (), but throws an exception if
    /// the index provided is out of bounds.
    double at(size_t i, size_t j) const;

    /// \brief Same as operator (), but throws an exception if
    /// the index provided is out of bounds.
    double& at(size_t i, size_t j);

    /// \brief Get a const ref to the underlying data of this matrix.
    const std::vector<double>& data() const;

    private:

    /// \brief Matrix elements stored here.
    std::vector<double> _data;
    size_t _rows, _columns;

    /// \brief Throws an exception if an index is out of bounds.
    void range_check(size_t i) const;

    /// \brief Throws an exception if an index is out of bounds.
    void range_check(size_t i, size_t j) const;
};

bool operator == (const dynamic_matrix &left,
                  const dynamic_matrix &right);

/// \brief X basis vector.
const static dynamic_matrix dunitx(3, 1, {1, 0, 0});
/// \brief Y basis vector.
const static dynamic_matrix dunity(3, 1, {0, 1, 0});
/// \brief Z basis vector.
const static dynamic_matrix dunitz(3, 1, {0, 0, 1});

/// \brief Print a matrix to a std::ostream.
std::ostream& operator << (std::ostream &os, const dynamic_matrix &m);

/// \brief Produces a multiline string representation of a matrix.
std::string pretty(const dynamic_matrix &m);

/// \brief Multiplication of a matrix by a scalar.
template <class T>
dynamic_matrix operator * (const dynamic_matrix &m, T scalar)
{
    auto ret = m;
    for (size_t i = 0; i < m.rows(); ++i)
    {
        for (size_t j = 0; j < m.columns(); ++j)
        {
            ret(i, j) *= scalar;
        }
    }
    return ret;
}

/// \brief Multiplication of a matrix by a scalar.
template <class T>
dynamic_matrix operator * (T scalar, const dynamic_matrix &m)
{
    return m*scalar;
}

/// \brief Division of a matrix by a scalar.
template <class T>
dynamic_matrix operator / (const dynamic_matrix &m, T divisor)
{
    return m*(1.0/divisor);
}

/// \brief Addition of two matrices.
dynamic_matrix operator + (const dynamic_matrix &left,
                           const dynamic_matrix &right);

/// \brief Multiplication of two matrices.
dynamic_matrix operator * (const dynamic_matrix &left,
                           const dynamic_matrix &right);

} // namespace lambda

#endif // LAMBDA_MATRIX_HPP
