#ifndef LAMBDA_MATRIX_HPP
#define LAMBDA_MATRIX_HPP

#include <array>
#include <vector>
#include <cstdint>
#include <iostream>
#include <iomanip>
#include <sstream>

/*! 
    \file
    \brief Defines lambda::matrix, as well as supporting operations for
    basic matrix arithmetic.
*/

/// \brief The namespace used by all lambda classes and functions.
namespace lambda
{

/// \brief Forward declaration of lambda::quaternion.
class quaternion;
/// \brief Forward declaratuin of lambda::axis_angle.
class axis_angle;

/// \brief A statically sized matrix class. Supports basic operations like
/// addition, multiplication. Is default-constructable, copy-constructable,
/// and is constructable from quaternions and axis-angles.
template <size_t M, size_t N> class matrix
{
    static_assert(M > 0 && N > 0,
        "Cannot create matrix of dimension 0");

    public:

    /// \brief Default constructor.
    matrix() : _data() { }

    /// \brief Copy constructor.
    matrix(const matrix<M, N> &other) : _data(other.data()) { }

    /// \brief Construct a MxN matrix with a list of M*N elements.
    /// Elements are listed in row-major order.
    template <typename ...T, typename std::enable_if<
        sizeof...(T) == M*N, int>::type = 0>
    matrix(T... args) : _data({static_cast<double>(args)...}) { }

    /// \brief Construct a matrix from a quaternion. Only unit quaternions
    /// are guaranteed to produce valid rotation matrices.
    matrix(const quaternion &q);

    /// \brief Construct a rotation matrix from an axis-angle.
    matrix(const axis_angle &aa);

    /// \brief Assignment operator.
    matrix<M, N>& operator = (const matrix<M, N> &m)
    {
        for (size_t i = 0; i < M*N; ++i)
        {
            _data[i] = m[i];
        }
    }

    /// \brief Access an element in row i, column j.
    double operator () (size_t i, size_t j) const
    {
        return _data[N*i + j];
    }

    /// \brief Get a reference to the element (i, j).
    double& operator () (size_t i, size_t j)
    {
        return _data[N*i + j];
    }

    /// \brief Get the element at index i, interpreting the matrix
    /// as a 1-dimensional array in a row-major fashion.
    double operator [] (size_t i) const
    {
        return _data[i];
    }

    /// \brief Get a reference to the element at i, in the
    /// equivalent 1-dimensional row-major array.
    double& operator [] (size_t i)
    {
        return _data[i];
    }

    /// \brief Same as operator [], but throws an exception if
    /// the index provided is out of bounds.
    double at(size_t i) const
    {
        range_check(i);
        return _data[i];
    }

    /// \brief Same as operator [], but throws an exception if
    /// the index provided is out of bounds.
    double& at(size_t i)
    {
        range_check(i);
        return _data[i];
    }

    /// \brief Same as operator (), but throws an exception if
    /// the index provided is out of bounds.
    double at(size_t i, size_t j) const
    {
        range_check(i, j);
        return _data[N*i + j];
    }

    /// \brief Same as operator (), but throws an exception if
    /// the index provided is out of bounds.
    double& at(size_t i, size_t j)
    {
        range_check(i, j);
        return _data[N*i + j];
    }

    /// \brief Get a const ref to the underlying data of this matrix.
    const std::array<double, M*N>& data() const
    {
        return _data;
    }

    private:

    /// \brief Matrix elements stored here.
    std::array<double, M*N> _data;

    /// \brief Throws an exception if an index is out of bounds.
    static void range_check(size_t i)
    {
        if (i >= N*M)
        {
            std::stringstream ss;
            ss << "Cannot access element (" << i
                << ") of " << M << "x" << N << "matrix";
            throw std::out_of_range(ss.str());
        }
    }

    /// \brief Throws an exception if an index is out of bounds.
    static void range_check(size_t i, size_t j)
    {
        if (N*i + j >= N*M)
        {
            std::stringstream ss;
            ss << "Cannot access element (" << i << ", " << j
                << ") of " << M << "x" << N << " matrix";
            throw std::out_of_range(ss.str());
        }
    }
};

/// \brief Convenience typedef for column vectors.
template <size_t N> using column_vector = matrix<N, 1>; 
/// \brief Convenience typedef for row vectors.
template <size_t N> using row_vector = matrix<1, N>;
/// \brief Convenience typedef; default vectors are column vectors.
template <size_t N> using vector = column_vector<N>;

/// \brief X basis vector.
const static column_vector<3> unitx(1, 0, 0);
/// \brief Y basis vector.
const static column_vector<3> unity(0, 1, 0);
/// \brief Z basis vector.
const static column_vector<3> unitz(0, 0, 1);

/// \brief Gets the identity vector of a given dimension.
template <size_t M, size_t N> matrix<M, N> identity()
{
    matrix<M, N> ret;
    for (int i = 0; i < M && i < N; ++i)
    {
        ret(i, i) = 1;
    }
    return ret;
}

/// \brief Print a matrix to a std::ostream.
template <size_t M, size_t N>
std::ostream& operator << (std::ostream &os, const matrix<M, N> &m)
{
    os.precision(3);
    os.setf(std::ios::fixed);
    os << "[";
    for (size_t i = 0; i < M; ++i)
    {
        for (size_t j = 0; j < N; ++j)
        {
            os << +m(i, j);
            if (j < N - 1) os << ", ";
        }
        if (i < M - 1) os << "; ";
    }
    os << "]";
}

/// \brief Produces a multiline string representation of a matrix.
template <size_t M, size_t N>
std::string pretty(const matrix<M, N> &m)
{
    std::stringstream ss;
    ss.precision(3);
    ss.setf(std::ios::fixed);

    size_t max_len = 0;
    for (size_t i = 0; i < M*N; ++i)
    {
        std::stringstream tiny;
        tiny.precision(3);
        tiny.setf(std::ios::fixed);
        tiny << m[i];
        size_t len = tiny.tellp();
        if (len > max_len) max_len = len;
    }

    for (size_t r = 0; r < M; ++r)
    {
        ss << "  [";
        for (size_t c = 0; c < N; ++c)
        {
            ss << std::setw(max_len + 1) << m(r, c);
        }
        ss << " ]\n";
    }
    return ss.str();
}

/// \brief Multiplication of a matrix by a scalar.
template <size_t M, size_t N, class T>
matrix<M, N> operator * (const matrix<M, N> &m, T scalar)
{
    auto ret = m;
    for (size_t i = 0; i < M; ++i)
    {
        for (size_t j = 0; j < N; ++j)
        {
            ret(i, j) *= scalar;
        }
    }
    return ret;
}

/// \brief Multiplication of a matrix by a scalar.
template <size_t M, size_t N, class T>
matrix<M, N> operator * (T scalar, const matrix<M, N> &m)
{
    return m*scalar;
}

/// \brief Division of a matrix by a scalar.
template <size_t M, size_t N, class T>
matrix<M, N> operator / (const matrix<M, N> &m, T divisor)
{
    return m*(1.0/divisor);
}

/// \brief Addition of two matrices.
template <size_t M, size_t N>
matrix<M, N> operator + (const matrix<M, N> &left,
                         const matrix<M, N> &right)
{
    matrix<M, N> ret;
    for (size_t i = 0; i < M; ++i)
    {
        for (size_t j = 0; j < N; ++j)
        {
            ret(i, j) = left(i, j) + right(i, j);
        }
    }
    return ret;
}

/// \brief Multiplication of two matrices.
template <size_t M, size_t N, size_t P>
matrix<M, P> operator * (const matrix<M, N> &left,
                         const matrix<N, P> &right)
{
    matrix<M, P> ret;
    for (size_t i = 0; i < M; ++i)
    {
        for (size_t j = 0; j < P; ++j)
        {
            double sum = 0;
            for (size_t k = 0; k < N; ++k)
            {
                sum += left(i, k) * right(k, j);
            }
            ret(i, j) = sum;
        }
    }
    return ret;
}

/// \brief Get the transpose of a matrix.
template <size_t M, size_t N>
matrix<N, M> transpose(const matrix<M, N> &m)
{
    matrix<N, M> ret;
    for (size_t i = 0; i < M; ++i)
    {
        for (size_t j = 0; j < N; ++j)
        {
            ret(j, i) = m(i, j);
        }
    }
    return ret;
}

/// \brief Augment a matrix with another matrix.
template <size_t M, size_t N, size_t P>
matrix<M, N + P> augment(const matrix<M, N> &A,
                         const matrix<M, P> &x)
{
    matrix<M, N + P> ret;
    for (size_t r = 0; r < M; ++r)
    {
        for (size_t c = 0; c < N + P; ++c)
        {
            if (c < N) ret(r, c) = A(r, c);
            else ret(r, c) = x(r, c - N);
        }
    }
    return ret;
}


/// \brief Compute the determinant of a square matrix.
template <size_t N> double det(const matrix<N, N> &m)
{
    double sum = 0;
    for (size_t i = 0; i < N; ++i)
    {
        double top = m(0, i);
        matrix<N-1, N-1> sub;

        for (size_t r = 1; r < N; ++r)
        {
            size_t j = 0;
            for (size_t c = 0; c < N - 1; ++c, ++j)
            {
                if (j == i) ++j;
                sub(r - 1, c) = m.at(r, j);
            }
        }

        double d = det(sub);
        sum += (top*d * (i % 2 == 0 ? 1 : -1));
    }
    return sum;
}

/// \brief Compute the determinant of a 2x2 matrix.
double det(const matrix<2, 2> &m);

/// \brief Compute the determinant of a 1x1 matrix.
double det(const matrix<1, 1> &m);

/// \brief Compute the trace of a square matrix.
template <size_t N> double trace(const matrix<N, N> &m)
{
    double sum = 0;
    for (size_t i = 0; i < N; ++i)
    {
        sum += m(i, i);
    }
    return sum;
}

template <size_t N> bool is_invertible(const matrix<N, N> &m)
{
    return det(m) != 0;
}

/// \brief Compute the inverse of a square matrix.
matrix<2, 2> inverse(const matrix<2, 2> &mat);

/// \brief Compute the inverse of a square matrix.
matrix<1, 1> inverse(const matrix<1, 1> &mat);

/// \brief Compute the inner product of two matrices.
template <size_t M, size_t N>
double inner_product(const matrix<M, N> &left,
                     const matrix<M, N> &right)
{
    double sum = 0;
    for (size_t i = 0; i < M*N; ++i)
    {
        sum += left[i]*right[i];
    }
    return sum;
}

/// \brief Compute a matrix raised to a power.
template <size_t N>
matrix<N, N> pow(const matrix<N, N> &mat, size_t ex)
{
    if (ex < 0) throw std::logic_error("unimplemented");

    if (ex == 0) return identity<N, N>();
    else if (ex == 1) return mat;
    
    matrix<3, 3> ret = mat;
    for (size_t i = 0; i < ex - 1; ++i)
    {
        ret = ret * mat;
    }
    return ret;
}

/// \brief Convenience operator for matrix power.
template <size_t N>
matrix<N, N> operator ^ (const matrix<N, N> &mat, size_t ex)
{
    return pow(mat, ex);
}

/// \brief Compute the cross product of two vectors.
column_vector<3> cross_product(const column_vector<3> &left,
                               const column_vector<3> &right);

/// \brief Compute the cross product of two vectors.
column_vector<3> cross_product(const column_vector<3> &left,
                               const row_vector<3> &right);

/// \brief Compute the cross product of two vectors.
column_vector<3> cross_product(const row_vector<3> &left,
                               const column_vector<3> &right);

/// \brief Compute the cross product of two vectors.
column_vector<3> cross_product(const row_vector<3> &left,
                               const row_vector<3> &right);

/// \brief Compute the skew-symmetric equivalent matrix from
/// a 3D vector.
matrix<3, 3> skew_symmetric(const row_vector<3> &v);

/// \brief Compute the skew-symmetric equivalent matrix from
/// a 3D vector.
matrix<3, 3> skew_symmetric(const column_vector<3> &v);

} // namespace lambda

#endif // LAMBDA_MATRIX_HPP
