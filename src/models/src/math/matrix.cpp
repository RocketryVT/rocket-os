// matrix.cpp

#include <matrix.hpp>
#include <quaternion.hpp>
#include <axis_angle.hpp>

namespace lambda
{

/// \brief Constructs a new 3x3 matrix from a quaternion.
template <> matrix<3, 3>::matrix(const quaternion &q)
    : _data({1 - 2*q[2]*q[2] - 2*q[3]*q[3],
             2*q[1]*q[2] - 2*q[3]*q[0],
             2*q[1]*q[3] + 2*q[2]*q[0],
             2*q[1]*q[2] + 2*q[3]*q[0],
             1 - 2*q[1]*q[1] - 2*q[3]*q[3],
             2*q[2]*q[3] - 2*q[1]*q[0],
             2*q[1]*q[3] - 2*q[2]*q[0],
             2*q[2]*q[3] + 2*q[1]*q[0],
             1 - 2*q[1]*q[1] - 2*q[2]*q[2]}) { }

/// \brief Constructs a new 3x3 matrix from an axis angle. TODO.
template <> matrix<3, 3>::matrix(const axis_angle &aa)
    : _data({0, 0, 0, 0, 0, 0, 0, 0, 0}) { }

/// \brief Computes the determinant of a 2x2 matrix.
double det(const matrix<2, 2> &m)
{
    return m(0,0)*m(1,1) - m(0,1)*m(1,0); 
}

/// \brief Computes the determinant of a 1x1 matrix.
double det(const matrix<1, 1> &m)
{
    return m(0,0);
}

/// \brief Computes the inverse of a 2x2 matrix.
matrix<2, 2> inverse(const matrix<2, 2> &mat)
{
    if (!is_invertible(mat))
    {
        std::stringstream ss;
        ss << "Cannot invert matrix " << mat
            << ", which is singular";
        throw std::domain_error(ss.str());
    }
    return matrix<2, 2>( mat[4], -mat[2],
                        -mat[3],  mat[1])/det(mat);
}

/// \brief Computes the inverse of a 1x1 matrix.
matrix<1, 1> inverse(const matrix<1, 1> &mat)
{
    if (!is_invertible(mat))
    {
        std::stringstream ss;
        ss << "Cannot invert matrix " << mat
            << ", which is singular";
        throw std::domain_error(ss.str());
    }
    return matrix<1, 1>(1/mat[0]);
}

/// \brief Computes the cross product of 2 3D column vectors.
column_vector<3> cross_product(const column_vector<3> &left,
                               const column_vector<3> &right)
{
    return column_vector<3>(
        left[1]*right[2] - left[2]*right[1],
        left[2]*right[0] - left[0]*right[2],
        left[0]*right[1] - left[1]*right[0]);
}

/// \brief Computes the cross product of a column and row vector.
column_vector<3> cross_product(const column_vector<3> &left,
                               const row_vector<3> &right)
{
    return cross_product(left, transpose(right));
}

/// \brief Computes the cross product of a row and column vector.
column_vector<3> cross_product(const row_vector<3> &left,
                               const column_vector<3> &right)
{
    return cross_product(transpose(left), right);
}

/// \brief Computes the cross product of 2 3D row vectors.
column_vector<3> cross_product(const row_vector<3> &left,
                               const row_vector<3> &right)
{
    return cross_product(transpose(left), transpose(right));
}

/// \brief Produces a skew-symmetric matrix from a 3D column vector.
matrix<3, 3> skew_symmetric(const column_vector<3> &v)
{
    return matrix<3, 3>(0, -v[2], v[1],
                        v[2], 0, -v[0],
                        -v[1], v[0], 0);
}

/// \brief Produces a skew-symmetric matrix from a 3D row vector.
matrix<3, 3> skew_symmetric(const row_vector<3> &v)
{
    return skew_symmetric(transpose(v));
}

} // namespace lambda
