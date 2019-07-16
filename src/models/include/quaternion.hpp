#ifndef LAMBDA_QUATERNION_HPP
#define LAMBDA_QUATERNION_HPP

#include <sstream>

#include <axis_angle.hpp>

/*! 
    \file
    \brief Defines lambda::quaternion, an elegant way to represent
           rotations without fear of gimbal lock.
*/

namespace lambda
{

/// \brief A class which implement quaternion operations and conversions
/// to and from rotation matrices and axis-angles.
class quaternion
{
    public:

    /// \brief Default constructor.
    quaternion();

    /// \brief Copy constructor.
    quaternion(const quaternion &q);

    /// \brief Constructs a quaternion with a scalar and vector component.
    quaternion(double scalar, const vector<3> &vec);

    /// \brief Constructs a quaternion with a list of numbers.
    quaternion(double q1, double q2, double q3, double q4);

    /// \brief Construct a quaternion from an axis-angle.
    quaternion(const axis_angle &aa);

    /// \brief Assignment operator.
    quaternion& operator = (const quaternion &q);

    /// \brief Get the scalar component.
    double scalar() const;

    /// \brief Get a reference to the scalar component.
    double& scalar();

    /// \brief Get a const ref to the vector component.
    const vector<3>& vec() const;

    /// \brief Get a reference to the vector component.
    vector<3>& vec();

    /// \brief Access an element in the quaternion.
    double operator [] (size_t index) const;

    /// \brief Get a reference to an element.
    double& operator [] (size_t index);

    /// \brief Same as operator [], but throws an exception
    /// if the index is out of bounds.
    double at(size_t index) const;
    
    /// \brief Same as operator [], but throws an exception
    /// if the index is out of bounds.
    double& at(size_t index);

    private:

    /// \brief The scalar component.
    double _scalar;
    /// \brief The vector component.
    vector<3> _vec;
};

/// \brief Print a quaternion to a std::ostream.
std::ostream& operator << (std::ostream &os, const quaternion &q);

} // namespace lambda

#endif // LAMBDA_QUATERNION_H
