#ifndef LAMBDA_AXIS_ANGLE_HPP
#define LAMBDA_AXIS_ANGLE_HPP

#include <cmath>
#include <matrix.hpp>

/*! 
    \file
    \brief Defines lambda::axis_angle, a datatype used to
    represent rotations about an axis. 
*/

namespace lambda
{

class quaternion;

/// \brief A class which implements axis-angles, which
/// represent a rotation about an axis.
class axis_angle
{
    public:

    /// \brief Default constructor.
    axis_angle();

    /// \brief Construct an axis-angle from an axis and angle.
    axis_angle(const column_vector<3> &axis, double angle);

    /// \brief Copy constructor.
    axis_angle(const axis_angle &aa);

    /// \brief Construct an axis-angle from a quaternion.
    axis_angle(const quaternion &q);

    /// \brief Assignment operator.
    axis_angle& operator = (const axis_angle &aa);

    /// \brief Get a const ref to the axis.
    const vector<3>& axis() const;

    /// \brief Get a reference to the axis.
    vector<3>& axis();

    /// \brief Get the angle.
    double angle() const;

    /// \brief Get a reference to the angle.
    double& angle();

    private:

    /// \brief The axis.
    column_vector<3> _axis;
    /// \brief The angle, in radians.
    double _angle;
};

/// \brief Print an axis-angle to a std::ostream.
std::ostream& operator << (std::ostream& os, const axis_angle &aa);

} // namespace lambda

#endif // LAMBDA_AXIS_ANGLE_HPP
