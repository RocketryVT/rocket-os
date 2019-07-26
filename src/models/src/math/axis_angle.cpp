// axis_angle.cpp

#include <axis_angle.hpp>

#include <quaternion.hpp>
#include <cmath>
#include <matrix.hpp>
#include <norm.hpp>

namespace lambda
{

axis_angle::axis_angle() : _axis(0, 0, 1), _angle(0) { }

axis_angle::axis_angle(const column_vector<3> &axis, double angle)
    : _axis(normalize(axis)), _angle(angle) { }

axis_angle::axis_angle(const axis_angle &aa)
    : _axis(aa.axis()), _angle(aa.angle()) { }

axis_angle::axis_angle(const quaternion &q)
{
    _angle = 2*std::acos(q[0]);
    _axis[0] = q[1]/std::sqrt(1 - q[0]*q[0]);
    _axis[1] = q[2]/std::sqrt(1 - q[0]*q[0]);
    _axis[2] = q[3]/std::sqrt(1 - q[0]*q[0]);
}

axis_angle& axis_angle::operator = (const axis_angle &aa)
{
    _axis = aa.axis();
    _angle = aa.angle();
}

const vector<3>& axis_angle::axis() const
{
    return _axis;
}

vector<3>& axis_angle::axis()
{
    return _axis;
}

double axis_angle::angle() const
{
    return _angle;
}

double& axis_angle::angle()
{
    return _angle;
}

std::ostream& operator << (std::ostream& os, const axis_angle &aa)
{
    os.precision(2);
    os.setf(std::ios::fixed);
    os << "[" << aa.axis() << ", " << aa.angle() << "]";
    return os;
}

} // namespace lambda

