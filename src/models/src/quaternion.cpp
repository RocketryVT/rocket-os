// quaternion.cpp

#include <quaternion.hpp>

#include <cmath>
#include <sstream>

void range_check(size_t i)
{
    if (i > 3)
    {
        std::stringstream ss;
        ss << "Cannot access element (" << i
            << ") of quaternion";
        throw std::out_of_range(ss.str());
    }
}

namespace lambda
{

quaternion::quaternion() : _scalar(0),  _vec() { }

quaternion::quaternion(const quaternion &q)
        : _scalar(q.scalar()), _vec(q.vec()) { }

quaternion::quaternion(double scalar, const vector<3> &vec)
        : _scalar(scalar), _vec(vec) { }

quaternion::quaternion(double q1, double q2, double q3, double q4)
        : _scalar(q1), _vec(q2, q3, q4) { }

quaternion::quaternion(const axis_angle &aa)
{
    double angle = aa.angle();
    vector<3> axis = aa.axis();
    _scalar = cos(angle/2);
    _vec[0] = axis[0]*std::sin(angle/2);
    _vec[1] = axis[1]*std::sin(angle/2);
    _vec[2] = axis[2]*std::sin(angle/2);
}

quaternion& quaternion::operator = (const quaternion &q)
{
    _scalar = q.scalar();
    _vec = q.vec();
}

double quaternion::scalar() const
{
    return _scalar;
}

double& quaternion::scalar()
{
    return _scalar;
}

const vector<3>& quaternion::vec() const
{
    return _vec;
}

vector<3>& quaternion::vec()
{
    return _vec;
}

double quaternion::operator [] (size_t index) const
{
    return index == 0 ? _scalar : _vec(index - 1, 0);
}

double& quaternion::operator [] (size_t index)
{
    return index == 0 ? _scalar : _vec(index - 1, 0);
}

double quaternion::at(size_t index) const
{
    range_check(index);
    return index == 0 ? _scalar : _vec(index - 1, 0);
}

double& quaternion::at(size_t index)
{
    range_check(index);
    return index == 0 ? _scalar : _vec(index - 1, 0);
}

std::ostream& operator << (std::ostream &os, const quaternion &q)
{
    os.precision(2);
    os.setf(std::ios::fixed);
    os << "[" << q[0] << ", " << q[1] << ", "
        << q[2] << ", " << q[3] << "]";
    return os;
}

} // namespace lambda

