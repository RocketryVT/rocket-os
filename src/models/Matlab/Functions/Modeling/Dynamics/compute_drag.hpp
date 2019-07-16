// compute_drag.cpp

#ifndef RVT_COMPUTE_DRAG_HPP
#define RVT_COMPUTE_DRAG_HPP

#include <lambda>

namespace rvt
{

/*
 * Computes the force of drag acting on an object
 * 
 * drag_coefficient
 *        Object drag coefficient
 * area
 *        Object reference area
 * density
 *        Ambient density
 * velocity
 *        Object velocity vector
 * 
 * returns:
 * D    - double vector
 *        Drag force vector
 * q    - double
 *        Dynamic Pressure
 * 
 * @author: Matt Marti
 * @date: 2019-05-01
 */

std::pair<lambda::column_vector<3>, double>
    compute_drag(double drag_coefficient, double area, double density,
                 const lambda::column_vector<3> &velocity);

} // namespace rvt

#endif // RVT_COMPUTE_DRAG_HPP
