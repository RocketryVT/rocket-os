// compute_drag.cpp

#include "compute_drag.hpp"

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
                 const lambda::column_vector<3> &velocity)
{
    double vel_norm = lambda::euclidean_norm(velocity);
    if (vel_norm == 0)
        return {{0, 0, 0}, 0};

    auto unit_vel = lambda::normalize(velocity);
    // q = 0.5*rho*U^2
    double dynamic_pressure = 0.5*density*vel_norm*vel_norm;
    // D = -Cd*A*q
    auto drag_vector = -drag_coefficient*area*
        dynamic_pressure*unit_vel;

    return {drag_vector, dynamic_pressure};
}

