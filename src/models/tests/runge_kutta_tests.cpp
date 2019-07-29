#include "catch.hpp"
#include <lambda>

TEST_CASE("Test runge-kutta forward euler method.", "[runge-kutta]")
{
    auto ydot = [] (double time, const lambda::column_vector<1> &state)
    {
        return lambda::column_vector<1>(state/time);
    };

    rvt::runge_kutta_integration<1, 1>(ydot, {4}, 0, 10, 0.1, rvt::forward_euler);
}

TEST_CASE("Test runge-kutta 4th order.", "[runge-kutta]")
{

}
