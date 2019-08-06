#include "catch.hpp"
#include <lambda>

TEST_CASE("Test runge-kutta forward euler method.", "[runge-kutta]")
{
    rvt::ordinary_diff_eq<2> ydot =
    [] (double time, const lambda::column_vector<2> &state)
    {
        return lambda::column_vector<2>(state/time);
    };

    std::vector<lambda::column_vector<2>> position_history;
    std::vector<double> time_history;
    std::vector<lambda::column_vector<2>> velocity_history;
    std::tie(position_history, time_history, velocity_history) =
        rvt::runge_kutta_integration(ydot, {4, 8}, 0, 10, 0.25, rvt::forward_euler);

    REQUIRE( position_history.size() == time_history.size() );
    REQUIRE( position_history.size() == velocity_history.size() );

    std::cout << "dy/dt = y/t" << "\n============" << std::endl;
    std::cout << "forward euler:" << std::endl;
    for (size_t i = 0; i < position_history.size(); ++i)
    {
        std::cout << time_history[i] << " " << position_history[i]
            << " " << velocity_history[i] << std::endl;
    }

    std::tie(position_history, time_history, velocity_history) =
        rvt::runge_kutta_integration(ydot, {4, 8}, 0, 10, 0.25, rvt::default_rk4);

    REQUIRE( position_history.size() == time_history.size() );
    REQUIRE( position_history.size() == velocity_history.size() );

    std::cout << "default rk4:" << std::endl;
    for (size_t i = 0; i < position_history.size(); ++i)
    {
        std::cout << time_history[i] << " " << position_history[i]
            << " " << velocity_history[i] << std::endl;
    }

    std::tie(position_history, time_history, velocity_history) =
        rvt::runge_kutta_integration(ydot, {4, 8}, 0, 10, 0.25, rvt::dormand_prince);

    REQUIRE( position_history.size() == time_history.size() );
    REQUIRE( position_history.size() == velocity_history.size() );

    std::cout << "dormand prince:" << std::endl;
    for (size_t i = 0; i < position_history.size(); ++i)
    {
        std::cout << time_history[i] << " " << position_history[i]
            << " " << velocity_history[i] << std::endl;
    }
}

TEST_CASE("Test runge-kutta 4th order.", "[runge-kutta]")
{

}
