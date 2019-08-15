#include "catch.hpp"
#include <models>

TEST_CASE("Test runge-kutta 4th-order.", "[runge-kutta]")
{
    rvt::ordinary_diff_eq<1> ydot =
    [] (double time, const lambda::column_vector<1> &state)
    {
        return lambda::column_vector<1>(
            std::pow(std::sin(time), 2*state[0]));
    };

    lambda::column_vector<1> y_initial(0);
    double t_initial = 0, t_final = 10, t_step = 0.5;

    std::vector<lambda::column_vector<1>> position_history;
    std::vector<double> time_history;
    std::vector<lambda::column_vector<1>> velocity_history;

    std::tie(position_history, time_history, velocity_history) =
        rvt::runge_kutta_integration(
            ydot, y_initial, t_initial, t_final, t_step, rvt::forward_euler);

    REQUIRE( position_history.size() == time_history.size() );
    REQUIRE( position_history.size() == velocity_history.size() );

    for (size_t i = 0; i < position_history.size(); ++i)
    {
        std::cout << time_history[i] << ","
            << position_history[i][0] << ","
            << std::endl;
    }
}
