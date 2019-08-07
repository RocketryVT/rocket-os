#include "catch.hpp"
#include <lambda>

TEST_CASE("Test runge-kutta forward euler method.", "[runge-kutta]")
{
    rvt::ordinary_diff_eq<1> ydot =
    [] (double time, const lambda::column_vector<1> &state)
    {
        return lambda::column_vector<1>(
            std::pow(std::sin(time), 2*state[0]));
    };

    lambda::column_vector<1> y_initial(0);
    double t_initial = 0, t_final = 5, t_step = 0.5;

    std::vector<lambda::column_vector<1>>
        position_history_1, position_history_2, position_history_3;
    std::vector<double> time_history;
    std::vector<lambda::column_vector<1>> velocity_history;
    std::tie(position_history_1, time_history, velocity_history) =
        rvt::runge_kutta_integration(
            ydot, y_initial, t_initial, t_final, t_step, rvt::forward_euler);

    REQUIRE( position_history_1.size() == time_history.size() );
    REQUIRE( position_history_1.size() == velocity_history.size() );

    std::tie(position_history_2, time_history, velocity_history) =
        rvt::runge_kutta_integration(
            ydot, y_initial, t_initial, t_final, t_step, rvt::default_rk4);

    REQUIRE( position_history_2.size() == time_history.size() );
    REQUIRE( position_history_2.size() == velocity_history.size() );

    std::tie(position_history_3, time_history, velocity_history) =
        rvt::runge_kutta_integration(
            ydot, y_initial, t_initial, t_final, t_step, rvt::dormand_prince);

    REQUIRE( position_history_3.size() == time_history.size() );
    REQUIRE( position_history_3.size() == velocity_history.size() );

    for (size_t i = 0; i < position_history_1.size(); ++i)
    {
        std::cout << time_history[i] << ","
            << position_history_1[i][0] << ","
            << position_history_2[i][0] << ","
            << position_history_3[i][0]
            << std::endl;
    }
}

TEST_CASE("Test runge-kutta 4th order.", "[runge-kutta]")
{

}
