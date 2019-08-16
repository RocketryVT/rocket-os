#include "catch.hpp"
#include <models>

TEST_CASE("Test runge-kutta 4th-order.", "[runge-kutta]")
{
    // test case found in
    // https://math.okstate.edu/people/yqwang/teaching/math4513_fall11/Notes/rungekutta.pdf

    rvt::ordinary_diff_eq<1> ydot =
    [] (double time, const lambda::column_vector<1> &state)
    {
        return lambda::column_vector<1>(
            state[0] - time*time + 1
        );
    };

    auto exact_solution = [] (double time)
    {
        return lambda::column_vector<1>(
            time*time + 2*time + 1 - 0.5*std::exp(time)
        );
    };

    lambda::column_vector<1> y_initial(0.5);
    double t_initial = 0, t_final = 3.1, t_step = 0.2;

    std::vector<lambda::column_vector<1>> position_history;
    std::vector<double> time_history;

    std::tie(position_history, time_history) =
        rvt::runge_kutta_integration(
            ydot, y_initial, t_initial, t_final, t_step, rvt::rk4);

    std::cout << "TIME, NUMERICAL, EXACT, ERROR" << std::endl;
    for (size_t i = 0; i < position_history.size(); ++i)
    {
        std::cout << std::fixed << std::setprecision(15)
            << time_history[i] << ","
            << std::setprecision(15)
            << position_history[i][0] << ","
            << std::setprecision(15)
            << exact_solution(time_history[i])[0] << ","
            << std::setprecision(15)
            << position_history[i][0] - exact_solution(time_history[i])[0]
            << std::endl;
    }
}
