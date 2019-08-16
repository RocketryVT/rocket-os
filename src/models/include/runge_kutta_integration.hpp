// runge_kutta_integration.hpp

#ifndef RVT_RUNGE_KUTTA_INTEGRATION_HPP
#define RVT_RUNGE_KUTTA_INTEGRATION_HPP

namespace rvt
{

template <size_t N>
using ordinary_diff_eq = std::function<lambda::column_vector<N>
    (double, const lambda::column_vector<N>&)>;

template <size_t N> struct runge_kutta_tableau
{
    runge_kutta_tableau(const lambda::matrix<N, N> &a,
                        const lambda::vector<N> &b,
                        const lambda::vector<N> &c);

    const lambda::matrix<N, N> a;
    const lambda::vector<N> b, c;
};

template <size_t N>
runge_kutta_tableau<N>::runge_kutta_tableau(
    const lambda::matrix<N, N> &a_,
    const lambda::vector<N> &b_,
    const lambda::vector<N> &c_) : a(a_), b(b_), c(c_) { }

const runge_kutta_tableau<1> forward_euler({0}, {1}, {0});

const runge_kutta_tableau<2> midpoint_method(
    // A matrix
   {0,   0,
    0.5, 0},
    // B vector
   {0, 1},
    // C vector
   {0, 0.5});

const runge_kutta_tableau<3> rk3(
    // A matrix
   {0,   0, 0,
    0.5, 0, 0,
    0,   1, 0},
    // B vector
   {1.0/6, 4.0/6, 1.0/6},
    // C vector
   {0, 0.5, 1});

const runge_kutta_tableau<4> rk4(
    // A matrix
   {0,   0,   0, 0,
    0.5, 0,   0, 0,
    0,   0.5, 0, 0,
    0,   0,   1, 0},
    // b vector
   {1.0/6, 2.0/6, 2.0/6, 1.0/6},
    // c vector
   {0, 0.5, 0.5, 1});

const runge_kutta_tableau<6> rk45(
    // A matrix
   {0,            0,           0,           0,            0,       0,
    1.0/4,        0,           0,           0,            0,       0,
    3.0/32,       9.0/32,      0,           0,            0,       0,
    1932.0/2197, -7200.0/2197, 7296.0/2197, 0,            0,       0,
    439.0/216,   -8,           3680.0/513, -845.0/4104,   0,       0,
   -8.0/27,       2,          -3544.0/2565, 1859.0/4104, -11.0/40, 0},
    // B vector
   {16.0/135, 0, 6656.0/12825, 28561.0/56430, -9.0/50, 2.0/55},
    // C vector
   {0, 1.0/4, 3.0/8, 12.0/13, 1, 1.0/2});

const runge_kutta_tableau<7> dormand_prince(
    // A matrix
   {0,            0,             0,            0,          0,             0,       0,
    1.0/5,        0,             0,            0,          0,             0,       0,
    3.0/40,       9.0/40,        0,            0,          0,             0,       0,
    44.0/45,      -56.0/15,      32.0/9,       0,          0,             0,       0,
    19372.0/6561, -25360.0/2187, 64448.0/6561, -212.0/729, 0,             0,       0,
    9017.0/3168,  -355.0/33,     46732.0/5247, 49.0/176,   -5103.0/18656, 0,       0,
    35.0/384,     0,             500.0/1113,   125.0/192,  -2187.0/6784,  11.0/84, 0},
    // B vector
   {35.0/384, 0, 500.0/1113, 125.0/192, -2187.0/6784, 11.0/84, 0},
    // C vector
   {0, 0.2, 0.3, 0.8, 8.0/9, 1, 1});

template <size_t M, size_t N>
lambda::vector<M> estimate_next_state(
    double t, double timestep,
    const lambda::vector<M> y,
    const ordinary_diff_eq<M> &f,
    const runge_kutta_tableau<N> &rkt)
{
    std::array<lambda::vector<M>, N> k;

    lambda::vector<M> ret = y;

    for (size_t i = 0; i < N; ++i)
    {
        lambda::vector<M> sum;
        for (size_t j = 0; j < i; ++j)
        {
            sum = sum + rkt.a.at(i, j)*k.at(j);
        }

        lambda::column_vector<M> prev_k;
        if (i > 0) prev_k = k.at(i - 1);

        k[i] = timestep*f(t + rkt.c.at(i)*timestep, y + prev_k*rkt.c.at(i));
        ret = ret + k[i]*rkt.b.at(i, 0);
    }

    return ret;
}


/*
 * Runge-Kutta integration
 * This function uses a 4th order Runge Kutta integration technique to
 * solve a time-dependent differential equation.
 *
 * @arg
 * ydot     - Anonymous Function
 *                Function which returns the time derivative of the state,
 *                which is a Ny x 1 double vector. Function takes the form
 *                    ydot = f(t, y)
 * y0           - Ny x 1 double vector
 *                Initial conditiondefault_rk4
 * tlims        - 2 x 1 double
 *                Integration time limits: [ Initial time; Final time ]
 * h            - double
 *                Time step value for integration
 * @return
 * ykhist       - Ny x Nt double matrix
 *                Time history of state values
 * tkhist       - 1 x Nt double vector
 *                Time history of time values of integration steps
 * ydotkhist    - Ny x Nt double matrix
 *                Time derivative values at each integration step
 *
 * @author: Matt Marti<
 * @date: 2019-04-29
 */



template <size_t M, size_t N>
std::tuple<std::vector<lambda::column_vector<M>>,
           std::vector<double>>
runge_kutta_integration(ordinary_diff_eq<M> &ydot,
    const lambda::column_vector<M> &y0,
    double t_initial, double t_final,
    double timestep,
    const runge_kutta_tableau<N> &tableau)
{
    std::vector<double> time_history =
        lambda::range(t_initial, t_final, timestep);

    const size_t nt = time_history.size();
    std::vector<lambda::column_vector<M>> y_history(nt);
    y_history[0] = y0;

    for (size_t i = 1; i < nt; ++i)
    {
        y_history.at(i) = estimate_next_state(time_history.at(i - 1),
            timestep, y_history.at(i - 1), ydot, tableau);
    }

    return std::make_tuple(y_history, time_history);
}

} // namespace rvt

#endif // RVT_RUNGE_KUTTA_INTEGRATION_HPP
