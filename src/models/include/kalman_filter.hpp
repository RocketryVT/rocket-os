#ifndef LAMBDA_KALMAN_FILTER_HPP
#define LAMBDA_KALMAN_FILTER_HPP

/*! 
    \file
    \brief Defines lambda::kalman_filter,
           a basic kalman filter implementation.
*/

namespace lambda
{

/// \brief A kalman filter implementation.
/// \param M The dimensionality of measurements.
/// \param N The dimensionality of state.
template <size_t M, size_t N> class kalman_filter
{
    public:

    /// \brief Default constructor; zero initializes all members.
    kalman_filter() { }

    /// \brief Constructs a new filter given several 
    ///        members as initialization.
    kalman_filter(const column_vector<N> &_initial_state,
                  const matrix<N, N> &_initial_covariance,
                  const matrix<N, N> &_state_transition,
                  const matrix<N, M> &_measurement_model,
                  const matrix<M, M> &_sensor_noise,
                  const matrix<N, N> &_process_noise) :

        state(_initial_state),
        state_covariance(_initial_covariance),
        state_transition(_state_transition),
        measurement_model(_measurement_model),
        sensor_noise(_sensor_noise),
        process_noise(_process_noise) { }

    /// \brief Propogate the state transition model and return
    ///        the new state estimate.
    const column_vector<N>& predict()
    {
        state = state_transition * state;

        state_covariance = state_transition *
            state_covariance * transpose(state_transition) +
            process_noise;

        return state;
    }

    /// \brief Update the state estimate with a measurement.
    const column_vector<N>& update(const column_vector<M> &meas)
    {
        auto residual = meas - measurement_model*state;

        auto innovation_covariance = sensor_noise +
            measurement_model * state_covariance*
            transpose(measurement_model);

        auto kalman_gain = state_covariance *
            transpose(measurement_model) *
            transpose(innovation_covariance);

        state += kalman_gain * residual;
        
        state_covariance = (identity<N, N>() -
            kalman_gain * measurement_model) *
            state_covariance;

        return state;
    }

    /// \brief The current state estimate.
    column_vector<N> state;
    /// \brief The covariance matrix for the state estimate.
    matrix<N, N> state_covariance;
    /// \brief The model for transition from current to future state.
    matrix<N, N> state_transition;
    /// \brief The model for mapping state to measurements.
    matrix<M, N> measurement_model;
    /// \brief Covariance of sensor noise.
    matrix<M, M> sensor_noise;
    /// \brief Covariance of process noise.
    matrix<N, N> process_noise;
    
};

/// \brief Extended kalman filter implementation. TODO
template <size_t M, size_t N> class extended_kalman_filter { };

/// \brief Unscented kalman filter implementation. TODO
template <size_t M, size_t N> class unscented_kalman_filter { };

/// \brief Convenience typedef for kalman filter.
template <size_t M, size_t N>
using kf = kalman_filter<M, N>;
/// \brief Convenience typedef for extended kalman filter.
template <size_t M, size_t N>
using ekf = extended_kalman_filter<M, N>;
/// \brief Convenience typedef for unscented kalman filter.
template <size_t M, size_t N>
using ukf = unscented_kalman_filter<M, N>;

}

#endif // LAMBDA_KALMAN_FILTER_HPP
