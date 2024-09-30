#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <cmath>
#include <random>

namespace py = pybind11;

// Utility function to wrap angles to [-pi, pi]
double WrapToPi(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

py::array_t<double> differential_kinematics(
    py::array_t<double> state,
    py::array_t<double> velocity,
    double step_time,
    bool noise = false,
    py::array_t<double> alpha = py::array_t<double>({0.03, 0, 0, 0.03})
) {
    auto state_ = state.unchecked<2>();
    auto velocity_ = velocity.unchecked<2>();
    auto alpha_ = alpha.unchecked<1>();

    double phi = state_(2, 0);

    double std_linear = 0, std_angular = 0;
    if (noise) {
        std_linear = std::sqrt(alpha_(0) * std::pow(velocity_(0, 0), 2) + alpha_(1) * std::pow(velocity_(1, 0), 2));
        std_angular = std::sqrt(alpha_(2) * std::pow(velocity_(0, 0), 2) + alpha_(3) * std::pow(velocity_(1, 0), 2));
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> d_linear(0, std_linear);
    std::normal_distribution<> d_angular(0, std_angular);

    double linear = velocity_(0, 0) + (noise ? d_linear(gen) : 0);
    double angular = velocity_(1, 0) + (noise ? d_angular(gen) : 0);

    py::array_t<double> next_state({3, 1});
    auto next_state_ = next_state.mutable_unchecked<2>();

    next_state_(0, 0) = state_(0, 0) + linear * std::cos(phi) * step_time;
    next_state_(1, 0) = state_(1, 0) + linear * std::sin(phi) * step_time;
    next_state_(2, 0) = WrapToPi(state_(2, 0) + angular * step_time);

    return next_state;
}

// Similarly define ackermann_kinematics and omni_kinematics

PYBIND11_MODULE(kinematics, m) {
    m.def("differential_kinematics", &differential_kinematics, "Calculate the next state for a differential wheel robot");
    // Add bindings for ackermann_kinematics and omni_kinematics
}