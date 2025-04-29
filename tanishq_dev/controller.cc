#include "controller.h"
#include "least_squares_solver.h"
#include "quaternion.h"

namespace Lionheart {

template <typename T>
AttitudeRegulationControllerT<T>::AttitudeRegulationControllerT(T kp, T kd)
    : kp(kp), kd(kd) {}

template <typename T>
Vector3<T>
AttitudeRegulationControllerT<T>::update(const Matrix3<T> &attitude_sns,
                                         const Matrix3<T> &attitude_cmd,
                                         const Vector3<T> &angvel_sns) {

    const Quat<T> sns = to_quat(attitude_sns);
    const Quat<T> cmd = to_quat(attitude_cmd);

    const Quat<T> err = cmd.product(sns);
    const Vector3<T> p{err.x, err.y, err.z};

    /// 7.7 from Fundamentals of Spacecraft Attitude Determination and Control
    /// by F.Landis Markley, John L. Crassidis.
    return -1 * kp * p - kd * angvel_sns;
}

template <typename T>
PositionRegulationControllerT<T>::PositionRegulationControllerT(T kp)
    : kp(kp) {}

template <typename T>
Vector3<T>
PositionRegulationControllerT<T>::update(const Vector3<T> &position_sns,
                                         const Vector3<T> &position_cmd) {
    const Vector3<T> err = position_cmd - position_sns;
    return kp * err;
}

template <typename T>
void solve_thrust_weights(const std::vector<Vector3<T>> thrust_positions,
                          const std::vector<Vector3<T>> thrust_vectors,
                          const Vector3<T> position_cmd,
                          const Vector3<T> torque_cmd,
                          std::vector<T> thrust_weights) {
    thrust_weights.clear();

    // The thrust weight solver is only hardcoded for 5 thrusters.
    assert(thrust_positions.size() == 5);
    assert(thrust_vectors.size() == 5);

    LeastSquaresSolver::Matrix6x5<T> A;
    for (size_t i = 0; i < 5; ++i) {
        A[0][i] = thrust_positions[i].x;
        A[1][i] = thrust_positions[i].y;
        A[2][i] = thrust_positions[i].z;
        A[3][i] = thrust_vectors[i].x;
        A[4][i] = thrust_vectors[i].y;
        A[5][i] = thrust_vectors[i].z;
    }

    LeastSquaresSolver::Vector6<T> b;
    b[0] = position_cmd.x;
    b[1] = position_cmd.y;
    b[2] = position_cmd.z;
    b[3] = torque_cmd.x;
    b[4] = torque_cmd.y;
    b[5] = torque_cmd.z;

    LeastSquaresSolver::Vector5<T> x;
    LeastSquaresSolver::solve_least_squares(A, b, x);

    for (int i = 0; i < 5; ++i) {
        thrust_weights.emplace_back(i);
    }
}

template struct AttitudeRegulationControllerT<double>;
template struct PositionRegulationControllerT<double>;

} // namespace Lionheart