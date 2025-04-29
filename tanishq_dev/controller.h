#include "tanishq_dev/linalg.h"

namespace Lionheart {
template <typename T> struct AttitudeRegulationControllerT {
    AttitudeRegulationControllerT(T kp, T kd);
    Vector3<T> update(const Matrix3<T> &attitude_sns,
                      const Matrix3<T> &attitude_cmd,
                      const Vector3<T> &angvel_sns);

    T kp;
    T kd;
};

template <typename T> struct PositionRegulationControllerT {
    PositionRegulationControllerT(T kp);
    Vector3<T> update(const Vector3<T> &position_sns,
                      const Vector3<T> &position_cmd);

    T kp;
};

template <typename T>
void solve_thrust_weights(const std::vector<Vector3<T>> thrust_positions,
                          const std::vector<Vector3<T>> thrust_vectors,
                          const Vector3<T> position_cmd,
                          const Vector3<T> torque_cmd,
                          std::vector<T> thrust_weights);

} // namespace Lionheart