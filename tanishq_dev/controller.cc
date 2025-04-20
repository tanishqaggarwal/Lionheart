#include "tanishq_dev/controller.h"

namespace Lionheart {

template <typename T>
AttitudeRegulationControllerT<T>::AttitudeRegulationControllerT(T kp, T kd)
    : kp(kp), kd(kd) {}

template <typename T>
Vector3<T>
AttitudeRegulationControllerT<T>::update(const Matrix3<T> &attitude_sns,
                                         const Matrix3<T> &attitude_cmd,
                                         const Vector3<T> &angvel_sns) {
    return Vector3<T>{};
}

} // namespace Lionheart